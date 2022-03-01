/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <hunter_move_base/hunter_move_base.h>
#include <move_base_msgs/RecoveryStatus.h>

#include <angles/angles.h>
#include <cmath>
#include <numeric>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

namespace hunter_move_base
{

    MoveBase::MoveBase(tf2_ros::Buffer &tf) : tf_(tf),
                                              as_(NULL),
                                              planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
                                              bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
                                              blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
                                              recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
                                              costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                              planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
                                              runPlanner_(false), user_goal_reached_(true), setup_(false), p_freq_change_(false), c_freq_change_(false),
                                              new_global_plan_(false), adas_trigger_(false), step_size_(5), buffer_size_(2), predict_time_(4.0), 
											  target_margin_(8), critical_margin_(5), x0_(1.0), x1_(1.5), y0_(0), y1_(0.5), weight_(10.0)
    {

        /* we dont want the hunter drive automatically*/
    	//as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);
        as_pilot_ = new MoveBaseActionServer(ros::NodeHandle(), "AGV_flag", boost::bind(&MoveBase::executeCb, this, _1), false);
        as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::SetGoalPoint, this, _1), false);

        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;

        recovery_trigger_ = PLANNING_R;

        //get some parameters that will be global to the move base node
        std::string global_planner, local_planner;
        private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
        private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
        private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
        private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
        private_nh.param("planner_frequency", planner_frequency_, 20.0);
        private_nh.param("controller_frequency", controller_frequency_, 20.0);
        private_nh.param("planner_patience", planner_patience_, 5.0);
        private_nh.param("controller_patience", controller_patience_, 15.0);
        private_nh.param("max_planning_retries", max_planning_retries_, -1); // disabled by default

        private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
        private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

        // parameters of make_plan service
        private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
        private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

        //set up plan triple buffer
        planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

        //set up the planner's thread
        planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

        odom_helper_.setOdomTopic("odom");

        //for commanding the base
        vel_to_sim_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_adas", 1);
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0);
        global_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("global_goal", 0);

        ros::NodeHandle action_nh("move_base");
        action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
        recovery_status_pub_ = action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

        //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
        //they won't get any useful information back about its status, but this is useful for tools
        //like nav_view and rviz
        ros::NodeHandle simple_nh("move_base_simple");
        goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

        //ros::NodeHandle adas_trigger_nh("adas_trigger");
        //cmd_vel_driver_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&MoveBase::triggerCB, this, _1));
        cmd_vel_driver_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel_driver", 1, boost::bind(&MoveBase::triggerCB, this, _1));

        //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
        private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
        private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
        private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

        private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
        private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
        private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

        //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
        planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
        planner_costmap_ros_->pause();

        //initialize the global planner
        try
        {
            planner_ = bgp_loader_.createInstance(global_planner);
            planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
        }
        catch (const pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
            exit(1);
        }

        //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
        controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
        controller_costmap_ros_->pause();

        //create a local planner
        try
        {
            tc_ = blp_loader_.createInstance(local_planner);
            ROS_INFO("Created local_planner %s", local_planner.c_str());
            tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
        }
        catch (const pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
            exit(1);
        }

        // Start actively updating costmaps based on sensor data
        planner_costmap_ros_->start();
        controller_costmap_ros_->start();

        //advertise a service for getting a plan
        make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

        //advertise a service for clearing the costmaps
        clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if (shutdown_costmaps_)
        {
            ROS_DEBUG_NAMED("move_base", "Stopping costmaps initially");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }

        //load any user specified recovery behaviors, and if that fails load the defaults
        if (!loadRecoveryBehaviors(private_nh))
        {
            loadDefaultRecoveryBehaviors();
        }

        //initially, we'll need to make a plan
        state_ = PLANNING;

        //we'll start executing recovery behaviors at the beginning of our list
        recovery_index_ = 0;

        //we're all set up now so we can start the action server
        as_->start();

        dsrv_ = new dynamic_reconfigure::Server<hunter_move_base::MoveBaseConfig>(ros::NodeHandle("~"));
        dynamic_reconfigure::Server<hunter_move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        costmap_model_ = std::make_shared<base_local_planner::CostmapModel>(*planner_costmap_ros_->getCostmap());

        // load costmap converter for obstacle classifying
        ros::NodeHandle nh_("~/" + name_);

        nh_.param("costmap_converter_plugin", costmap_converter_plugin_, costmap_converter_plugin_);
        nh_.param("odom_topic", odom_topic_, odom_topic_);
        nh_.param("costmap_converter_spin_thread", costmap_converter_spin_thread_, costmap_converter_spin_thread_);
        costmap_ = controller_costmap_ros_->getCostmap();

        //Initialize a costmap to polygon converter
        if (!costmap_converter_plugin_.empty())
        {
            try
            {
                costmap_converter_ = costmap_converter_loader_.createInstance(costmap_converter_plugin_);
                std::string converter_name = costmap_converter_loader_.getName(costmap_converter_plugin_);
                costmap_converter_->setOdomTopic(odom_topic_);
                costmap_converter_->initialize(ros::NodeHandle(nh_, "costmap_converter/" + converter_name));
                costmap_converter_->setCostmap2D(costmap_);

                costmap_converter_->startWorker(ros::Rate(costmap_converter_rate_), costmap_, costmap_converter_spin_thread_);
                ROS_INFO_STREAM("Costmap conversion plugin " << costmap_converter_plugin_ << " loaded.");
            }
            catch (pluginlib::PluginlibException &ex)
            {
                ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
                costmap_converter_.reset();
            }
        }
        else
        {
            ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
        }

        // load visualization tool
        ros::NodeHandle nh_hunter("~/" + std::string("hunter_move_base"));
        vis_ = HunterVisualizationPtr(new HunterVisualization(nh_hunter));
    }

    void MoveBase::reconfigureCB(hunter_move_base::MoveBaseConfig &config, uint32_t level)
    {
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);

        //The first time we're called, we just want to make sure we have the
        //original configuration
        if (!setup_)
        {
            last_config_ = config;
            default_config_ = config;
            setup_ = true;
            return;
        }

        if (config.restore_defaults)
        {
            config = default_config_;
            //if someone sets restore defaults on the parameter server, prevent looping
            config.restore_defaults = false;
        }

        if (planner_frequency_ != config.planner_frequency)
        {
            planner_frequency_ = config.planner_frequency;
            p_freq_change_ = true;
        }

        if (controller_frequency_ != config.controller_frequency)
        {
            controller_frequency_ = config.controller_frequency;
            c_freq_change_ = true;
        }

        planner_patience_ = config.planner_patience;
        controller_patience_ = config.controller_patience;
        max_planning_retries_ = config.max_planning_retries;
        conservative_reset_dist_ = config.conservative_reset_dist;

        recovery_behavior_enabled_ = config.recovery_behavior_enabled;
        clearing_rotation_allowed_ = config.clearing_rotation_allowed;
        shutdown_costmaps_ = config.shutdown_costmaps;

        oscillation_timeout_ = config.oscillation_timeout;
        oscillation_distance_ = config.oscillation_distance;
        if (config.base_global_planner != last_config_.base_global_planner)
        {
            boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
            //initialize the global planner
            ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
            try
            {
                planner_ = bgp_loader_.createInstance(config.base_global_planner);

                // wait for the current planner to finish planning
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

                // Clean up before initializing the new planner
                planner_plan_->clear();
                latest_plan_->clear();
                controller_plan_->clear();
                resetState();
                planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

                lock.unlock();
            }
            catch (const pluginlib::PluginlibException &ex)
            {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s",
                          config.base_global_planner.c_str(), ex.what());
                planner_ = old_planner;
                config.base_global_planner = last_config_.base_global_planner;
            }
        }

        if (config.base_local_planner != last_config_.base_local_planner)
        {
            boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
            //create a local planner
            try
            {
                tc_ = blp_loader_.createInstance(config.base_local_planner);
                // Clean up before initializing the new planner
                planner_plan_->clear();
                latest_plan_->clear();
                controller_plan_->clear();
                resetState();
                tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
            }
            catch (const pluginlib::PluginlibException &ex)
            {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s",
                          config.base_local_planner.c_str(), ex.what());
                tc_ = old_planner;
                config.base_local_planner = last_config_.base_local_planner;
            }
        }

        make_plan_clear_costmap_ = config.make_plan_clear_costmap;
        make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

        last_config_ = config;
    }

    void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal)
    {
        ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
        move_base_msgs::MoveBaseActionGoal action_goal;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.target_pose = *goal;

        action_goal_pub_.publish(action_goal);
    }

    void MoveBase::triggerCB(const geometry_msgs::Twist::ConstPtr &twist_ptr)
    {
        if (twist_ptr == NULL ||
            (std::fabs(twist_ptr->linear.x) < DBL_EPSILON &&
             std::fabs(twist_ptr->angular.z) < DBL_EPSILON) ||
            twist_ptr->linear.x < DBL_EPSILON)
        {
            adas_trigger_ = false;
        }
        else
        {
            adas_trigger_ = true;
        }

        driver_cmd_.linear.x = twist_ptr->linear.x;
        driver_cmd_.linear.y = twist_ptr->linear.y;
        driver_cmd_.angular.z = twist_ptr->angular.z;
    }

    void MoveBase::clearCostmapWindows(double size_x, double size_y)
    {
        geometry_msgs::PoseStamped global_pose;

        //clear the planner's costmap
        getRobotPose(global_pose, planner_costmap_ros_);

        std::vector<geometry_msgs::Point> clear_poly;
        double x = global_pose.pose.position.x;
        double y = global_pose.pose.position.y;
        geometry_msgs::Point pt;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

        //clear the controller's costmap
        getRobotPose(global_pose, controller_costmap_ros_);

        clear_poly.clear();
        x = global_pose.pose.position.x;
        y = global_pose.pose.position.y;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
    }

    bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
    {
        //clear the costmaps
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
        controller_costmap_ros_->resetLayers();

        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
        planner_costmap_ros_->resetLayers();
        return true;
    }

    bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
    {
        if (as_->isActive())
        {
            ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
            return false;
        }

        //make sure we have a costmap for our planner
        if (planner_costmap_ros_ == NULL)
        {
            ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
            return false;
        }

        geometry_msgs::PoseStamped start;
        //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
        if (req.start.header.frame_id.empty())
        {
            geometry_msgs::PoseStamped global_pose;
            if (!getRobotPose(global_pose, planner_costmap_ros_))
            {
                ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
                return false;
            }
            start = global_pose;
        }
        else
        {
            start = req.start;
        }

        if (make_plan_clear_costmap_)
        {
            //update the copy of the costmap the planner uses
            clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
        }

        //first try to make a plan to the exact desired goal
        std::vector<geometry_msgs::PoseStamped> global_plan;
        if (!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty())
        {
            ROS_DEBUG_NAMED("move_base", "Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
                            req.goal.pose.position.x, req.goal.pose.position.y);

            //search outwards for a feasible goal within the specified tolerance
            geometry_msgs::PoseStamped p;
            p = req.goal;
            bool found_legal = false;
            float resolution = planner_costmap_ros_->getCostmap()->getResolution();
            float search_increment = resolution * 3.0;
            if (req.tolerance > 0.0 && req.tolerance < search_increment)
                search_increment = req.tolerance;
            for (float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment)
            {
                for (float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment)
                {
                    for (float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment)
                    {

                        //don't search again inside the current outer layer
                        if (x_offset < max_offset - 1e-9 && y_offset < max_offset - 1e-9)
                            continue;

                        //search to both sides of the desired goal
                        for (float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0)
                        {

                            //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
                            if (y_offset < 1e-9 && y_mult < -1.0 + 1e-9)
                                continue;

                            for (float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0)
                            {
                                if (x_offset < 1e-9 && x_mult < -1.0 + 1e-9)
                                    continue;

                                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                                if (planner_->makePlan(start, p, global_plan))
                                {
                                    if (!global_plan.empty())
                                    {

                                        if (make_plan_add_unreachable_goal_)
                                        {
                                            //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                                            //(the reachable goal should have been added by the global planner)
                                            global_plan.push_back(req.goal);
                                        }

                                        found_legal = true;
                                        ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                        break;
                                    }
                                }
                                else
                                {
                                    ROS_DEBUG_NAMED("move_base", "Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                }
                            }
                        }
                    }
                }
            }
        }

        //copy the plan into a message to send out
        resp.plan.poses.resize(global_plan.size());
        for (unsigned int i = 0; i < global_plan.size(); ++i)
        {
            resp.plan.poses[i] = global_plan[i];
        }
        return true;
    }

    MoveBase::~MoveBase()
    {
        recovery_behaviors_.clear();

        delete dsrv_;

        if (as_ != NULL)
            delete as_;

        if (planner_costmap_ros_ != NULL)
            delete planner_costmap_ros_;

        if (controller_costmap_ros_ != NULL)
            delete controller_costmap_ros_;

        planner_thread_->interrupt();
        planner_thread_->join();

        delete planner_thread_;

        delete planner_plan_;
        delete latest_plan_;
        delete controller_plan_;

        planner_.reset();
        tc_.reset();
    }

    bool MoveBase::makePlan(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

        //make sure to set the plan to be empty initially
        plan.clear();

        //since this gets called on handle activate
        if (planner_costmap_ros_ == NULL)
        {
            ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
            return false;
        }

        //get the starting pose of the robot
        geometry_msgs::PoseStamped global_pose;
        if (!getRobotPose(global_pose, planner_costmap_ros_))
        {
            ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
            return false;
        }

        const geometry_msgs::PoseStamped &start = global_pose;

        //if the planner fails or returns a zero length plan, planning failed
        if (!planner_->makePlan(start, goal, plan) || plan.empty())
        {
            ROS_DEBUG_NAMED("move_base", "Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
            return false;
        }

        return true;
    }

    void MoveBase::publishZeroVelocity()
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }

    void MoveBase::publishDefaultVelocityToSimulator()
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 999.0;
        cmd_vel.linear.y = 999.0;
        cmd_vel.angular.z = 999.0;
        vel_to_sim_pub_.publish(cmd_vel);
    }

    void MoveBase::publishZeroVelocityToSimulator()
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_to_sim_pub_.publish(cmd_vel);
    }

    bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion &q)
    {
        //first we need to check if the quaternion has nan's or infs
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
        {
            ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
            return false;
        }

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

        //next, we need to check if the length of the quaternion is close to zero
        if (tf_q.length2() < 1e-6)
        {
            ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
            return false;
        }

        //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
        tf_q.normalize();

        tf2::Vector3 up(0, 0, 1);

        double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

        if (fabs(dot - 1) > 1e-3)
        {
            ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
            return false;
        }

        return true;
    }

    geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg)
    {
        std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
        geometry_msgs::PoseStamped goal_pose, global_pose;
        goal_pose = goal_pose_msg;

        //just get the latest available transform... for accuracy they should send
        //goals in the frame of the planner
        goal_pose.header.stamp = ros::Time();

        try
        {
            tf_.transform(goal_pose_msg, global_pose, global_frame);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                     goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
            return goal_pose_msg;
        }

        return global_pose;
    }

    void MoveBase::wakePlanner(const ros::TimerEvent &event)
    {
        // we have slept long enough for rate
        planner_cond_.notify_one();
    }

    void MoveBase::planThread()
    {
        ROS_DEBUG_NAMED("move_base_plan_thread", "Starting planner thread...");
        ros::NodeHandle n;
        ros::Timer timer;
        bool wait_for_wake = false;
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        ROS_WARN("Oscar::Start Move Base.");
        while (n.ok())
        {
            //check if we should run the planner (the mutex is locked)
            while (wait_for_wake || !runPlanner_)
            {
                ros::Time start_time_ = ros::Time::now();
                if (user_goal_reached_)
                {
                    //ROS_WARN("Oscar***We are here when no user goal.");
                    if (!planner_costmap_ros_ || !costmap_model_)
                    {
                        //ROS_WARN("Oscar::Planner_costmap_ros has not been built. Continue.");
                        ros::Duration(3).sleep();
                        continue;
                    };

                    ros::Time start_time_ADAS = ros::Time::now();

                    // classify and visualize the obstacles
                    Obstacles_.clear();
                    if (costmap_converter_)
                    {
                        ClassifyObstacles();
                    }

                    //vis_->publishObstacles(Obstacles_, ALL);

                    if (!adas_trigger_)
                    {
                        //ROS_WARN("Oscar::Waiting for Human Driver.");
                        publishDefaultVelocityToSimulator();
                        if (planner_frequency_ > 0)
                        {
                            sleepPlanner(n, timer, start_time_ADAS, lock);
                        }
                        continue;
                    }

                    state_ = CONTROLLING;

                    geometry_msgs::PoseStamped current_robot_pose;
                    planner_costmap_ros_->getRobotPose(current_robot_pose);
                    robot_pose_ = PoseSE2(current_robot_pose.pose);
                    ROS_WARN("Oscar::The robot pose is:%f,%f,%f", robot_pose_.x(), robot_pose_.y(), robot_pose_.theta());

                    geometry_msgs::PoseStamped robot_velo_tf;
                    odom_helper_.getRobotVel(robot_velo_tf);

                    robot_velo_.linear.x = robot_velo_tf.pose.position.x;
                    robot_velo_.linear.y = robot_velo_tf.pose.position.y;
                    robot_velo_.angular.z = tf2::getYaw(robot_velo_tf.pose.orientation);
                    //ROS_WARN("Oscar::The velo is:%.3f,%.3f,%.3f", driver_cmd_.linear.x, driver_cmd_.linear.y, driver_cmd_.angular.z);

                    geometry_msgs::PoseStamped temp_goal_pose = current_robot_pose;
                    geometry_msgs::PoseStamped predicted_goal_pose;
                    std::vector<geometry_msgs::PoseStamped> pose_list;

                    bool assistant_driving = false;
                    int time_interval = step_size_ * predict_time_;
					//ROS_WARN("Oscar::time interval is:%d", time_interval);

                    // predict 3 seconds ahead
                    for (int i = 0; i < time_interval; ++i)
                    {
                        temp_goal_pose = ComputeNewPosition(temp_goal_pose, driver_cmd_, 1.0 / step_size_);
                        pose_list.push_back(temp_goal_pose);
                    }

					// to be deledted
					const PoseSE2 predict_pose = PoseSE2(pose_list.back().pose);
                    ROS_WARN("Oscar::The predicted pose is:%f,%f,%f", predict_pose.x(), predict_pose.y(), predict_pose.theta());
					//

                    //boundary of interest of obstacles
                    unsigned int x_max, x_min, y_max, y_min;
                    for (auto it = pose_list.begin(); it != pose_list.end(); ++it)
                    {
                        double px = (*it).pose.position.x;
                        double py = (*it).pose.position.y;
                        double central_cost = -255.0;
                        unsigned int cell_x, cell_y;
                        if (!costmap_->worldToMap(px, py, cell_x, cell_y))
                        {
                            //we're off the map
                            ROS_WARN("Oscar::Off Map %f, %f", px, py);
                            central_cost = costmap_2d::NO_INFORMATION;
                        }

                        if (it == pose_list.begin())
                        {
                            x_max = cell_x;
                            x_min = cell_x;
                            y_max = cell_y;
                            y_min = cell_y;
                        }
                        SetBoundary(cell_x, cell_y, x_max, x_min, y_max, y_min);
                    }

                    Padding(x_max, x_min, y_max, y_min);

                    //ROS_WARN("Oscar::the boundaries are %d, %d, %d, %d", x_min, x_max, y_min, y_max);

					Obstacles_human_path_.clear();
                    if (!ObstaclesOfInterest(x_max, x_min, y_max, y_min))
                    {
                        ROS_WARN("Oscar::Next Cycle.");
                        publishDefaultVelocityToSimulator();
                        continue;
                    }

					vis_->publishObstacles(Obstacles_human_path_, HUMAN);

					std::vector<double> force_rej;
					bool safe_stop = false;
					bool power_steering = false;

					if (safe_stop_vec_.size() > (buffer_size_ - 1))
					{
						safe_stop_vec_.pop_front();
					}

					if (power_steering_vec_.size() > (buffer_size_ - 1))
					{
						power_steering_vec_.pop_front();
					}

					std::vector<double> force_human_vec;

                    for (auto it = pose_list.begin(); it != pose_list.end(); ++it)
                    {
                        double px = (*it).pose.position.x;
                        double py = (*it).pose.position.y;
                        double theta = angles::normalize_angle(tf2::getYaw((*it).pose.orientation));
                        double central_cost = -255.0;
                        double footprint_cost = 255.0;
                        if (px == robot_pose_.x() && py == robot_pose_.y())
                        {
                            ROS_WARN("Oscar::there is no velocity at all, skip safety check.");
                            break;
                        }

						if ((it - pose_list.begin()) <= target_margin_)
						{
		                    std::vector<geometry_msgs::Point> footprint = planner_costmap_ros_->getRobotFootprint();
		                    if (footprint.size() < 3)
		                    {
		                        unsigned int cell_x, cell_y;
		                        if (!planner_costmap_ros_->getCostmap()->worldToMap(px, py, cell_x, cell_y))
		                        {
		                            //we're off the map
		                            ROS_WARN("Oscar::Off Map %f, %f", px, py);
		                            central_cost = costmap_2d::NO_INFORMATION;
		                            //continue;
		                        }
		                        central_cost = planner_costmap_ros_->getCostmap()->getCost(cell_x, cell_y);
		                    }
		                    else
		                    {
		                        /*double cos_th = cos(theta);
		                        double sin_th = sin(theta);
		                        std::vector<geometry_msgs::Point> oriented_footprint;
		                        for(unsigned int i = 0; i < footprint.size(); ++i){
		                        geometry_msgs::Point new_pt;
		                        new_pt.x = px + (footprint[i].x * cos_th - footprint[i].y * sin_th);
		                        new_pt.y = py + (footprint[i].x * sin_th + footprint[i].y * cos_th);
		                        oriented_footprint.push_back(new_pt);
		                        }

		                        geometry_msgs::Point robot_position;
		                        robot_position.x = px;
		                        robot_position.y = py;

		                        double robot_inscribed_radius = 0.0; 
		                        double robot_circumscribed_radius = 0.0;
		                        costmap_2d::calculateMinAndMaxDistances(footprint, robot_inscribed_radius, robot_circumscribed_radius);*/
		                        footprint_cost = costmap_model_->footprintCost(px, py, theta, footprint);
		                    }

		                    double critical_cost = 1.0; //1.0 * costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

		                    //ROS_WARN("Oscar::The new predicted robot pose is: %f, %f", px, py);
		                    //ROS_WARN("Oscar::The cell cost and critical cost is:  %f, %f", cell_cost, critical_cost);

		                    if (robot_velo_.linear.x > 10.0)
		                    {
		                        double delta_dist = LinearInterpolation(robot_velo_.linear.x);
		                        critical_cost = ComputeCost(delta_dist);
		                    }

		                    if (central_cost >= critical_cost || footprint_cost < 0.0)
		                    {
								ROS_WARN("Oscar::The goal point is: %d", int(it - pose_list.begin()));
		                        //ROS_WARN("Oscar::The velo is:%.3f,%.3f,%.3f", robot_velo_.linear.x, robot_velo_.linear.y, robot_velo_.angular.z);
		                        //ROS_WARN("Oscar::Px and Py and Index is:%f, %f, %d", px, py, (int)(it - pose_list.begin()));
		                        //ROS_WARN("Oscar:::::::The cell cost and critical cost is:%f, %f", footprint_cost, critical_cost);
		                        //ROS_WARN("Oscar::the goal is not valid, need driving assistance.");
		                        safe_stop_vec_.push_back(true);
								//power_steering_vec_.push_back(true);
								if ((it - pose_list.begin()) <= critical_margin_)
								{
									predicted_goal_pose = current_robot_pose;
		                            ROS_WARN("Oscar::Too close! Stop at current position.");
								}
								else
								{
									int margin = it - pose_list.begin() - critical_margin_;
									std::advance(it, -std::min(margin, 2));
									bool initial_flag = (std::fabs(prev_goal_pose_.pose.position.x) < DBL_EPSILON) && (std::fabs(prev_goal_pose_.pose.position.y) < DBL_EPSILON);
									if (UpdateGoalPose(*it) || initial_flag)
									{
										predicted_goal_pose = *(it);
		                                ROS_WARN("Oscar::Goal Point has been changed, select %d point as the goal: %f, %f", int(it - pose_list.begin()), predicted_goal_pose.pose.position.x,
																															predicted_goal_pose.pose.position.y);
									}
									else
									{
										predicted_goal_pose = prev_goal_pose_;
		                                ROS_WARN("Oscar::Same Goal point as before.");
									}						
								}
								safe_stop = true;
		                        prev_goal_pose_ = predicted_goal_pose;
								break;
		                    }
						}

						double force_rej = 0.0;
						CalculateAPF((*it).pose, force_rej);

						// need to start power steering process
						/*if (std::fabs(force_rej) > 1.0 && !power_steering)
						{
							power_steering = true;
							ROS_WARN("Oscar::Need power steering.");
						}
						*/

						double force_rej_weighted = force_rej * std::pow(decay_rate_, int(it - pose_list.begin()));
						ROS_WARN("Oscar::force_rej_weighted:%f, weight:%f", force_rej_weighted, std::pow(decay_rate_, int(it - pose_list.begin())));
						force_human_vec.push_back(force_rej_weighted);
						// APF calculated

						// if the loop does not break, then safe stop is not needed.
						if (it == (pose_list.end() - 1))
						{
							safe_stop_vec_.push_back(false);
						}

						/* we change the rule of triggering power steering.Do it outside of the loop.
						if (it == (pose_list.end() - 1))
						{
							safe_stop_vec_.push_back(false);
							if (power_steering)
							{
								power_steering_vec_.push_back(true);
							}
							else
							{
								power_steering_vec_.push_back(false);
							}
						}
						*/
                    }

					// calculate average apf value of human path
					double force_human_sum = 0.0;
					double force_human_avg = 0.0;					
					if (int(force_human_vec.size()) > 0)
					{		            
						for (auto it = force_human_vec.begin(); it != force_human_vec.end(); ++it)
						{
							force_human_sum += *it;
						}
						force_human_avg = force_human_sum / force_human_vec.size();		
					}

					if (std::fabs(force_human_avg) > 1.0)
					{
						power_steering = true;
					}

					power_steering_vec_.push_back(power_steering);

					// Start to evaluate whether ADAS is needed.
					int long_index = 0;
					int lat_index = 0;
					
					for (auto it = safe_stop_vec_.begin(); it != safe_stop_vec_.end(); ++it)
					{
						if ((*it) != 0)
						{
							long_index += 1;
						}
					}


					for (auto it = power_steering_vec_.begin(); it != power_steering_vec_.end(); ++it)
					{
						if (*(it) != 0)
						{
							lat_index += 1;
						}
					}

					ROS_WARN("The long index is %d, %d, The lat index is: %d, %d", safe_stop_vec_.front(), safe_stop_vec_.back(), power_steering_vec_.front(), power_steering_vec_.back());

					if (long_index > 0)
					{
						if (safe_stop)
						{
							planner_goal_ = predicted_goal_pose;					
						}
						else
						{
							planner_goal_ = prev_goal_pose_;
						}
                        ROS_WARN("Oscar::LONG->The planner goal pose is:%f,%f", planner_goal_.pose.position.x, planner_goal_.pose.position.y);
					}
					else if (lat_index > 0)
					{
						planner_goal_ = global_goal_;
						ROS_WARN("Oscar::LAT->The planner goal pose is:%f, %f", planner_goal_.pose.position.x, planner_goal_.pose.position.y);
					}
					else
					{
						ROS_WARN("Oscar::Driver is doing good, well play, the final pose is:%f,%f", pose_list.back().pose.position.x, pose_list.back().pose.position.y);
                        publishDefaultVelocityToSimulator();
                        if (planner_frequency_ > 0)
                        {
                            sleepPlanner(n, timer, start_time_ADAS, lock);
                        }
						continue;
					}

                    ROS_WARN("Oscar::Start Plan.");

                    if (shutdown_costmaps_)
                    {
                        ROS_DEBUG_NAMED("move_base", "Starting up costmaps that were shut down previously");
                        planner_costmap_ros_->start();
                        controller_costmap_ros_->start();
                    }

                    last_valid_control_ = ros::Time::now();
                    last_valid_plan_ = ros::Time::now();
                    last_oscillation_reset_ = ros::Time::now();
                    planning_retries_ = 0;

                    geometry_msgs::PoseStamped temp_goal = planner_goal_;
                    //ROS_WARN("Oscar::ADAS Start to Plan.");

                    //run planner
                    planner_plan_->clear();
                    bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

                    if (gotPlan)
                    {
                        //ROS_WARN("Oscar::move_base_plan_thread, Got Plan with %zu points!", planner_plan_->size());
                        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
                        std::vector<geometry_msgs::PoseStamped> *temp_plan = planner_plan_;

                        planner_plan_ = latest_plan_;
                        latest_plan_ = temp_plan;
                        last_valid_plan_ = ros::Time::now();
                        planning_retries_ = 0;
                        new_global_plan_ = true;

                        //ROS_WARN("Oscar::move_base_plan_thread, Generated a plan from the base_global_planner");
                    }
                    //if we didn't get a plan and we are in the planning state (the robot isn't moving)
                    else
                    {
                        ROS_WARN("Oscar::move_base_plan_thread, No Plan...");
                        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

                        //check if we've tried to make a plan for over our time limit or our maximum number of retries
                        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
                        //is negative (the default), it is just ignored and we have the same behavior as ever
                        planning_retries_++;
                        if (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))
                        {
                            //we'll move into our obstacle clearing mode
                            state_ = CLEARING;
                            publishZeroVelocityToSimulator();
                            recovery_trigger_ = PLANNING_R;
                        }
                    }

                    //for timing that gives real time even in simulation
                    ros::WallTime start = ros::WallTime::now();

                    //the real work on pursuing a goal is done here
					std::vector<geometry_msgs::Pose> local_path;
                    bool done = executeCycle(local_path, force_human_vec,predict_time_, long_index, lat_index);

                    //check if execution of the goal has completed in some way

                    ros::WallDuration t_diff = ros::WallTime::now() - start;
                    ROS_WARN("Oscar::move_base, Full control cycle time: %.9f\n", t_diff.toSec());

                    //setup sleep interface if needed
                    if (planner_frequency_ > 0)
                    {
                        sleepPlanner(n, timer, start_time_ADAS, lock);
                    }

                    // Plan next cycle for Virtual Goal
                    //adas_trigger_ = false;
                    continue;
                }

                ROS_WARN("Oscar***User goal haven't reached.");

                //if we should not be running the planner then suspend this thread
                ROS_DEBUG_NAMED("move_base_plan_thread", "Planner thread is suspending");
                planner_cond_.wait(lock);
                wait_for_wake = false;
            }
            ros::Time start_time = ros::Time::now();

            //time to plan! get a copy of the goal and unlock the mutex
            geometry_msgs::PoseStamped temp_goal = planner_goal_;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base_plan_thread", "Planning...");

            //run planner
            planner_plan_->clear();
            bool n_ok = n.ok();
            bool makePlann = makePlan(temp_goal, *planner_plan_);
            bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

            if (gotPlan)
            {
                ROS_DEBUG_NAMED("move_base_plan_thread", "Got Plan with %zu points!", planner_plan_->size());
                //pointer swap the plans under mutex (the controller will pull from latest_plan_)
                std::vector<geometry_msgs::PoseStamped> *temp_plan = planner_plan_;

                lock.lock();
                planner_plan_ = latest_plan_;
                latest_plan_ = temp_plan;
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                new_global_plan_ = true;

                ROS_DEBUG_NAMED("move_base_plan_thread", "Generated a plan from the base_global_planner");

                //make sure we only start the controller if we still haven't reached the goal
                if (runPlanner_)
                    state_ = CONTROLLING;
                if (planner_frequency_ <= 0)
                    runPlanner_ = false;
                lock.unlock();
            }
            //if we didn't get a plan and we are in the planning state (the robot isn't moving)
            else if (state_ == PLANNING)
            {
                ROS_DEBUG_NAMED("move_base_plan_thread", "No Plan...");
                ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

                //check if we've tried to make a plan for over our time limit or our maximum number of retries
                //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
                //is negative (the default), it is just ignored and we have the same behavior as ever
                lock.lock();
                planning_retries_++;
                if (runPlanner_ &&
                    (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_)))
                {
                    //we'll move into our obstacle clearing mode
                    state_ = CLEARING;
                    runPlanner_ = false; // proper solution for issue #523
                    publishZeroVelocity();
                    recovery_trigger_ = PLANNING_R;
                }
                lock.unlock();
            }

            //take the mutex for the next iteration
            lock.lock();

            //setup sleep interface if needed
            if (planner_frequency_ > 0)
            {
                double time_interval = ros::Time::now().toSec() - start_time.toSec();
                ROS_WARN("Oscar::The time of planthread is:%.9f", time_interval);
                ros::Duration sleep_time = (start_time + ros::Duration(1.0 / planner_frequency_)) - ros::Time::now();
                //ROS_WARN("OScar::The planner frequency and sleep_time is:, %.3lf, %.9f", planner_frequency_, sleep_time.toSec());
                if (sleep_time > ros::Duration(0.0))
                {
                    wait_for_wake = true;
                    timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
                }
            }
        }
    }

    void MoveBase::SetGoalPoint(const move_base_msgs::MoveBaseGoalConstPtr &move_base_goal)
    {
        if (!isQuaternionValid(move_base_goal->target_pose.pose.orientation))
        {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
        }

        global_goal_ = goalToGlobalFrame(move_base_goal->target_pose);
        global_goal_pub_.publish(global_goal_);
        as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Successfully Set Global Goal.");
    }

    void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr &move_base_goal)
    {
        ROS_ERROR("111111");

        if (!isQuaternionValid(move_base_goal->target_pose.pose.orientation))
        {
            ROS_ERROR("222222");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
        }

        return;

        geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

        publishZeroVelocity();
        //we have a goal so start the planner
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        planner_goal_ = goal;
        runPlanner_ = true;
        user_goal_reached_ = false;
        planner_cond_.notify_one();
        lock.unlock();

        current_goal_pub_.publish(goal);

        ros::Rate r(controller_frequency_);
        if (shutdown_costmaps_)
        {
            ROS_DEBUG_NAMED("move_base", "Starting up costmaps that were shut down previously");
            planner_costmap_ros_->start();
            controller_costmap_ros_->start();
        }

        //we want to make sure that we reset the last time we had a valid plan and control
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;

        ros::NodeHandle n;
        while (n.ok())
        {
            if (c_freq_change_)
            {
                ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
                r = ros::Rate(controller_frequency_);
                c_freq_change_ = false;
            }

            //ROS_WARN("Oscar::Control frequency is : %.9f", controller_frequency_);

            if (as_->isPreemptRequested())
            {
                if (as_->isNewGoalAvailable())
                {
                    //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
                    move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

                    if (!isQuaternionValid(new_goal.target_pose.pose.orientation))
                    {
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
                        return;
                    }

                    goal = goalToGlobalFrame(new_goal.target_pose);

                    //we'll make sure that we reset our state for the next execution cycle
                    recovery_index_ = 0;
                    state_ = PLANNING;

                    //we have a new goal so make sure the planner is awake
                    lock.lock();
                    planner_goal_ = goal;
                    runPlanner_ = true;
                    planner_cond_.notify_one();
                    lock.unlock();

                    //publish the goal point to the visualizer
                    ROS_DEBUG_NAMED("move_base", "move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
                    current_goal_pub_.publish(goal);

                    //make sure to reset our timeouts and counters
                    last_valid_control_ = ros::Time::now();
                    last_valid_plan_ = ros::Time::now();
                    last_oscillation_reset_ = ros::Time::now();
                    planning_retries_ = 0;
                }
                else
                {
                    //if we've been preempted explicitly we need to shut things down
                    resetState();

                    //notify the ActionServer that we've successfully preempted
                    ROS_DEBUG_NAMED("move_base", "Move base preempting the current goal");
                    as_->setPreempted();

                    //we'll actually return from execute after preempting
                    return;
                }
            }

            //we also want to check if we've changed global frames because we need to transform our goal pose
            if (goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID())
            {
                goal = goalToGlobalFrame(goal);

                //we want to go back to the planning state for the next execution cycle
                recovery_index_ = 0;
                state_ = PLANNING;

                //we have a new goal so make sure the planner is awake
                lock.lock();
                planner_goal_ = goal;
                runPlanner_ = true;
                planner_cond_.notify_one();
                lock.unlock();

                //publish the goal point to the visualizer
                ROS_DEBUG_NAMED("move_base", "The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
                current_goal_pub_.publish(goal);

                //make sure to reset our timeouts and counters
                last_valid_control_ = ros::Time::now();
                last_valid_plan_ = ros::Time::now();
                last_oscillation_reset_ = ros::Time::now();
                planning_retries_ = 0;
            }

            //for timing that gives real time even in simulation
            ros::WallTime start = ros::WallTime::now();

            //the real work on pursuing a goal is done here
            bool done = executeCycle(goal);

            //if we're done, then we'll return from execute
            if (done)
                return;

            //check if execution of the goal has completed in some way

            ros::WallDuration t_diff = ros::WallTime::now() - start;
            ROS_WARN("Oscar::Full control cycle time: %.9f\n", t_diff.toSec());
            ROS_DEBUG_NAMED("move_base", "Full control cycle time: %.9f\n", t_diff.toSec());

            r.sleep();
            //make sure to sleep for the remainder of our cycle time
            if (r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
                ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
        }

        //wake up the planner thread so that it can exit cleanly
        lock.lock();
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //if the node is killed then we'll abort and return
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
        return;
    }

    double MoveBase::distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
    {
        return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
    }

    bool MoveBase::executeCycle(geometry_msgs::PoseStamped &goal)
    {
        boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
        //we need to be able to publish velocity commands
        geometry_msgs::Twist cmd_vel;

        //update feedback to correspond to our curent position
        geometry_msgs::PoseStamped global_pose;
        getRobotPose(global_pose, planner_costmap_ros_);
        const geometry_msgs::PoseStamped &current_position = global_pose;

        //push the feedback out
        move_base_msgs::MoveBaseFeedback feedback;
        feedback.base_position = current_position;
        as_->publishFeedback(feedback);

        //check to see if we've moved far enough to reset our oscillation timeout
        if (distance(current_position, oscillation_pose_) >= oscillation_distance_)
        {
            last_oscillation_reset_ = ros::Time::now();
            oscillation_pose_ = current_position;

            //if our last recovery was caused by oscillation, we want to reset the recovery index
            if (recovery_trigger_ == OSCILLATION_R)
                recovery_index_ = 0;
        }

        //check that the observation buffers for the costmap are current, we don't want to drive blind
        if (!controller_costmap_ros_->isCurrent())
        {
            ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety", ros::this_node::getName().c_str());
            publishZeroVelocity();
            return false;
        }

        //if we have a new plan then grab it and give it to the controller
        if (new_global_plan_)
        {
            //make sure to set the new plan flag to false
            new_global_plan_ = false;

            ROS_DEBUG_NAMED("move_base", "Got a new plan...swap pointers");

            //do a pointer swap under mutex
            std::vector<geometry_msgs::PoseStamped> *temp_plan = controller_plan_;

            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            controller_plan_ = latest_plan_;
            latest_plan_ = temp_plan;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base", "pointers swapped!");

            if (!tc_->setPlan(*controller_plan_))
            {
                //ABORT and SHUTDOWN COSTMAPS
                ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                resetState();

                //disable the planner thread
                lock.lock();
                runPlanner_ = false;
                lock.unlock();

                as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
                return true;
            }

            //make sure to reset recovery_index_ since we were able to find a valid plan
            if (recovery_trigger_ == PLANNING_R)
                recovery_index_ = 0;
        }
        else
        {
            ROS_WARN("Oscar//////No new global plan.");
        }

        //the move_base state machine, handles the control logic for navigation
        switch (state_)
        {
        //if we are in a planning state, then we'll attempt to make a plan
        case PLANNING:
        {
            ROS_INFO("Oscar//////////executeCycle,We are in Planning case.");
            boost::recursive_mutex::scoped_lock lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
        }
            ROS_DEBUG_NAMED("move_base", "Waiting for plan, in the planning state.");
            break;

        //if we're controlling, we'll attempt to find valid velocity commands
        case CONTROLLING:
            ROS_INFO("Oscar//////////executeCycle,We are in Controlling case.");
            ROS_DEBUG_NAMED("move_base", "In controlling state.");

            //check to see if we've reached our goal
            if (tc_->isGoalReached())
            {
                ROS_DEBUG_NAMED("move_base", "Goal reached!");
                resetState();

                //disable the planner thread
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = false;
                user_goal_reached_ = true;
                lock.unlock();

                as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
                return true;
            }

            //check for an oscillation condition
            if (oscillation_timeout_ > 0.0 &&
                last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
            {
                publishZeroVelocity();
                state_ = CLEARING;
                recovery_trigger_ = OSCILLATION_R;
            }

            {
                boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

                if (tc_->computeVelocityCommands(cmd_vel))
                {
                    ROS_DEBUG_NAMED("move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

                    ROS_WARN("Oscar::Teb_ the velocity is:%.3lf, %.3lf, %.3lf",
                             cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

                    last_valid_control_ = ros::Time::now();
                    //make sure that we send the velocity command to the base
                    vel_pub_.publish(cmd_vel);
                    if (recovery_trigger_ == CONTROLLING_R)
                        recovery_index_ = 0;
                }
                else
                {
                    ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
                    ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

                    //check if we've tried to find a valid control for longer than our time limit
                    if (ros::Time::now() > attempt_end)
                    {
                        //we'll move into our obstacle clearing mode
                        publishZeroVelocity();
                        state_ = CLEARING;
                        recovery_trigger_ = CONTROLLING_R;
                    }
                    else
                    {
                        //otherwise, if we can't find a valid control, we'll go back to planning
                        last_valid_plan_ = ros::Time::now();
                        planning_retries_ = 0;
                        state_ = PLANNING;
                        publishZeroVelocity();

                        //enable the planner thread in case it isn't running on a clock
                        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                        runPlanner_ = true;
                        planner_cond_.notify_one();
                        lock.unlock();
                    }
                }
            }
            break;

        //we'll try to clear out space with any user-provided recovery behaviors
        case CLEARING:
            ROS_INFO("Oscar///////executeCycle, we are in clearing case");
            ROS_DEBUG_NAMED("move_base", "In clearing/recovery state");
            //we'll invoke whatever recovery behavior we're currently on if they're enabled
            if (recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
            {
                ROS_DEBUG_NAMED("move_base_recovery", "Executing behavior %u of %zu", recovery_index_ + 1, recovery_behaviors_.size());

                move_base_msgs::RecoveryStatus msg;
                msg.pose_stamped = current_position;
                msg.current_recovery_number = recovery_index_;
                msg.total_number_of_recoveries = recovery_behaviors_.size();
                msg.recovery_behavior_name = recovery_behavior_names_[recovery_index_];

                recovery_status_pub_.publish(msg);

                recovery_behaviors_[recovery_index_]->runBehavior();

                //we at least want to give the robot some time to stop oscillating after executing the behavior
                last_oscillation_reset_ = ros::Time::now();

                //we'll check if the recovery behavior actually worked
                ROS_DEBUG_NAMED("move_base_recovery", "Going back to planning state");
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                state_ = PLANNING;

                //update the index of the next recovery behavior that we'll try
                recovery_index_++;
            }
            else
            {
                ROS_DEBUG_NAMED("move_base_recovery", "All recovery behaviors have failed, locking the planner and disabling it.");
                //disable the planner thread
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();

                ROS_DEBUG_NAMED("move_base_recovery", "Something should abort after this.");

                if (recovery_trigger_ == CONTROLLING_R)
                {
                    ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
                }
                else if (recovery_trigger_ == PLANNING_R)
                {
                    ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
                }
                else if (recovery_trigger_ == OSCILLATION_R)
                {
                    ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
                    as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
                }
                resetState();
                return true;
            }
            break;
        default:
            ROS_ERROR("This case should never be reached, something is wrong, aborting");
            resetState();
            //disable the planner thread
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = false;
            lock.unlock();
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
            return true;
        }

        //we aren't done yet
        return false;
    }

    bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node)
    {
        XmlRpc::XmlRpcValue behavior_list;
        if (node.getParam("recovery_behaviors", behavior_list))
        {
            if (behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                for (int i = 0; i < behavior_list.size(); ++i)
                {
                    if (behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                    {
                        if (behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type"))
                        {
                            //check for recovery behaviors with the same name
                            for (int j = i + 1; j < behavior_list.size(); j++)
                            {
                                if (behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                                {
                                    if (behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type"))
                                    {
                                        std::string name_i = behavior_list[i]["name"];
                                        std::string name_j = behavior_list[j]["name"];
                                        if (name_i == name_j)
                                        {
                                            ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                                                      name_i.c_str());
                                            return false;
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
                            return false;
                        }
                    }
                    else
                    {
                        ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                                  behavior_list[i].getType());
                        return false;
                    }
                }

                //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
                for (int i = 0; i < behavior_list.size(); ++i)
                {
                    try
                    {
                        //check if a non fully qualified name has potentially been passed in
                        if (!recovery_loader_.isClassAvailable(behavior_list[i]["type"]))
                        {
                            std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
                            for (unsigned int i = 0; i < classes.size(); ++i)
                            {
                                if (behavior_list[i]["type"] == recovery_loader_.getName(classes[i]))
                                {
                                    //if we've found a match... we'll get the fully qualified name and break out of the loop
                                    ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                             std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                                    behavior_list[i]["type"] = classes[i];
                                    break;
                                }
                            }
                        }

                        boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

                        //shouldn't be possible, but it won't hurt to check
                        if (behavior.get() == NULL)
                        {
                            ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
                            return false;
                        }

                        //initialize the recovery behavior with its name
                        behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
                        recovery_behavior_names_.push_back(behavior_list[i]["name"]);
                        recovery_behaviors_.push_back(behavior);
                    }
                    catch (pluginlib::PluginlibException &ex)
                    {
                        ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
                        return false;
                    }
                }
            }
            else
            {
                ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
                          behavior_list.getType());
                return false;
            }
        }
        else
        {
            //if no recovery_behaviors are specified, we'll just load the defaults
            return false;
        }

        //if we've made it here... we've constructed a recovery behavior list successfully
        return true;
    }

    //we'll load our default recovery behaviors here
    void MoveBase::loadDefaultRecoveryBehaviors()
    {
        recovery_behaviors_.clear();
        try
        {
            //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
            ros::NodeHandle n("~");
            n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
            n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

            //first, we'll load a recovery behavior to clear the costmap
            boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("conservative_reset");
            recovery_behaviors_.push_back(cons_clear);

            //next, we'll load a recovery behavior to rotate in place
            boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
            if (clearing_rotation_allowed_)
            {
                rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
                recovery_behavior_names_.push_back("rotate_recovery");
                recovery_behaviors_.push_back(rotate);
            }

            //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
            boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("aggressive_reset");
            recovery_behaviors_.push_back(ags_clear);

            //we'll rotate in-place one more time
            if (clearing_rotation_allowed_)
            {
                recovery_behaviors_.push_back(rotate);
                recovery_behavior_names_.push_back("rotate_recovery");
            }
        }
        catch (pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
        }

        return;
    }

    void MoveBase::resetState()
    {
        // Disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        // Reset statemachine
        state_ = PLANNING;
        recovery_index_ = 0;
        recovery_trigger_ = PLANNING_R;
        publishZeroVelocity();

        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if (shutdown_costmaps_)
        {
            ROS_DEBUG_NAMED("move_base", "Stopping costmaps");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }
    }

    void MoveBase::resetStateOfADAS()
    {
        state_ = CONTROLLING;
        recovery_index_ = 0;
        recovery_trigger_ = PLANNING_R;
        publishZeroVelocityToSimulator();

        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if (shutdown_costmaps_)
        {
            ROS_DEBUG_NAMED("move_base", "Stopping costmaps");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }
    }

    bool MoveBase::getRobotPose(geometry_msgs::PoseStamped &global_pose, costmap_2d::Costmap2DROS *costmap)
    {
        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        geometry_msgs::PoseStamped robot_pose;
        tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
        robot_pose.header.frame_id = robot_base_frame_;
        robot_pose.header.stamp = ros::Time();     // latest available
        ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

        // get robot pose on the given costmap frame
        try
        {
            tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
        }
        catch (tf2::LookupException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ConnectivityException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ExtrapolationException &ex)
        {
            ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        // check if global_pose time stamp is within costmap transform tolerance
        if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
        {
            ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. "
                                   "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f",
                              costmap->getName().c_str(),
                              current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
            return false;
        }

        return true;
    }

    bool MoveBase::executeCycle(std::vector<geometry_msgs::Pose>& local_path, const std::vector<double> force_human_vec, double time, double long_flag, double lat_flag)
    {
        boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
        //we need to be able to publish velocity commands
        geometry_msgs::Twist cmd_vel;

        //update feedback to correspond to our curent position
        geometry_msgs::PoseStamped global_pose;
        getRobotPose(global_pose, planner_costmap_ros_);
        const geometry_msgs::PoseStamped &current_position = global_pose;

        //check to see if we've moved far enough to reset our oscillation timeout
        if (distance(current_position, oscillation_pose_) >= oscillation_distance_)
        {
            last_oscillation_reset_ = ros::Time::now();
            oscillation_pose_ = current_position;

            //if our last recovery was caused by oscillation, we want to reset the recovery index
            if (recovery_trigger_ == OSCILLATION_R)
                recovery_index_ = 0;
        }

        //check that the observation buffers for the costmap are current, we don't want to drive blind
        if (!controller_costmap_ros_->isCurrent())
        {
            ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety", ros::this_node::getName().c_str());
            publishZeroVelocityToSimulator();
            return false;
        }

        //if we have a new plan then grab it and give it to the controller
        if (new_global_plan_)
        {
            //make sure to set the new plan flag to false
            new_global_plan_ = false;

            ROS_DEBUG_NAMED("move_base", "Got a new plan...swap pointers");

            //do a pointer swap under mutex
            std::vector<geometry_msgs::PoseStamped> *temp_plan = controller_plan_;

            controller_plan_ = latest_plan_;
            latest_plan_ = temp_plan;

            if (!tc_->setPlan(*controller_plan_))
            {
                //ABORT and SHUTDOWN COSTMAPS
                ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                resetStateOfADAS();
                return true;
            }

            //make sure to reset recovery_index_ since we were able to find a valid plan
            if (recovery_trigger_ == PLANNING_R)
                recovery_index_ = 0;
        }

        //the move_base state machine, handles the control logic for navigation
        switch (state_)
        {
        //if we're controlling, we'll attempt to find valid velocity commands
        case CONTROLLING:
            ROS_DEBUG_NAMED("move_base", "In controlling state.");

            //check to see if we've reached our goal
            if (tc_->isGoalReached())
            {
                ROS_DEBUG_NAMED("move_base", "Goal reached!");
                resetStateOfADAS();
                return true;
            }

            //check for an oscillation condition
            if (oscillation_timeout_ > 0.0 &&
                last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
            {
                publishZeroVelocityToSimulator();
                state_ = CLEARING;
                recovery_trigger_ = OSCILLATION_R;
            }

            {
                boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

                if (tc_->computeVelocityCommands(cmd_vel))
                {
					double dt = 0.0;
          			tc_->getLocalPath(local_path, time, dt);

					ROS_WARN("Oscar::The size of local path is:%d, the period of local path is:%f, the last pose is:%f", local_path.size(), dt, local_path.back().position.x);

                    last_valid_control_ = ros::Time::now();
                    tc_->SetGoalNotReached();

                    cmd_buffer_.push_back(cmd_vel);
                    if (cmd_buffer_.size() > 3)
                    {
                        cmd_buffer_.pop_front();
                    }

                    /*
					double sum_lon = 0.0;
                    double sum_rot = 0.0;
                    for (auto it = cmd_buffer_.begin(); it != cmd_buffer_.end(); ++it)
                    {
                        sum_lon += it->linear.x;
                        sum_rot += it->angular.z;
                    }*/

                    /*double ave_lon = sum_lon / cmd_buffer_.size();

                    if (ave_lon <= 0)
                    {
                    cmd_vel.linear.x = sum_lon / cmd_buffer_.size();
                    }
                    else
                    {
                      cmd_vel.linear.x = 0.0;
                    }

                    cmd_vel.angular.z = sum_rot / cmd_buffer_.size();
					*/

                    ROS_WARN("Oscar::move_base:Got a new command from the local planner: %.3lf, %.3lf, %.3lf",
                             cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

					if (long_flag)
					{
						cmd_vel.linear.z = LONG;
						vel_to_sim_pub_.publish(cmd_vel);
					}
					else if (lat_flag)
					{
						std::vector<double> force_auto_vec;
		                for (auto it = local_path.begin(); it != local_path.end(); ++it)
		                {
							double force_rej = 0.0;
							CalculateAPF(*it, force_rej);
							double force_rej_weighted = force_rej * std::pow(decay_rate_, int(it - local_path.begin()));
							ROS_WARN("Oscar::autoforce_rej_weighted:%f, weight:%f", force_rej_weighted, std::pow(decay_rate_, int(it - local_path.begin())));
							force_auto_vec.push_back(force_rej_weighted);
						}

						double apf_human = 0.0;
						double apf_auto  = 0.0;
						if (dt < predict_time_)
						{
							double pose_number = std::ceil(dt) * step_size_;
							for (int i = 0; i < pose_number; ++i)
							{
								apf_human += std::fabs(force_human_vec[i]);
							}
						}
						else
						{
							for (int i = 0; i < force_human_vec.size(); ++i)
							{
								apf_human += std::fabs(force_human_vec[i]);
							}
						}

						for (int i = 0; i < force_auto_vec.size(); ++i)
						{
							apf_auto += std::fabs(force_auto_vec[i]);
						}

						apf_human = std::min(std::fabs(apf_human / force_human_vec.size()), force_max_);
						apf_auto  = std::min(std::fabs(apf_auto / force_auto_vec.size()), force_max_);
						ROS_WARN("Oscar::apf_human:%f, apf_auto:%f", apf_human, apf_auto);
						cmd_vel.linear.z = double(LAT);
						ROS_WARN("Oscar::cmd_vel.linear.z is:%f, and automation is:%f", cmd_vel.linear.z, double(LAT));
						cmd_vel.angular.x = apf_human;
						cmd_vel.angular.y = apf_auto;
						vel_to_sim_pub_.publish(cmd_vel);
					}

                    if (recovery_trigger_ == CONTROLLING_R)
                        recovery_index_ = 0;
                }
                else
                {
                    ROS_WARN("Oscar::move_base, The local planner could not find a valid plan.");
                    ros::Time attempt_end = last_valid_control_ + ros::Duration(planner_patience_);

                    //check if we've tried to find a valid control for longer than our time limit
                    if (ros::Time::now() > attempt_end)
                    {
                        //we'll move into our obstacle clearing mode
                        publishZeroVelocityToSimulator();
                        state_ = CLEARING;
                        recovery_trigger_ = CONTROLLING_R;
                        ROS_WARN("Oscar::Turn to CLEARING mode.");
                    }
                    else
                    {
                        //otherwise, if we can't find a valid control, we'll go back to planning
                        last_valid_plan_ = ros::Time::now();
                        planning_retries_ = 0;
                        publishZeroVelocityToSimulator();
                        ROS_WARN("Oscar::Cannot find a valid control.");
                    }
                }
            }

            break;

        //we'll try to clear out space with any user-provided recovery behaviors
        case CLEARING:
            ROS_DEBUG_NAMED("move_base", "In clearing/recovery state");
            //we'll invoke whatever recovery behavior we're currently on if they're enabled
            if (recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
            {
                ROS_DEBUG_NAMED("move_base_recovery", "Executing behavior %u of %zu", recovery_index_ + 1, recovery_behaviors_.size());

                move_base_msgs::RecoveryStatus msg;
                msg.pose_stamped = current_position;
                msg.current_recovery_number = recovery_index_;
                msg.total_number_of_recoveries = recovery_behaviors_.size();
                msg.recovery_behavior_name = recovery_behavior_names_[recovery_index_];

                recovery_status_pub_.publish(msg);

                recovery_behaviors_[recovery_index_]->runBehavior();

                //we at least want to give the robot some time to stop oscillating after executing the behavior
                last_oscillation_reset_ = ros::Time::now();

                //we'll check if the recovery behavior actually worked
                ROS_DEBUG_NAMED("move_base_recovery", "Going back to planning state");
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;

                //update the index of the next recovery behavior that we'll try
                recovery_index_++;
            }
            else
            {
                ROS_DEBUG_NAMED("move_base_recovery", "All recovery behaviors have failed, locking the planner and disabling it.");

                ROS_DEBUG_NAMED("move_base_recovery", "Something should abort after this.");

                if (recovery_trigger_ == CONTROLLING_R)
                {
                    ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
                }
                else if (recovery_trigger_ == PLANNING_R)
                {
                    ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
                }
                else if (recovery_trigger_ == OSCILLATION_R)
                {
                    ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
                }
                resetStateOfADAS();
                return true;
            }
            break;
        default:
            ROS_ERROR("This case should never be reached, something is wrong, aborting");
            resetStateOfADAS();
            return true;
        }

        //we aren't done yet
        return false;
    }

    geometry_msgs::PoseStamped MoveBase::ComputeNewPosition(const geometry_msgs::PoseStamped &pos, const geometry_msgs::Twist &vel, double dt)
    {
        geometry_msgs::PoseStamped new_pos;
        std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
        new_pos.header.stamp = ros::Time();
        new_pos.header.frame_id = global_frame;

        double pos_x = pos.pose.position.x;
        double pos_y = pos.pose.position.y;
        double pos_theta = tf2::getYaw(pos.pose.orientation);
        double vel_x = vel.linear.x;
        double vel_y = vel.linear.y;
        double vel_theta = vel.angular.z;

        new_pos.pose.position.x = pos_x + (vel_x * cos(pos_theta) + vel_y * cos(M_PI_2 + pos_theta)) * dt;
        new_pos.pose.position.y = pos_y + (vel_x * sin(pos_theta) + vel_y * sin(M_PI_2 + pos_theta)) * dt;
        new_pos.pose.orientation = tf::createQuaternionMsgFromYaw(pos_theta + vel_theta * dt);

        return new_pos;
    }

    void MoveBase::SetBoundary(const unsigned int cell_x, const unsigned int cell_y, unsigned int &x_max, unsigned int &x_min,
                               unsigned int &y_max, unsigned int &y_min)
    {
        if (cell_x > x_max)
        {
            x_max = cell_x;
        }
        else if (cell_x < x_min)
        {
            x_min = cell_x;
        }

        if (cell_y > y_max)
        {
            y_max = cell_y;
        }
        else if (cell_y < y_min)
        {
            y_min = cell_y;
        }
    }

    void MoveBase::sleepPlanner(ros::NodeHandle &n, ros::Timer &timer, ros::Time &start_time_ADAS, boost::unique_lock<boost::recursive_mutex> &lock)
    {
        ros::Duration sleep_time = (start_time_ADAS + ros::Duration(1.0 / planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0))
        {
            timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
            planner_cond_.wait(lock);
        }
        else
        {
            ros::Duration cycle_time = ros::Time::now() - start_time_ADAS;
            ROS_WARN("Oscar::ADAS loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", planner_frequency_, cycle_time.toSec());
        }
    }

    double MoveBase::LinearInterpolation(const double xp)
    {
        double yp = y0_ + ((y1_ - y0_) / (x1_ - x0_)) * (xp - x0_);
        return yp;
    }

    double MoveBase::ComputeCost(const double delta_dist)
    {
        double cost = exp(-1.0 * weight_ * delta_dist) * (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1.0);
        return cost;
    }

    bool MoveBase::UpdateGoalPose(const geometry_msgs::PoseStamped &pose)
    {
        bool update_x = std::fabs(pose.pose.position.x - prev_goal_pose_.pose.position.x) > 0.15; //(0.15 + driver_cmd_.linear.x * target_margin_ / step_size_);
        bool update_y = std::fabs(pose.pose.position.y - prev_goal_pose_.pose.position.y) > 0.1; //(0.1 + driver_cmd_.linear.y * target_margin_ / step_size_);
        if (update_x || update_y)
        {
            return true;
        }
        return false;
    }

    void MoveBase::ClassifyObstacles()
    {
        if (!costmap_converter_)
            return;

        //Get obstacles from costmap converter
        costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
        if (!obstacles)
            return;

        for (std::size_t i = 0; i < obstacles->obstacles.size(); ++i)
        {
            const costmap_converter::ObstacleMsg *obstacle = &obstacles->obstacles.at(i);
            const geometry_msgs::Polygon *polygon = &obstacle->polygon;
            if (polygon->points.size() == 1) // Point
            {
                continue;
                //Obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
            }
            else if (polygon->points.size() == 2) // Line
            {
                Obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                                  polygon->points[1].x, polygon->points[1].y)));
            }
            else if (polygon->points.size() > 2) // Real polygon
            {
                PolygonObstacle *polyobst = new PolygonObstacle();
                for (std::size_t j = 0; j < polygon->points.size(); ++j)
                {
                    polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
                }
                polyobst->finalizePolygon();
                Obstacles_.push_back(ObstaclePtr(polyobst));
            }

            // Set velocity, if obstacle is moving
            if (!Obstacles_.empty())
                Obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
        }
    }

    void MoveBase::Padding(unsigned int &x_max, unsigned int &x_min, unsigned int &y_max, unsigned int &y_min)
    {
        // padding from x and y directions, notice that the obstacles at back are not interested
        x_max = std::min((x_max + padding_size_x_), (costmap_->getSizeInCellsX() - 1));
        x_min = std::max(x_min, (unsigned int)0);
        y_max = std::min((y_max + padding_size_y_), (costmap_->getSizeInCellsY() - 1));
        y_min = std::max((y_min - padding_size_y_), (unsigned int)0);
    }

    bool MoveBase::ObstaclesOfInterest(const unsigned int x_max, const unsigned int x_min, const unsigned int y_max, const unsigned int y_min)
    {
        if (!Obstacles_.size())
        {
            //ROS_WARN("Oscar::No Obstacles at all, either driver is doing good or something wrong with costmap converter.");
            return false;
        }

        for (ObstContainer::const_iterator obs = Obstacles_.begin(); obs != Obstacles_.end(); ++obs)
        {
            const Eigen::Vector2d pose = robot_pose_.position();
            Eigen::Vector2d closest_pt = (*obs)->GetClosestPoint(pose);
            unsigned int cell_x, cell_y;
            costmap_->worldToMap(closest_pt.x(), closest_pt.y(), cell_x, cell_y);
            if ((cell_x >= x_min && cell_x <= x_max) && (cell_y >= y_min && cell_y <= y_max))
            {
                Obstacles_human_path_.push_back(*obs);
            }
        }
        ROS_WARN("Oscar::there are %d obstacles need to be considered.", (int)Obstacles_human_path_.size());
		if (Obstacles_human_path_.size() < 1)
		{
			return false;
		}
        return true;
   }

   void MoveBase::CalculateAPF(const geometry_msgs::Pose pose, double& force_rej)
   {
		double force_rej_x = 0.0;
		double force_rej_y = 0.0;
		for (int i = 0; i < Obstacles_human_path_.size(); ++i)
		{
			ObstaclePtr obs_ptr = Obstacles_human_path_.at(i);
			std::vector<double> x_pts;
			std::vector<double> y_pts;
			if (typeid(*obs_ptr) == typeid(LineObstacle))
			{
			  continue;
			}
			else if (typeid(*obs_ptr) == typeid(PolygonObstacle))
			{
			  Point2dContainer points_vec = boost::dynamic_pointer_cast<hunter_move_base::PolygonObstacle>(obs_ptr)->vertices();
			  for (int j = 0; j < points_vec.size(); ++j)
			  {
				x_pts.push_back(points_vec.at(j).coeffRef(0));
				y_pts.push_back(points_vec.at(j).coeffRef(1));
			  }
			}
			else
			{
			  ROS_WARN("What hell is it?");
			  continue;
			}

			const int len = x_pts.size();
			double x_mean = std::accumulate(x_pts.begin(), x_pts.end(), 0.0)/len;
			double y_mean = std::accumulate(y_pts.begin(), y_pts.end(), 0.0)/len;

			std::vector<double> x_var_vec(len);
			std::vector<double> y_var_vec(len);
			std::transform(x_pts.begin(), x_pts.end(), x_var_vec.begin(), [x_mean](double x){return x - x_mean;});
			std::transform(y_pts.begin(), y_pts.end(), y_var_vec.begin(), [y_mean](double y){return y - y_mean;});

			double xy_cov = std::inner_product(x_var_vec.begin(), x_var_vec.end(), y_var_vec.begin(), 0.0)/(len - 1);
			double x_var  = std::inner_product(x_var_vec.begin(), x_var_vec.end(), x_var_vec.begin(), 0.0)/(len - 1);
			double y_var  = std::inner_product(y_var_vec.begin(), y_var_vec.end(), y_var_vec.begin(), 0.0)/(len - 1);
			double cov_det  = x_var * y_var - std::pow(xy_cov, 2);
			double dist_mh = 0.0;
			double dist_critical = 0.0;

			const PoseSE2 predict_pose = PoseSE2(pose);
			const Eigen::Vector2d robot_position = predict_pose.position();
			const Eigen::Vector2d closest_pt = obs_ptr->GetClosestPoint(robot_position);

			if (x_var < 0.01 && y_var < 0.01)
			{
			  dist_mh = std::sqrt(std::pow(predict_pose.x() - x_mean, 2) + std::pow(predict_pose.y() - y_mean, 2));
			  dist_critical = std::sqrt(std::pow(closest_pt.coeffRef(0) - x_mean, 2) + std::pow(closest_pt.coeffRef(1) - y_mean, 2));
			  ROS_WARN("Oscar::Circle obstacle");
			}
			else if (std::fabs(xy_cov) > 0.01 && std::fabs(cov_det) > 0.01) //make sure determinant of covarience matrix is non-zero.
			{
			  double x_var_inv = y_var / (x_var * y_var - std::pow(xy_cov, 2));
			  double y_var_inv = x_var / (x_var * y_var - std::pow(xy_cov, 2));
			  double xy_cov_inv = - xy_cov / (x_var * y_var - std::pow(xy_cov, 2));

			  ROS_WARN("Oscar::x_var_inv:%f, y_var_inv:%f, xy_cov_inv:%f", x_var_inv, y_var_inv, xy_cov_inv);

			  dist_mh = std::sqrt(std::pow(predict_pose.x() - x_mean, 2) * x_var_inv + 
						2 * (predict_pose.x() - x_mean) * (predict_pose.y() - y_mean) * xy_cov_inv +
						std::pow(predict_pose.y() - y_mean, 2) * y_var_inv);

			  dist_critical = std::sqrt(std::pow(closest_pt.coeffRef(0) - x_mean, 2) * x_var_inv + 
							  2 * (closest_pt.coeffRef(0) - x_mean) * (closest_pt.coeffRef(1) - y_mean) * xy_cov_inv +
							  std::pow(closest_pt.coeffRef(1) - y_mean, 2) * y_var_inv);
			}
			else
			{
			  dist_mh = std::sqrt(std::pow(predict_pose.x() - x_mean, 2)/x_var + std::pow(predict_pose.y() - y_mean, 2)/y_var);
			  dist_critical = std::sqrt(std::pow(closest_pt.coeffRef(0) - x_mean, 2)/x_var + std::pow(closest_pt.coeffRef(1) - y_mean, 2)/y_var);
			  ROS_WARN("Oscar::Standard Ellipse Obstacle.");
			}

			unsigned int cell_x_obs, cell_y_obs;
			if (costmap_ != NULL)
			{
			  if (!costmap_->worldToMap(x_mean, y_mean, cell_x_obs, cell_y_obs))
			  {
				  //we're off the map
				  ROS_WARN("Oscar::Off Map %f, %f", x_mean, y_mean);
			  }
			}

			ROS_WARN("Oscar::The obstacle locates at %f, %f, pose are:%f, %f, x_var:%f, y_var:%f, xy_cov:%f, cov_det:%f, mahal:%f, critical:%f",
					x_mean, y_mean, predict_pose.x(), predict_pose.y(), x_var, y_var, xy_cov, cov_det, dist_mh, dist_critical);
			//ROS_WARN("Oscar::The obstacle cell is %d, %d", cell_x_obs, cell_y_obs);

			//Mahalanobis distance based APF
			/* 
			Using Costmap, not very precise however, we use coordinates instead
			double sin_theta = (static_cast<int>(cell_x_robot) - static_cast<int>(cell_x_obs)) / std::sqrt((std::pow(static_cast<int>(cell_x_robot) - static_cast<int>(cell_x_obs), 2) 													+ std::pow(static_cast<int>(cell_y_robot) - static_cast<int>(cell_y_obs), 2)));
			double cos_theta = (static_cast<int>(cell_y_robot) - static_cast<int>(cell_y_obs)) / std::sqrt((std::pow(static_cast<int>(cell_x_robot) - static_cast<int>(cell_x_obs), 2) 													+ std::pow(static_cast<int>(cell_y_robot) - static_cast<int>(cell_y_obs), 2)));
			*/

			double x_mean_local = cos(predict_pose.theta()) * (x_mean - predict_pose.x()) + sin(predict_pose.theta()) * (y_mean - predict_pose.y());
			double y_mean_local = - sin(predict_pose.theta()) * (x_mean - predict_pose.x()) + cos(predict_pose.theta()) * (y_mean - predict_pose.y());
			double cos_theta = (0.0 - x_mean_local) / (std::sqrt(std::pow(y_mean_local, 2) + std::pow(x_mean_local, 2)));
			double sin_theta = (0.0 - y_mean_local) / (std::sqrt(std::pow(y_mean_local, 2) + std::pow(x_mean_local, 2)));
			//ROS_WARN("Oscar::The angle is:%f", theta);
			//ROS_WARN("Oscar::The sine theta is:%f", sin_theta);
			//ROS_WARN("Oscar::The cosine theta is:%f", cos_theta);

			if (cos_theta > 0)
			{
				ROS_WARN("Oscar::The obstacle is already passed, next one.");
				continue;
			}

			//APF
		    double force_x = 0.0;
		    double force_y = 0.0;
		    if (dist_mh <= dist_critical && sin_theta <= 0)
		    {
		      force_x = -force_max_;
		      force_y = -force_max_;
		    }
			else if(dist_mh <= dist_critical && sin_theta > 0)
			{
		      force_x = force_max_;
		      force_y = force_max_;
			}
		    else
		    {
		      double force = std::min(sigma_ / std::pow(dist_mh, 3), force_max_);
		      if (force <= force_max_)
		      {
		        force_x = force * cos_theta;
		        force_y = force * sin_theta;
		      }
		      else
			  {
		        force_x = force_max_ * cos_theta;
		        force_y = force_max_ * sin_theta;
		      }
		    }

			force_rej_x += force_x;
			force_rej_y += force_y;

			ROS_WARN("Oscar::::::The force in each direction is: %f, %f", force_rej_x, force_rej_y);
   		}

		// make sure force max in case of multiple obstacles
		force_rej = std::min(std::sqrt(std::pow(force_rej_x,2) + std::pow(force_rej_y, 2)), force_max_);
   }
};
