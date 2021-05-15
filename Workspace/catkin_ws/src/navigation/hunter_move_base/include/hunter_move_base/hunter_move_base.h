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
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_converter/ObstacleMsg.h>
#include <costmap_converter/costmap_converter_interface.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <hunter_move_base/Obstacle.h>
#include <hunter_move_base/Obstacle_Point.h>
#include <hunter_move_base/Obstacle_Line.h>
#include <hunter_move_base/Obstacle_Polygon.h>
#include <hunter_move_base/visualization.h>
#include "hunter_move_base/MoveBaseConfig.h"
#include "hunter_move_base/pose_se2.h"
#include <sensor_msgs/Joy.h>

namespace hunter_move_base
{
    //typedefs to help us out with the action server so that we don't hace to type so much
    typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

    enum MoveBaseState
    {
        PLANNING,
        CONTROLLING,
        CLEARING
    };

    enum RecoveryTrigger
    {
        PLANNING_R,
        CONTROLLING_R,
        OSCILLATION_R
    };

	enum ObstacleIndex
	{
		ALL 	= 0,
		HUMAN 	= 1,
		AUTO 	= 2
	};

    /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
    class MoveBase
    {
    public:
        /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
        MoveBase(tf2_ros::Buffer &tf);

        /**
       * @brief  Destructor - Cleans up
       */
        virtual ~MoveBase();

        /**
       * @brief  Performs a control cycle
       * @param goal A reference to the goal to pursue
       * @return True if processing of the goal is done, false otherwise
       */
        bool executeCycle(geometry_msgs::PoseStamped &goal);

        /**
       * @brief  Performs a plan cycle of ADAS
       * @return True if processing of the goal is done, false otherwise
       */
        bool executeCycle();

        /**
       * @brief  Computes a new position
       * @param  pos  Start position
       * @param  vel  Current velocity
       * @param  dt   Time interval 
       * @return New Pose
       */
        geometry_msgs::PoseStamped ComputeNewPosition(const geometry_msgs::PoseStamped &pos, const geometry_msgs::Twist &vel, double dt);

        /**
       * @brief  Set boundary of interest of obstacle region
       * @param  cell_x  Current cell x
       * @param  cell_y  Current cell y
       * @param  x_max   Max boundary x
       * @param  x_min   Min boundary x
       * @param  y_max   Max boundary y
       * @param  y_min   Min boundary y
       */
        void SetBoundary(const unsigned int cell_x, const unsigned int cell_y, unsigned int &x_max, unsigned int &x_min,
                         unsigned int &y_max, unsigned int &y_min);

    private:
        /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
        bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

        /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
        bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

        /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
        bool makePlan(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan);

        /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
        bool loadRecoveryBehaviors(ros::NodeHandle node);

        /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
        void loadDefaultRecoveryBehaviors();

        /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
        void clearCostmapWindows(double size_x, double size_y);

        /**
       * @brief  Publishes a velocity command of zero to the base
       */
        void publishZeroVelocity();
        void publishZeroVelocityToSimulator();
        void publishDefaultVelocityToSimulator();

        /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
        void resetState();

        void resetStateOfADAS();

        void goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal);

        //void triggerCB(const sensor_msgs::Joy::ConstPtr& joy_ptr);

        void triggerCB(const geometry_msgs::Twist::ConstPtr &twist_ptr);

        void planThread();

        void executeCb(const move_base_msgs::MoveBaseGoalConstPtr &move_base_goal);

        bool isQuaternionValid(const geometry_msgs::Quaternion &q);

        bool getRobotPose(geometry_msgs::PoseStamped &global_pose, costmap_2d::Costmap2DROS *costmap);

        double distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);

        geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg);

        /**
       * @brief This is used to wake the planner at periodic intervals.
       */
        void wakePlanner(const ros::TimerEvent &event);

        /**
       * @brief This is used to sleep the planner thread
       */
        void sleepPlanner(ros::NodeHandle &n, ros::Timer &timer, ros::Time &start_time_ADAS, boost::unique_lock<boost::recursive_mutex> &lock);

        /**
       * @brief  Linear Interpolation
       * @param  Target point
       * @return Interpolated Value.
       */
        double LinearInterpolation(const double xp);

        /**
       * @brief  Cost computation
       * @param  relative distance
       * @return Cost
       */
        double ComputeCost(const double delta_dist);

        /**
       * @brief Update Goal Pose
       * @param Target Pose
       * @return True if need to update, false otherwise.
       */
        bool UpdateGoalPose(const geometry_msgs::PoseStamped &pose);

        /**
       * @brief Set Global goal pose
       * @param Goal Point from actionlib
       */
        void SetGoalPoint(const move_base_msgs::MoveBaseGoalConstPtr &move_base_goal);

        /**
	   * @brief Classify the obstacles within certain shapes
	   */
        void ClassifyObstacles();

        void Padding(unsigned int &x_max, unsigned int &x_min, unsigned int &y_max, unsigned int &y_min);

        bool ObstaclesOfInterest(const unsigned int x_max, const unsigned int x_min, const unsigned int y_max, const unsigned int y_min);

        tf2_ros::Buffer &tf_;

        MoveBaseActionServer *as_;

        boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
        costmap_2d::Costmap2DROS *planner_costmap_ros_, *controller_costmap_ros_;

        boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
        std::string robot_base_frame_, global_frame_;

        std::vector<boost::shared_ptr<nav_core::RecoveryBehavior>> recovery_behaviors_;
        std::vector<std::string> recovery_behavior_names_;
        unsigned int recovery_index_;

        geometry_msgs::PoseStamped global_pose_;
        double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
        double planner_patience_, controller_patience_;
        int32_t max_planning_retries_;
        uint32_t planning_retries_;
        double conservative_reset_dist_, clearing_radius_;
        ros::Publisher current_goal_pub_, global_goal_pub_, vel_pub_, vel_to_sim_pub_, action_goal_pub_, recovery_status_pub_;
        ros::Subscriber goal_sub_, cmd_vel_driver_sub_;
        ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
        bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
        bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
        double oscillation_timeout_, oscillation_distance_;

        MoveBaseState state_;
        RecoveryTrigger recovery_trigger_;

        ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
        geometry_msgs::PoseStamped oscillation_pose_;
        pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
        pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
        pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

        //set up plan triple buffer
        std::vector<geometry_msgs::PoseStamped> *planner_plan_;
        std::vector<geometry_msgs::PoseStamped> *latest_plan_;
        std::vector<geometry_msgs::PoseStamped> *controller_plan_;

        //set up the planner's thread
        bool runPlanner_;
        boost::recursive_mutex planner_mutex_;
        boost::condition_variable_any planner_cond_;
        geometry_msgs::PoseStamped global_goal_;
        geometry_msgs::PoseStamped planner_goal_;
        boost::thread *planner_thread_;

        //user interface flag
        bool user_goal_reached_;

        boost::recursive_mutex configuration_mutex_;
        dynamic_reconfigure::Server<hunter_move_base::MoveBaseConfig> *dsrv_;

        void reconfigureCB(hunter_move_base::MoveBaseConfig &config, uint32_t level);

        hunter_move_base::MoveBaseConfig last_config_;
        hunter_move_base::MoveBaseConfig default_config_;
        bool setup_, p_freq_change_, c_freq_change_;
        bool new_global_plan_;

        // elements employed by ADAS
        geometry_msgs::Twist robot_velo_, driver_cmd_;
        geometry_msgs::PoseStamped prev_goal_pose_;
        PoseSE2 robot_pose_;
        base_local_planner::OdometryHelperRos odom_helper_;
        double predict_time_;
        bool adas_trigger_;
        bool prev_plan_flag_;
        int target_margin_;
        int step_size_;
        std::list<geometry_msgs::Twist> cmd_buffer_;
        std::shared_ptr<base_local_planner::CostmapModel> costmap_model_;

        //Interpolation upper and lower bound
        double x0_, x1_, y0_, y1_;

        //parameters of potential function
        double weight_;

        //costmap converter
        pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_;
        boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;
        std::string costmap_converter_plugin_ = "costmap_converter::CostmapToPolygonsDBSMCCH";
		std::string name_ = "MoveBase";
        std::string odom_topic_ = "odom";
        int costmap_converter_rate_ = 5;
        bool costmap_converter_spin_thread_ = true;

        //Classify Obstacles
        costmap_2d::Costmap2D *costmap_;
        ObstContainer Obstacles_;
        ObstContainer Obstacles_human_path_;
        ObstContainer Obstacles_auto_path_;
        HunterVisualizationPtr vis_;
        unsigned int padding_size_x_ = 10;
        unsigned int padding_size_y_ = 10;
    };
};
#endif

