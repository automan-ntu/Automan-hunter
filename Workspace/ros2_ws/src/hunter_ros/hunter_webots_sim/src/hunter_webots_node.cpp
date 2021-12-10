/*
 * hunter_webots_node.cpp
 *
 * Created on: Jun 02, 2020 12:53
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <iostream>

#include "hunter_webots_sim/hunter_webots_interface.hpp"

using namespace westonrobot;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

static int controllerCount;
static std::vector<std::string> controllerList;

void quit(int sig) {
  ROS_INFO("User stopped the 'hunter_webots_node'.");
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  exit(0);
}

// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
  ROS_INFO("Controller #%d: %s.", controllerCount,
           controllerList.back().c_str());
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "hunter_webots_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh, private_node("~");

  HunterROSMessenger messenger(&nh);
  private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                  std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_,
                                  std::string("base_link"));
  private_node.param<int>("sim_control_rate", messenger.sim_control_rate_, 50);
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_, true);
  messenger.SetupSubscription();

  const uint32_t time_step = 1000 / messenger.sim_control_rate_;
  HunterWebotsInterface hunter_webots(&nh, &messenger, time_step);
  ROS_INFO("Chosen time step: '%d', make sure you set the same time step in Webots scene", time_step);

  signal(SIGINT, quit);

  // subscribe to the topic model_name to get the list of availables controllers
  std::string controllerName;
  ros::Subscriber nameSub =
      nh.subscribe("model_name", 100, controllerNameCallback);
  while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();

  // if there is more than one controller available, it let the user choose
  if (controllerCount == 1)
    controllerName = controllerList[0];
  else {
    int wantedController = 0;
    std::cout << "Choose the # of the controller you want to use:\n";
    std::cin >> wantedController;
    if (1 <= wantedController && wantedController <= controllerCount)
      controllerName = controllerList[wantedController - 1];
    else {
      ROS_ERROR("Invalid number for controller choice.");
      return 1;
    }
  }
  ROS_INFO("Using controller: '%s'", controllerName.c_str());

  // leave topic once it is not necessary anymore
  nameSub.shutdown();

  // init robot components
  hunter_webots.InitComponents(controllerName);

  ROS_INFO("Entering ROS main loop...");

  // main loop
  timeStepClient = nh.serviceClient<webots_ros::set_int>("/" + controllerName +
                                                         "/robot/time_step");
  timeStepSrv.request.value = time_step;
  ros::Rate loop_rate(messenger.sim_control_rate_); 
//   uint32_t cnt = 0;
  while (ros::ok()) {
    if (timeStepClient.call(timeStepSrv) && timeStepSrv.response.success) {
      hunter_webots.UpdateSimState();
      ros::spinOnce();
    } else {
      ROS_ERROR("Failed to call service time_step for next step.");
    //   break;
    }
    // if(++cnt == 166) break;
    loop_rate.sleep();
  }
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);

  ros::shutdown();
  return 0;
}