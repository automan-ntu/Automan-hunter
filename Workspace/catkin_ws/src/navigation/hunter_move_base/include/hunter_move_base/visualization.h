/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

// ros stuff
#include <ros/ros.h>
#include <ros/publisher.h>

// boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

// std
#include <iterator>

// messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

// move_base
#include <hunter_move_base/Obstacle.h>

namespace hunter_move_base
{

  enum VisualizationIndex
  {
	INDEX1 = 0,
	INDEX2 = 1,
	INDEX3 = 2
  };

  /**
 * @class HunterVisualization
 * @brief Visualize stuff from the Hunter_move_base
 */
  class HunterVisualization
  {
  public:
    /**
   * @brief Default constructor
   * @remarks do not forget to call initialize()
   */
    HunterVisualization();

    /**
   * @brief Constructor that initializes the class and registers topics
   * @param nh local ros::NodeHandle
   */
    HunterVisualization(ros::NodeHandle &nh);

    /**
   * @brief Initializes the class and registers topics.
   * 
   * Call this function if only the default constructor has been called before.
   * @param nh local ros::NodeHandle
   */
    void initialize(ros::NodeHandle &nh);

    /** @name Publish to topics */
    //@{

    /**
   * @brief Publish obstacle positions to the ros topic \e ../../Hunter_markers
   * @todo Move filling of the marker message to polygon class in order to avoid checking types.
   * @param obstacles Obstacle container
   */
    void publishObstacles(const ObstContainer &obstacles, const unsigned int Index) const;

    /**
   * @brief Helper function to generate a color message from single values
   * @param a Alpha value
   * @param r Red value
   * @param g Green value
   * @param b Blue value
   * @return Color message
   */
    static std_msgs::ColorRGBA toColorMsg(double a, double r, double g, double b);

  protected:
    /**
   * @brief Small helper function that checks if initialize() has been called and prints an error message if not.
   * @return \c true if not initialized, \c false if everything is ok
   */
    bool printErrorWhenNotInitialized() const;

    ros::Publisher hunter_marker_pub_;  //!< Publisher for visualization markers

    bool initialized_; //!< Keeps track about the correct initialization of this class

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  //! Abbrev. for shared instances of the HunterVisualization
  typedef boost::shared_ptr<HunterVisualization> HunterVisualizationPtr;

  //! Abbrev. for shared instances of the HunterVisualization (read-only)
  typedef boost::shared_ptr<const HunterVisualization> HunterVisualizationConstPtr;

} // namespace hunter_move_base

#endif /* VISUALIZATION_H_ */
