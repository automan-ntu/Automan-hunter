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

#include <ros/publisher.h>
#include <hunter_move_base/visualization.h>
#include <hunter_move_base/Obstacle.h>
#include <hunter_move_base/Obstacle_Point.h>
#include <hunter_move_base/Obstacle_Line.h>
#include <hunter_move_base/Obstacle_Polygon.h>

namespace hunter_move_base
{

  HunterVisualization::HunterVisualization() : initialized_(false)
  {
  }

  HunterVisualization::HunterVisualization(ros::NodeHandle &nh) : initialized_(false)
  {
    initialize(nh);
  }

  void HunterVisualization::initialize(ros::NodeHandle &nh)
  {
    if (initialized_)
      ROS_WARN("HunterVisualization already initialized. Reinitalizing...");

    // register topics
    hunter_marker_pub_ = nh.advertise<visualization_msgs::Marker>("hunter_markers", 1000);
    initialized_ = true;
  }

  void HunterVisualization::publishObstacles(const ObstContainer &obstacles, const unsigned int Index) const
  {
    if (obstacles.empty() || printErrorWhenNotInitialized())
      return;

    // Visualize line obstacles
    {
      std::size_t idx = 0 + int(Index) * 99;
      for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
      {
        boost::shared_ptr<LineObstacle> pobst = boost::dynamic_pointer_cast<LineObstacle>(*obst);
        if (!pobst)
          continue;

        visualization_msgs::Marker marker;
        marker.header.frame_id = std::string("map");
        marker.header.stamp = ros::Time::now();
        marker.ns = "LineObstacles";
        marker.id = idx++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.5);
        geometry_msgs::Point start;
        start.x = pobst->start().x();
        start.y = pobst->start().y();
        start.z = 0;
        marker.points.push_back(start);
        geometry_msgs::Point end;
        end.x = pobst->end().x();
        end.y = pobst->end().y();
        end.z = 0;
        marker.points.push_back(end);

		switch (Index)
        {
		    case INDEX1:
				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
		        break;
		    
			case INDEX2:
				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				break;

			case INDEX3:
				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 0.0;
				marker.color.g = 0.0;
				marker.color.b = 1.0;
				break;

		    default:
				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
		        break;
        }

        hunter_marker_pub_.publish(marker);
      }
    }

    // Visualize polygon obstacles
    {
      std::size_t idx = 0 + int(Index) * 99;
      for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
      {
        boost::shared_ptr<PolygonObstacle> pobst = boost::dynamic_pointer_cast<PolygonObstacle>(*obst);
        if (!pobst)
          continue;

        visualization_msgs::Marker marker;
        marker.header.frame_id = std::string("map");
        marker.header.stamp = ros::Time::now();
        marker.ns = "PolyObstacles";
        marker.id = idx++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.5);

        for (Point2dContainer::const_iterator vertex = pobst->vertices().begin(); vertex != pobst->vertices().end(); ++vertex)
        {
          geometry_msgs::Point point;
          point.x = vertex->x();
          point.y = vertex->y();
          point.z = 0;
          marker.points.push_back(point);
        }

        // Also add last point to close the polygon
        // but only if polygon has more than 2 points (it is not a line)
        if (pobst->vertices().size() > 2)
        {
          geometry_msgs::Point point;
          point.x = pobst->vertices().front().x();
          point.y = pobst->vertices().front().y();
          point.z = 0;
          marker.points.push_back(point);
        }

		switch (Index)
        {
		    case INDEX1:
				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
		        break;
		    
			case INDEX2:
				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				break;

			case INDEX3:
				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 0.0;
				marker.color.g = 0.0;
				marker.color.b = 1.0;
				break;

		    default:
				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.color.a = 1.0;
				marker.color.r = 1.0;
				marker.color.g = 0.0;
				marker.color.b = 0.0;
		        break;
        }
        marker.color.b = 0.0;

        hunter_marker_pub_.publish(marker);
      }
    }
  }

  std_msgs::ColorRGBA HunterVisualization::toColorMsg(double a, double r, double g, double b)
  {
    std_msgs::ColorRGBA color;
    color.a = a;
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
  }

  bool HunterVisualization::printErrorWhenNotInitialized() const
  {
    if (!initialized_)
    {
      ROS_ERROR("HunterVisualization class not initialized. You must call initialize or an appropriate constructor");
      return true;
    }
    return false;
  }

} // namespace hunter_move_base
