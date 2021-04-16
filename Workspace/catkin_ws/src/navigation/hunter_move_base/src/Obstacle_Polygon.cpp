/*
 * Time:09 April 2021
 * Author:huang.wenhui@ntu.edu.sg
 */

#include <ros/console.h>
#include <ros/assert.h>

#include <hunter_move_base/Obstacle_Polygon.h>

namespace hunter_move_base
{
  double PolygonObstacle::GetMinDistance(const Eigen::Vector2d &position) const
  {
    return distance_point_to_polygon_2d(position, vertices_);
  }

  double PolygonObstacle::GetMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) const
  {
    return distance_segment_to_polygon_2d(line_start, line_end, vertices_);
  }

  double PolygonObstacle::GetMinDistance(const Point2dContainer &polygon) const
  {
    return distance_polygon_to_polygon_2d(polygon, vertices_);
  }

  double PolygonObstacle::GetPredictedMinDistance(const Eigen::Vector2d &position, double t) const
  {
    Point2dContainer pred_vertices;
    predictVertices(t, pred_vertices);
    return distance_point_to_polygon_2d(position, pred_vertices);
  }

  double PolygonObstacle::GetPredictedMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end, double t) const
  {
    Point2dContainer pred_vertices;
    predictVertices(t, pred_vertices);
    return distance_segment_to_polygon_2d(line_start, line_end, pred_vertices);
  }

  double PolygonObstacle::GetPredictedMinDistance(const Point2dContainer &polygon, double t) const
  {
    Point2dContainer pred_vertices;
    predictVertices(t, pred_vertices);
    return distance_polygon_to_polygon_2d(polygon, pred_vertices);
  }

  Eigen::Vector2d PolygonObstacle::GetClosestPoint(const Eigen::Vector2d &position) const
  {
    // the polygon is a point
    if (noVertices() == 1)
    {
      return vertices_.front();
    }

    if (noVertices() > 1)
    {

      Eigen::Vector2d new_pt = closest_point_on_line_segment_2d(position, vertices_.at(0), vertices_.at(1));

      if (noVertices() > 2) // real polygon and not a line
      {
        double dist = (new_pt - position).norm();
        Eigen::Vector2d closest_pt = new_pt;

        // check each polygon edge
        for (int i = 1; i < noVertices() - 1; ++i) // skip the first one, since we already checked it (new_pt)
        {
          new_pt = closest_point_on_line_segment_2d(position, vertices_.at(i), vertices_.at(i + 1));
          double new_dist = (new_pt - position).norm();
          if (new_dist < dist)
          {
            dist = new_dist;
            closest_pt = new_pt;
          }
        }
        // afterwards we check the edge between goal and start (close polygon)
        new_pt = closest_point_on_line_segment_2d(position, vertices_.back(), vertices_.front());
        double new_dist = (new_pt - position).norm();
        if (new_dist < dist)
          return new_pt;
        else
          return closest_pt;
      }
      else
      {
        return new_pt; // closest point on line segment
      }
    }

    ROS_ERROR("PolygonObstacle::getClosestPoint() cannot find any closest point. Polygon ill-defined?");
    return Eigen::Vector2d::Zero(); // todo: maybe boost::optional?
  }

  void PolygonObstacle::fixPolygonClosure()
  {
    if (vertices_.size() < 2)
      return;

    if (vertices_.front().isApprox(vertices_.back()))
      vertices_.pop_back();
  }

  void PolygonObstacle::calcCentroid()
  {
    if (vertices_.empty())
    {
      centroid_.setConstant(NAN);
      ROS_WARN("PolygonObstacle::calcCentroid(): number of vertices is empty. the resulting centroid is a vector of NANs.");
      return;
    }

    // if polygon is a point
    if (noVertices() == 1)
    {
      centroid_ = vertices_.front();
      return;
    }

    // if polygon is a line:
    if (noVertices() == 2)
    {
      centroid_ = 0.5 * (vertices_.front() + vertices_.back());
      return;
    }

    // otherwise:

    centroid_.setZero();

    // calculate centroid (see wikipedia http://de.wikipedia.org/wiki/Geometrischer_Schwerpunkt#Polygon)
    double A = 0; // A = 0.5 * sum_0_n-1 (x_i * y_{i+1} - x_{i+1} * y_i)
    for (int i = 0; i < noVertices() - 1; ++i)
    {
      A += vertices_.at(i).coeffRef(0) * vertices_.at(i + 1).coeffRef(1) - vertices_.at(i + 1).coeffRef(0) * vertices_.at(i).coeffRef(1);
    }
    A += vertices_.at(noVertices() - 1).coeffRef(0) * vertices_.at(0).coeffRef(1) - vertices_.at(0).coeffRef(0) * vertices_.at(noVertices() - 1).coeffRef(1);
    A *= 0.5;

    if (A != 0)
    {
      for (int i = 0; i < noVertices() - 1; ++i)
      {
        double aux = (vertices_.at(i).coeffRef(0) * vertices_.at(i + 1).coeffRef(1) - vertices_.at(i + 1).coeffRef(0) * vertices_.at(i).coeffRef(1));
        centroid_ += (vertices_.at(i) + vertices_.at(i + 1)) * aux;
      }
      double aux = (vertices_.at(noVertices() - 1).coeffRef(0) * vertices_.at(0).coeffRef(1) - vertices_.at(0).coeffRef(0) * vertices_.at(noVertices() - 1).coeffRef(1));
      centroid_ += (vertices_.at(noVertices() - 1) + vertices_.at(0)) * aux;
      centroid_ /= (6 * A);
    }
    else // A == 0 -> all points are placed on a 'perfect' line
    {
      // seach for the two outer points of the line with the maximum distance inbetween
      int i_cand = 0;
      int j_cand = 0;
      double max_dist = 0;
      for (int i = 0; i < noVertices(); ++i)
      {
        for (int j = i + 1; j < noVertices(); ++j) // start with j=i+1
        {
          double dist = (vertices_[j] - vertices_[i]).norm();
          if (dist > max_dist)
          {
            max_dist = dist;
            i_cand = i;
            j_cand = j;
          }
        }
      }
      // calc centroid of that line
      centroid_ = 0.5 * (vertices_[i_cand] + vertices_[j_cand]);
    }
  }

}
