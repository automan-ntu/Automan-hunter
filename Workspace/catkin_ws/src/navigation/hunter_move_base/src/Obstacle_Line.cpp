/*
 * Time:10 April 2021
 * Author:huang.wenhui@ntu.edu.sg
 */

#include <ros/console.h>
#include <ros/assert.h>

#include <hunter_move_base/Obstacle_Line.h>

namespace hunter_move_base
{
LineObstacle::LineObstacle() : Obstacle()
{
    start_.setZero();
    end_.setZero();
    centroid_.setZero();
}

LineObstacle::LineObstacle(double x1, double y1, double x2, double y2) : Obstacle()
{
    start_.x() = x1;
    start_.y() = y1;
    end_.x() = x2;
    end_.y() = y2;
    calcCentroid();
}

const Eigen::Vector2d &LineObstacle::GetCentralPoint() const
{
    return centroid_;
}

Eigen::Vector2d LineObstacle::getClosestPoint(const Eigen::Vector2d &position) const
{
    return closest_point_on_line_segment_2d(position, start_, end_);
}

void LineObstacle::calcCentroid()
{
    centroid_ = 0.5 * (start_ + end_);
}

double LineObstacle::GetMinDistance(const Eigen::Vector2d &position) const
{
    return distance_point_to_segment_2d(position, start_, end_);
}

double LineObstacle::GetMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) const
{
    return distance_segment_to_segment_2d(start_, end_, line_start, line_end);
}

double LineObstacle::GetMinDistance(const Point2dContainer &polygon) const
{
    return distance_segment_to_polygon_2d(start_, end_, polygon);
}

double LineObstacle::GetPredictedMinDistance(const Eigen::Vector2d &position, double t) const
{
    Eigen::Vector2d offset = t * GetCentroidVelocity();
    return distance_point_to_segment_2d(position, start_ + offset, end_ + offset);
}

double LineObstacle::GetPredictedMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end, double t) const
{
    Eigen::Vector2d offset = t * GetCentroidVelocity();
    return distance_segment_to_segment_2d(start_ + offset, end_ + offset, line_start, line_end);
}

double LineObstacle::GetPredictedMinDistance(const Point2dContainer &polygon, double t) const
{
    Eigen::Vector2d offset = t * GetCentroidVelocity();
    return distance_segment_to_polygon_2d(start_ + offset, end_ + offset, polygon);
}
}
