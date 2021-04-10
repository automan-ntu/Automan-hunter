/*
 * Time:10 April 2021
 * Author:huang.wenhui@ntu.edu.sg
 */

#include <ros/console.h>
#include <ros/assert.h>

#include <hunter_move_base/Obstacle_Point.h>

namespace hunter_move_base
{

PointObstacle::PointObstacle() : Obstacle()
{
    pos_ = Eigen::Vector2d::Zero();
}

PointObstacle::PointObstacle(double x, double y) : Obstacle()
{
    pos_ = Eigen::Vector2d(x, y);
}

const Eigen::Vector2d &PointObstacle::GetCentralPoint() const
{
    return pos_;
}

Eigen::Vector2d PointObstacle::getClosestPoint(const Eigen::Vector2d &position) const
{
    return pos_;
}

double PointObstacle::GetMinDistance(const Eigen::Vector2d &position) const
{
    return (position - pos_).norm();
}

double PointObstacle::GetMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) const
{
    return distance_point_to_segment_2d(pos_, line_start, line_end);
}

double PointObstacle::GetMinDistance(const Point2dContainer &polygon) const
{
    return distance_point_to_polygon_2d(pos_, polygon);
}

double PointObstacle::GetPredictedMinDistance(const Eigen::Vector2d &position, double t) const
{
    return (pos_ + t * GetCentroidVelocity() - position).norm();
}

double PointObstacle::GetPredictedMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end, double t) const
{
    return distance_point_to_segment_2d(pos_ + t * GetCentroidVelocity(), line_start, line_end);
}

double PointObstacle::GetPredictedMinDistance(const Point2dContainer &polygon, double t) const
{
    return distance_point_to_polygon_2d(pos_ + t * GetCentroidVelocity(), polygon);
}

}
