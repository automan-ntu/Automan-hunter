/*
 * Time:10 April 2021
 * Author:huang.wenhui@ntu.edu.sg
 */

#ifndef OBSTACLE_POINT_H_
#define OBSTACLE_POINT_H_

#include <hunter_move_base/Obstacle.h>
#include <hunter_move_base/distance_calculations.h>

namespace hunter_move_base
{

class PointObstacle : public Obstacle
{
    /**
     * @brief Default constructor of the polygon obstacle class
     */
    PointObstacle();

    PointObstacle(double x, double y);

    virtual const Eigen::Vector2d &GetCentralPoint() const;

    // get minimum distance from the obstacle w.r.t. various shapes
    virtual double GetMinDistance(const Eigen::Vector2d &position) const;

    virtual double GetMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) const;

    virtual double GetMinDistance(const Point2dContainer &polygon) const;

    // get predicted minimum distance from the obstacle after time period w.r.t. various shapes
    virtual double GetPredictedMinDistance(const Eigen::Vector2d &position, double t) const;

    virtual double GetPredictedMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end, double t) const;

    virtual double GetPredictedMinDistance(const Point2dContainer &polygon, double t) const;

    // get closest point from the object
    virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d &position) const;

protected:
    Eigen::Vector2d pos_;
};
}
#endif
