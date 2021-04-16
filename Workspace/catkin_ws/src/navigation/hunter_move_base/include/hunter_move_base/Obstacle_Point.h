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
    public:
        /**
     * @brief Default constructor of the polygon obstacle class
     */
        PointObstacle();

        PointObstacle(double x, double y);

        const Eigen::Vector2d &GetCentralPoint() const;

        // get minimum distance from the obstacle w.r.t. various shapes
        double GetMinDistance(const Eigen::Vector2d &position) const;

        double GetMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) const;

        double GetMinDistance(const Point2dContainer &polygon) const;

        // get predicted minimum distance from the obstacle after time period w.r.t. various shapes
        double GetPredictedMinDistance(const Eigen::Vector2d &position, double t) const;

        double GetPredictedMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end, double t) const;

        double GetPredictedMinDistance(const Point2dContainer &polygon, double t) const;

        // get closest point from the object
        Eigen::Vector2d GetClosestPoint(const Eigen::Vector2d &position) const;

        // implements predictCentroidConstantVelocity() of the base class
        void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const
        {
            position = pos_ + t * GetCentroidVelocity();
        }

        const Eigen::Vector2d &position() const { return pos_; } //!< Return the current position of the obstacle (read-only)
        Eigen::Vector2d &position() { return pos_; }             //!< Return the current position of the obstacle
        double &x() { return pos_.coeffRef(0); }                 //!< Return the current x-coordinate of the obstacle
        const double &x() const { return pos_.coeffRef(0); }     //!< Return the current y-coordinate of the obstacle (read-only)
        double &y() { return pos_.coeffRef(1); }                 //!< Return the current x-coordinate of the obstacle
        const double &y() const { return pos_.coeffRef(1); }     //!< Return the current y-coordinate of the obstacle (read-only)

    protected:
        Eigen::Vector2d pos_;
    };
}
#endif
