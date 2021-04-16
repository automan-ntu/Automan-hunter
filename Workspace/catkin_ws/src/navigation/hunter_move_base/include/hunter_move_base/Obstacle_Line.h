/*
 * Time:10 April 2021
 * Author:huang.wenhui@ntu.edu.sg
 */

#ifndef OBSTACLE_LINE_H_
#define OBSTACLE_LINE_H_

#include <hunter_move_base/Obstacle.h>
#include <hunter_move_base/distance_calculations.h>

namespace hunter_move_base
{

    class LineObstacle : public Obstacle
    {
    public:
        /**
     * @brief Default constructor of the polygon obstacle class
     */
        LineObstacle();

        LineObstacle(double x1, double y1, double x2, double y2);

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
        virtual Eigen::Vector2d GetClosestPoint(const Eigen::Vector2d &position) const;

        const Eigen::Vector2d &start() const { return start_; }
        void setStart(const Eigen::Ref<const Eigen::Vector2d> &start)
        {
            start_ = start;
            calcCentroid();
        }
        const Eigen::Vector2d &end() const { return end_; }
        void setEnd(const Eigen::Ref<const Eigen::Vector2d> &end)
        {
            end_ = end;
            calcCentroid();
        }

    protected:
        void calcCentroid();

        Eigen::Vector2d start_;
        Eigen::Vector2d end_;
        Eigen::Vector2d centroid_;
    };
}
#endif
