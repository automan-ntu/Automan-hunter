/*
 * Time:08 April 2021
 * Author:huang.wenhui@ntu.edu.sg
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <hunter_move_base/distance_calculations.h>

namespace hunter_move_base
{
    /**
 * @class class for obstacles
 * @brief Info about obstacles
 */
    class Obstacle
    {
    public:
        /**
     * @brief Default constructor of the obstacle class
     */
        Obstacle() : dynamic_(false), centroid_velocity_(Eigen::Vector2d::Zero()) {}

        /**
     * @brief Deconstructor of the obstacle class
     */
        ~Obstacle() {}

        /**
     * @brief Get the central point of the obstacle
     * @return Coordinate of central point
     */
        virtual const Eigen::Vector2d &GetCentralPoint() const = 0;

        /**
     * @brief  Get minimum distance from the obstacle
     * @param  position Position of the object
     * @return The distance
     */
        virtual double GetMinDistance(const Eigen::Vector2d &position) const = 0;

        /**
     * @brief Get the minimum euclidean distance to the obstacle (line as reference)
     * @param line_start 2d position of the begin of the reference line
     * @param line_end 2d position of the end of the reference line
     * @return The nearest possible distance to the obstacle
     */
        virtual double GetMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end) const = 0;

        /**
     * @brief Get minimum distance from the obstacle
     * @param polygon points of polygon obstacle
     * @return The distance
     */
        virtual double GetMinDistance(const Point2dContainer &polygon) const = 0;

        /**
     * @brief Get the closest point on the obstacle w.r.t. the object
     * @param position position of the object
     * @return The closest point
     */
        virtual Eigen::Vector2d GetClosestPoint(const Eigen::Vector2d &position) const = 0;

        /**
    * @brief Get the predicted distance to the moving obstacle using a constant velocity and orientation
    * @param position position of the object
    * @param t time horizon of prediction
    * @return The predicted distance to the obstacle after time t
    */
        virtual double GetPredictedMinDistance(const Eigen::Vector2d &position, double t) const = 0;

        /**
    * @brief Get the estimated minimum spatiotemporal distance to the moving obstacle using a constant velocity model (line as reference)
    * @param line_start 2d position of the begin of the reference line
    * @param line_end 2d position of the end of the reference line
    * @param t time, for which the minimum distance to the obstacle is estimated
    * @return The nearest possible distance to the obstacle at time t
    */
        virtual double GetPredictedMinDistance(const Eigen::Vector2d &line_start, const Eigen::Vector2d &line_end, double t) const = 0;

        /**
     * @brief Get the predicted minimum distance to moving obstacle using a constant velocity and orientation
     * @param polygon points of polygon obstacle
     * @param t time horizon of prediciton
     * @return The predicted minimum distance to the obstacle after time t
     */
        virtual double GetPredictedMinDistance(const Point2dContainer &polygon, double t) const = 0;

        /**
     * @brief  sGet centroid velocity
     * @return Velocity
     */
        Eigen::Vector2d GetCentroidVelocity() const { return centroid_velocity_; }

        void setCentroidVelocity(const geometry_msgs::TwistWithCovariance &velocity,
                                 const geometry_msgs::QuaternionStamped &orientation)
        {
            setCentroidVelocity(velocity, orientation.quaternion);
        }

        void setCentroidVelocity(const geometry_msgs::TwistWithCovariance &velocity,
                                 const geometry_msgs::Quaternion &orientation)
        {
            Eigen::Vector2d vel;
            vel.coeffRef(0) = velocity.twist.linear.x;
            vel.coeffRef(1) = velocity.twist.linear.y;

            // to avoid the sensor error
            if (vel.norm() < 0.001)
                return;

            setCentroidVelocity(vel);
        }

        void setCentroidVelocity(const Eigen::Ref<const Eigen::Vector2d> &vel)
        {
            centroid_velocity_ = vel;
            dynamic_ = true;
        }

    private:
        bool dynamic_;
        Eigen::Vector2d centroid_velocity_;
    };

    typedef boost::shared_ptr<Obstacle> ObstaclePtr;
    typedef std::vector<ObstaclePtr> ObstContainer;

}

#endif /* obstacle */
