/*
 * Time:09 April 2021
 * Author:huang.wenhui@ntu.edu.sg
 */

#ifndef OBSTACLE_POLYGON_H_
#define OBSTACLE_POLYGON_H_

#include <geometry_msgs/Polygon.h>
#include <hunter_move_base/Obstacle.h>
#include <hunter_move_base/distance_calculations.h>

namespace hunter_move_base
{

class PolygonObstacle : public Obstacle
{
  /**
   * @brief Default constructor of the polygon obstacle class
   */
  PolygonObstacle() : Obstacle(), finalized_(false)
  {
    centroid_.setConstant(NAN);
  }

  virtual const Eigen::Vector2d& GetCentralPoint() const
  {
    assert(finalized_ && "Finalize the polygon after all vertices are added.");
    return centroid_;
  }
  
  // get minimum distance from the obstacle w.r.t. various shapes
  virtual double GetMinDistance(const Eigen::Vector2d& position) const;
  
  virtual double GetMinDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const;

  virtual double GetMinDistance(const Point2dContainer& polygon) const;

  // get predicted minimum distance from the obstacle after time period w.r.t. various shapes
  virtual double GetPredictedMinDistance(const Eigen::Vector2d& position, double t) const;

  virtual double GetPredictedMinDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const;

  virtual double GetPredictedMinDistance(const Point2dContainer& polygon, double t) const;

  // get closest point from the object
  virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const;

  /**
   * @brief Add a vertex to the polygon obstacle
   * @param x x-coordinate of the new vertex
   * @param y y-coordinate of the new vertex
   */  
  void pushBackVertex(double x, double y)
  {
    vertices_.push_back(Eigen::Vector2d(x,y));
    finalized_ = false;
  }
  
  /**
   * @brief Finalize the polygon obstacle
   */
  void finalizePolygon()
  {
    fixPolygonClosure();
    calcCentroid();
    finalized_ = true;
  }

  void predictVertices(double t, Point2dContainer &pred_vertices) const
  {
    // Predict obstacle (polygon) at time t
    pred_vertices.resize(vertices_.size());
    Eigen::Vector2d offset = t * GetCentroidVelocity();
    for (std::size_t i = 0; i < vertices_.size(); i++)
    {
      pred_vertices[i] = vertices_[i] + offset;
    }
  }

  /**
   * @brief Clear all vertices (Afterwards the polygon is not valid anymore)
   */
  void clearVertices() {vertices_.clear(); finalized_ = false;}
  
  /**
   * @brief Get the number of vertices defining the polygon (the first vertex is counted once)
   */
  int noVertices() const {return (int)vertices_.size();}

protected:
  
  void fixPolygonClosure();
  void calcCentroid();

  
  Point2dContainer vertices_;
  Eigen::Vector2d centroid_;
  
  bool finalized_;
};
}
#endif
