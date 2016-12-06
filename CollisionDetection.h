#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H
#include </usr/local/Cellar/eigen/3.3.0/include/eigen3/Eigen/Dense>
#include <vector>

class CollisionDetection{
 public:
  virtual bool inCollision(const std::vector<Eigen::Vector3d> & point1s,
			   const std::vector<Eigen::Vector3d> & point2s,
			   const std::vector<double> & radii,
			   std::vector<int> & indices) const = 0;

  //point1s.size() # of cylinders

  //cylinder0 -> point1s.at(0) -------- point2s.at(0) with radius radii.at(0);

  //Eigen::Vector3d pt = point1s.at(0);

  //double ptX = pt(0);
  //double ptY = pt(1);
  //double ptZ = pt(2);

  virtual std::vector<Eigen::Vector3d> getInCollisionPoints(const std::vector<Eigen::Vector3d> & point1s,
							    const std::vector<Eigen::Vector3d> & point2s,
							    const std::vector<double> & radii) const {
    return std::vector<Eigen::Vector3d>();
  }
};


#endif
