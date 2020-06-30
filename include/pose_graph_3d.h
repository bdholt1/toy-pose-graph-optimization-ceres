#ifndef _POSE_GRAPH_3D_H_
#define _POSE_GRAPH_3D_H_

#include <Eigen/Eigen>

#include <memory>
#include <string>

#include <list>
#include <vector>

enum class EdgeType { Odometry, LoopClosure, BogusLoopClosure, LandmarkObservation };

struct Vertex3D {
  Vertex3D(int index, Eigen::Vector3d p, Eigen::Quaterniond q) : index_(index), p_(p), q_(q) {};

  int index_;
  Eigen::Vector3d p_;
  Eigen::Quaterniond q_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Edge3D {
  Edge3D(std::shared_ptr<Vertex3D> a, std::shared_ptr<Vertex3D> b, Eigen::Vector3d p, Eigen::Quaterniond q, EdgeType type, Eigen::Matrix<double, 6, 6> information) : a_(a), b_(b), p_(p), q_(q), type_(type), information_(information) {};

  std::shared_ptr<Vertex3D> a_, b_;
  Eigen::Vector3d p_;
  Eigen::Quaterniond q_;
  // The inverse of the measurement covariance matrix.
  Eigen::Matrix<double, 6, 6> information_;
  EdgeType type_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class PoseGraph3D {
 public:
  void AddVertex(std::shared_ptr<Vertex3D> vertex);

  void AddEdge(Edge3D edge);

  void AddBogusLoopClosures(int n);

  void Optimise();

  void LoadFromFile(const std::string& filename);

  void WriteVerticesToFile(const std::string& filename);

 private:
  // Vertices are stored with a shared_ptr because multiple edges may refer to the same vertices
  std::vector<std::shared_ptr<Vertex3D>> vertices_;
  std::list<Edge3D> edges_;
};

#endif //_POSE_GRAPH_3D_H_
