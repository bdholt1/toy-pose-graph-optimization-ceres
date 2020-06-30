#ifndef _POSE_GRAPH_2D_H_
#define _POSE_GRAPH_2D_H_

#include <Eigen/Eigen>

#include <memory>
#include <string>

#include <list>
#include <vector>

enum class EdgeType { Odometry, LoopClosure, BogusLoopClosure, LandmarkObservation };

/*
enum class VertexType {
  Robot,
  Landmark
};
 */

struct Vertex2D {
  Vertex2D(int index, Eigen::Vector2d p, double theta) : index_(index), p_(p), theta_(theta) {}

  int index_;
  Eigen::Vector2d p_;
  double theta_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Edge2D {
  Edge2D(std::shared_ptr<Vertex2D> a, std::shared_ptr<Vertex2D> b, Eigen::Vector2d p, double theta, EdgeType type,
         Eigen::Matrix3d information)
      : a_(a), b_(b), p_(p), theta_(theta), type_(type), information_(information) {}

  std::shared_ptr<Vertex2D> a_, b_;
  Eigen::Vector2d p_;
  double theta_;
  // The inverse of the measurement covariance matrix.
  Eigen::Matrix3d information_;
  EdgeType type_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class PoseGraph2D {
 public:
  void AddVertex(std::shared_ptr<Vertex2D> vertex);

  void AddEdge(Edge2D edge);

  void AddBogusLoopClosures(int n);

  void Optimise();

  void LoadFromFile(const std::string& filename);

  void WriteVerticesToFile(const std::string& filename);

 private:
  // Vertices are stored with a shared_ptr because multiple edges may refer to the same vertices
  std::vector<std::shared_ptr<Vertex2D>> vertices_;
  std::list<Edge2D> edges_;
};

#endif  // _POSE_GRAPH_2D_H_
