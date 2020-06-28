#ifndef TOY_POSE_GRAPH_OPTIMIZATION_CERES__POSE_GRAPH_2D_H_
#define TOY_POSE_GRAPH_OPTIMIZATION_CERES__POSE_GRAPH_2D_H_

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

class Vertex2D {
 public:
  Vertex2D(int index, double x, double y, double theta) : index_(index), p_{x, y, theta} {}

  int index_;
  double p_[3];
};

class Edge2D {
 public:
  Edge2D(std::shared_ptr<Vertex2D> a, std::shared_ptr<Vertex2D> b, double x, double y, double theta, EdgeType type,
         Eigen::Matrix3d information)
      : a_(a), b_(b), x_(x), y_(y), theta_(theta), type_(type), information_(information) {}

  std::shared_ptr<Vertex2D> a_, b_;
  double x_, y_, theta_;
  // The inverse of the measurement covariance matrix.
  Eigen::Matrix3d information_;
  EdgeType type_;
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

#endif  // TOY_POSE_GRAPH_OPTIMIZATION_CERES__POSE_GRAPH_2D_H_
