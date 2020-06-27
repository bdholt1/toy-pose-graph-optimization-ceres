#ifndef TOY_POSE_GRAPH_OPTIMIZATION_CERES__POSE_GRAPH_2D_H_
#define TOY_POSE_GRAPH_OPTIMIZATION_CERES__POSE_GRAPH_2D_H_

#include <Eigen/Eigen>

#include <fstream>
#include <string>

#include <list>
#include <vector>

enum class EdgeType { Odometry, LoopClosure, BogusLoopClosure, LandmarkObservation };

/*
enum class NodeType {
  Robot,
  Landmark
};
 */

class Node2D {
 public:
  Node2D(int index, double x, double y, double theta) : index_(index), p_{x, y, theta} {}

  int index_;
  double p_[3];
};

class Edge2D {
 public:
  Edge2D(Node2D* a, Node2D* b, EdgeType type) : a_(a), b_(b), type_(type) {}

  void setEdgeTransform(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
  }

  void setInformationMatrix(double I11, double I12, double I13, double I22, double I23, double I33) {
    information_(0, 0) = I11;
    information_(0, 1) = I12;
    information_(0, 2) = I13;
    information_(1, 1) = I22;
    information_(1, 2) = I23;
    information_(2, 2) = I33;

    // Set the lower triangular part of the information matrix.
    information_(1, 0) = information_(0, 1);
    information_(2, 0) = information_(0, 2);
    information_(2, 1) = information_(1, 2);
  }

  Node2D *a_, *b_;
  double x_, y_, theta_;
  // The inverse of the measurement covariance matrix.
  Eigen::Matrix3d information_;
  EdgeType type_;
};

class PoseGraph2D {
 public:
  void AddNode(Node2D* node);

  void AddEdge(Edge2D edge);

  void AddBogusLoopClosures(int n);

  void LoadFromFile(const std::string& filename);

  void WriteToFile(const std::string& filename);

  // TODO: these members are public because direct access is required to build
  // the ceres problem
  std::vector<Node2D*> nodes_;  // nodes must be a vector because indices identify nodes
  std::list<Edge2D> edges_;
};

#endif  // TOY_POSE_GRAPH_OPTIMIZATION_CERES__POSE_GRAPH_2D_H_
