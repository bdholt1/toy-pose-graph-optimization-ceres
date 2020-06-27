#ifndef TOY_POSE_GRAPH_OPTIMIZATION_CERES__POSE_GRAPH_2D_H_
#define TOY_POSE_GRAPH_OPTIMIZATION_CERES__POSE_GRAPH_2D_H_

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
  Node2D(int index, double x, double y, double theta) {
    this->index = index;
    p = new double[3];
    p[0] = x;
    p[1] = y;
    p[2] = theta;
  }

  int index;
  double* p;
};

class Edge2D {
 public:
  Edge2D(const Node2D* a, const Node2D* b, EdgeType type) {
    this->a = a;
    this->b = b;
    this->type = type;
  }

  void setEdgeTransform(double x, double y, double theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
  }

  void setInformationMatrix(double I11, double I12, double I13, double I22, double I23, double I33) {
    this->I11 = I11;
    this->I12 = I12;
    this->I13 = I13;
    this->I22 = I22;
    this->I23 = I23;
    this->I33 = I33;
  }

  const Node2D *a, *b;
  double x, y, theta;
  double I11, I12, I13, I22, I23, I33;
  EdgeType type;
};

class PoseGraph2D {
 public:
  void AddNode(Node2D* node);

  void AddEdge(Edge2D* edge);

  void AddBogusLoopClosures(int n);

  void LoadFromFile(const std::string& filename);

  void WriteToFile(const std::string& filename);

  std::vector<Node2D*> nodes_;  // nodes must be a vector because the indices are used to find nodes
  std::list<Edge2D*> edges_;
};

#endif  // TOY_POSE_GRAPH_OPTIMIZATION_CERES__POSE_GRAPH_2D_H_
