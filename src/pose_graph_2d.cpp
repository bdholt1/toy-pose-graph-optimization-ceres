
#include "pose_graph_2d.h"

#include <fstream>
#include <random>

void PoseGraph2D::AddNode(std::shared_ptr<Vertex2D> vertex) { vertices_.push_back(vertex); }

void PoseGraph2D::AddEdge(Edge2D edge) { edges_.push_back(edge); }

void PoseGraph2D::LoadFromFile(const std::string& filename) {
  // Read the file in g2o format
  std::ifstream g2o_file(filename.c_str());

  std::string type;
  while (g2o_file >> type) {
    if (type == "VERTEX_SE2") {
      int index;
      double x, y, theta;
      g2o_file >> index >> x >> y >> theta;
      vertices_.push_back(std::make_unique<Vertex2D>(index, x, y, theta));
    } else if (type == "EDGE_SE2") {
      int a_index, b_index;
      double x, y, theta;
      double I11, I12, I13, I22, I23, I33;
      g2o_file >> a_index >> b_index >> x >> y >> theta >> I11 >> I12 >> I13 >> I22 >> I23 >> I33;

      // odometry nodes by definition follow from each other
      // loop closures are any vertices that do not follow on
      bool indices_follow = (abs(a_index - b_index) == 1);

      Edge2D edge(vertices_[a_index], vertices_[b_index], indices_follow ? EdgeType::Odometry : EdgeType::LoopClosure);
      edge.setEdgeTransform(x, y, theta);
      edge.setInformationMatrix(I11, I12, I13, I22, I23, I33);
      edges_.push_back(edge);
    }
  }
}

// write nodes to file to be visualized with python script
void PoseGraph2D::WriteVerticesToFile(const std::string& filename) {
  std::ofstream fp;
  fp.open(filename.c_str());
  for (auto& v : vertices_) {
    fp << v->index_ << " " << v->p_[0] << " " << v->p_[1] << " " << v->p_[2] << std::endl;
  }
}

void PoseGraph2D::AddBogusLoopClosures(int n) {
  std::default_random_engine generator;
  std::uniform_int_distribution<int> vertex_distribution(0, vertices_.size());
  std::uniform_real_distribution<double> translation_distribution(-2.0, 2.0);
  std::uniform_real_distribution<double> orientation_distribution(-M_PI, M_PI);
  std::uniform_real_distribution<double> information_distribution(0.000001, 1);

  for (int i = 0; i < n; i++) {
    int a = vertex_distribution(generator);
    int b = vertex_distribution(generator);
    Edge2D edge(vertices_[a], vertices_[b], EdgeType::BogusLoopClosure);
    edge.setEdgeTransform(translation_distribution(generator), translation_distribution(generator),
                          orientation_distribution(generator));
    edge.setInformationMatrix(information_distribution(generator), information_distribution(generator),
                              information_distribution(generator), information_distribution(generator),
                              information_distribution(generator), information_distribution(generator));
    edges_.push_back(edge);
  }
}
