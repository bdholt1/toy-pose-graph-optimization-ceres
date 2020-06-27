
#include "pose_graph_2d.h"

#include <random>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

void PoseGraph2D::AddNode(Node2D* node) { nodes_.push_back(node); }

void PoseGraph2D::AddEdge(Edge2D* edge) { edges_.push_back(edge); }

void PoseGraph2D::LoadFromFile(const std::string& filename) {
  // Read the file in g2o format
  std::ifstream fp;
  fp.open(filename.c_str());

  std::string line;
  while (std::getline(fp, line)) {
    std::vector<std::string> words;
    boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
    if (words[0].compare("VERTEX_SE2") == 0) {
      int node_index = boost::lexical_cast<int>(words[1]);
      double x = boost::lexical_cast<double>(words[2]);
      double y = boost::lexical_cast<double>(words[3]);
      double theta = boost::lexical_cast<double>(words[4]);

      Node2D* node = new Node2D(node_index, x, y, theta);
      nodes_.push_back(node);
    }

    if (words[0].compare("EDGE_SE2") == 0) {
      // cout << e << words[0] << endl;
      int a_indx = boost::lexical_cast<int>(words[1]);
      int b_indx = boost::lexical_cast<int>(words[2]);

      double dx = boost::lexical_cast<double>(words[3]);
      double dy = boost::lexical_cast<double>(words[4]);
      double dtheta = boost::lexical_cast<double>(words[5]);

      double I11, I12, I13, I22, I23, I33;
      I11 = boost::lexical_cast<double>(words[6]);
      I12 = boost::lexical_cast<double>(words[7]);
      I13 = boost::lexical_cast<double>(words[8]);
      I22 = boost::lexical_cast<double>(words[9]);
      I23 = boost::lexical_cast<double>(words[10]);
      I33 = boost::lexical_cast<double>(words[11]);

      // odometry nodes by definition follow from each other
      // loop closures are any nodes that do not follow on
      bool indices_follow = (abs(a_indx - b_indx) == 1);

      Edge2D* edge =
          new Edge2D(nodes_[a_indx], nodes_[b_indx], indices_follow ? EdgeType::Odometry : EdgeType::LoopClosure);
      edge->setEdgeTransform(dx, dy, dtheta);
      edge->setInformationMatrix(I11, I12, I13, I22, I23, I33);
      edges_.push_back(edge);
    }
  }
}

// write nodes to file to be visualized with python script
void PoseGraph2D::WriteToFile(const std::string& filename) {
  std::ofstream fp;
  fp.open(filename.c_str());
  for (Node2D* n : nodes_) {
    fp << n->index << " " << n->p[0] << " " << n->p[1] << " " << n->p[2] << std::endl;
  }
}

void PoseGraph2D::AddBogusLoopClosures(int n) {

  std::default_random_engine generator;
  std::uniform_int_distribution<int> node_distribution(0, nodes_.size());
  std::uniform_real_distribution<double> translation_distribution(0.5, 10.0);
  std::uniform_real_distribution<double> orientation_distribution(-M_PI, M_PI);

  for (int i = 0; i < n; i++) {
    int a = node_distribution(generator);
    int b = node_distribution(generator);
    // std::cout << a << "<--->" << b << std::endl;
    Edge2D* edge = new Edge2D(nodes_[a], nodes_[b], EdgeType::BogusLoopClosure);
    edge->setEdgeTransform(translation_distribution(generator),
        translation_distribution(generator), orientation_distribution(generator));
    edges_.push_back(edge);
  }
}
