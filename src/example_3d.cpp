#include "pose_graph_3d.h"

int main() {
  std::string data_path = std::string("../data");
  std::string input_filename = data_path + "/parking-garage.g2o";
  PoseGraph3D graph;
  graph.LoadFromFile(input_filename);
  // graph.AddBogusLoopClosures(25);
  graph.WriteVerticesToFile(data_path + "/init_nodes.txt");

  graph.Optimise();

  // Write pose graph after optimization
  graph.WriteVerticesToFile(data_path + "/after_opt_nodes.txt");
}
