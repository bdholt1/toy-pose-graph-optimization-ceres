#include <iostream>

#include <ceres/ceres.h>

#include "pose_graph_2d.h"
#include "pose_graph_2d_error_terms.h"

int main() {
  std::string data_path = std::string("../data");
  std::string input_filename = data_path + "/M3500.g2o";
  PoseGraph2D graph;
  graph.LoadFromFile(input_filename);
  //graph.AddBogusLoopClosures(25);
  graph.WriteNodesToFile(data_path + "/init_nodes.txt");

  ceres::Problem problem;
  ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

  for (auto& edge : graph.edges_) {
    ceres::CostFunction* cost_function = nullptr;
    const Eigen::Matrix3d sqrt_information = edge.information_.llt().matrixL();
    if (edge.type_ == EdgeType::Odometry) {
      cost_function = RelativeMotionError::Create(edge.x_, edge.y_, edge.theta_, sqrt_information);
    } else if (edge.type_ == EdgeType::LoopClosure) {
      cost_function = DCSLoopClosureError::Create(edge.x_, edge.y_, edge.theta_, sqrt_information);
    } else if (edge.type_ == EdgeType::BogusLoopClosure) {
      cost_function = DCSLoopClosureError::Create(edge.x_, edge.y_, edge.theta_, sqrt_information);
    }
    problem.AddResidualBlock(cost_function, loss_function, edge.a_->p_, edge.b_->p_);
  }

  // The pose graph optimization problem has three DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigate this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  problem.SetParameterBlockConstant(graph.nodes_[0]->p_);

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;

  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  // Write pose graph after optimization
  graph.WriteNodesToFile(data_path + "/after_opt_nodes.txt");
}
