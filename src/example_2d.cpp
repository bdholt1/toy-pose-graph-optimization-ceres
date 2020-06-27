#include <iostream>

#include <ceres/ceres.h>

#include <pose_graph_2d.h>

#include "pose_graph_2d_error_terms.h"

int main()
{
    std::string data_path = std::string( "../data");
    std::string input_filename = data_path + "/input_M3500_g2o.g2o";
    PoseGraph2D graph;
    graph.LoadFromFile(input_filename);
    //graph.AddBogusLoopClosures(25);
    graph.WriteToFile(data_path + "/init_nodes.txt");

    ceres::Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(0.01);

    for( Edge2D* edge : graph.edges_)
    {
      ceres::CostFunction* cost_function = nullptr;
      if (edge->type == EdgeType::Odometry) {
        cost_function = RelativeMotionError::Create(edge->x, edge->y, edge->theta);
      }
      else if (edge->type == EdgeType::LoopClosure) {
        cost_function = DCSLoopClosureError::Create(edge->x, edge->y, edge->theta);
      }
      else if (edge->type == EdgeType::BogusLoopClosure) {
        cost_function = DCSLoopClosureError::Create(edge->x, edge->y, edge->theta);
      }
      problem.AddResidualBlock(cost_function, loss_function, edge->a->p, edge->b->p);
    }


    // The pose graph optimization problem has three DOFs that are not fully
    // constrained. This is typically referred to as gauge freedom. You can apply
    // a rigid body transformation to all the nodes and the optimization problem
    // will still have the exact same cost. The Levenberg-Marquardt algorithm has
    // internal damping which mitigate this issue, but it is better to properly
    // constrain the gauge freedom. This can be done by setting one of the poses
    // as constant so the optimizer cannot change it.
    problem.SetParameterBlockConstant(graph.nodes_[0]->p);

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;

    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;

    // Write Pose Graph after Optimization
    graph.WriteToFile(data_path + "/after_opt_nodes.txt");
}