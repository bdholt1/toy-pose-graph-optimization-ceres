
#include "pose_graph_2d.h"
#include "pose_graph_2d_error_terms.h"

#include <ceres/ceres.h>

#include <fstream>
#include <random>

void PoseGraph2D::AddVertex(std::shared_ptr<Vertex2D> vertex) { vertices_.push_back(vertex); }

void PoseGraph2D::AddEdge(Edge2D edge) { edges_.push_back(edge); }

void PoseGraph2D::Optimise() {
  ceres::Problem problem;
  ceres::LossFunction* loss_function = nullptr;

  for (auto& edge : edges_) {
    ceres::CostFunction* cost_function = nullptr;
    const Eigen::Matrix3d sqrt_information = edge.information_.llt().matrixL();
    if (edge.type_ == EdgeType::Odometry) {
      cost_function = RelativeMotionError::Create(edge.p_, edge.theta_, sqrt_information);
    } else if (edge.type_ == EdgeType::LoopClosure) {
      cost_function = DCSLoopClosureError::Create(edge.p_, edge.theta_, sqrt_information);
    } else if (edge.type_ == EdgeType::BogusLoopClosure) {
      cost_function = DCSLoopClosureError::Create(edge.p_, edge.theta_, sqrt_information);
    }
    problem.AddResidualBlock(cost_function, loss_function, edge.a_->p_.data(), &edge.a_->theta_, edge.b_->p_.data(),
                             &edge.b_->theta_);
  }

  // The pose graph optimization problem has three DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigate this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  problem.SetParameterBlockConstant(vertices_[0]->p_.data());
  problem.SetParameterBlockConstant(&vertices_[0]->theta_);

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;

  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
}

void PoseGraph2D::LoadFromFile(const std::string& filename) {
  // Read the file in g2o format
  std::ifstream g2o_file(filename.c_str());

  std::string type;
  while (g2o_file >> type) {
    if (type == "VERTEX_SE2") {
      int index;
      Eigen::Vector2d p;
      double theta;
      g2o_file >> index >> p.x() >> p.y() >> theta;
      vertices_.push_back(std::make_shared<Vertex2D>(index, p, theta));
    } else if (type == "EDGE_SE2") {
      int a_index, b_index;
      Eigen::Vector2d p;
      double theta;
      Eigen::Matrix3d information;
      g2o_file >> a_index >> b_index >> p.x() >> p.y() >> theta;

      for (int i = 0; i < 3 && g2o_file.good(); ++i) {
        for (int j = i; j < 3 && g2o_file.good(); ++j) {
          g2o_file >> information(i, j);
          if (i != j) {
            information(j, i) = information(i, j);
          }
        }
      }

      // odometry nodes by definition follow from each other
      // loop closures are any vertices that do not follow on
      bool indices_follow = (abs(a_index - b_index) == 1);
      EdgeType type = indices_follow ? EdgeType::Odometry : EdgeType::LoopClosure;
      Edge2D edge(vertices_[a_index], vertices_[b_index], p, theta, type, information);
      edges_.push_back(edge);
    }
  }
}

// write nodes to file to be visualized with python script
void PoseGraph2D::WriteVerticesToFile(const std::string& filename) {
  std::ofstream fp;
  fp.open(filename.c_str());
  for (auto& v : vertices_) {
    fp << v->index_ << " " << v->p_[0] << " " << v->p_[1] << " " << v->theta_ << std::endl;
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
    Eigen::Vector2d p;
    p.x() = translation_distribution(generator);
    p.y() = translation_distribution(generator);
    double theta = orientation_distribution(generator);
    Eigen::Matrix3d information;

    for (int i = 0; i < 3; ++i) {
      for (int j = i; j < 3; ++j) {
        information(i, j) = information_distribution(generator);
        if (i != j) {
          information(j, i) = information(i, j);
        }
      }
    }

    Edge2D edge(vertices_[a], vertices_[b], p, theta, EdgeType::BogusLoopClosure, information);
    edges_.push_back(edge);
  }
}
