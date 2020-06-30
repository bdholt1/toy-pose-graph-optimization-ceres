#include "pose_graph_3d.h"
#include "pose_graph_3d_error_terms.h"

#include <ceres/ceres.h>

#include <fstream>

void PoseGraph3D::AddVertex(std::shared_ptr<Vertex3D> vertex) { vertices_.push_back(vertex); }

void PoseGraph3D::AddEdge(Edge3D edge) { edges_.push_back(edge); }

void PoseGraph3D::Optimise() {
  ceres::Problem problem;
  ceres::LossFunction* loss_function = nullptr;
  ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

  for (auto& edge : edges_) {
    ceres::CostFunction* cost_function = nullptr;
    const Eigen::Matrix<double, 6, 6> sqrt_information = edge.information_.llt().matrixL();
    if (edge.type_ == EdgeType::Odometry) {
      cost_function = RelativeMotionError::Create(edge.p_, edge.q_, sqrt_information);
    } else if (edge.type_ == EdgeType::LoopClosure) {
      cost_function = RelativeMotionError::Create(edge.p_, edge.q_, sqrt_information);
    }
    problem.AddResidualBlock(cost_function, loss_function, edge.a_->p_.data(), edge.a_->q_.coeffs().data(),
                             edge.b_->p_.data(), edge.b_->q_.coeffs().data());

    problem.SetParameterization(edge.a_->q_.coeffs().data(), quaternion_local_parameterization);
    problem.SetParameterization(edge.b_->q_.coeffs().data(), quaternion_local_parameterization);
  }

  // The pose graph optimization problem has six DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigates this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  problem.SetParameterBlockConstant(vertices_[0]->p_.data());
  problem.SetParameterBlockConstant(vertices_[0]->q_.coeffs().data());

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;

  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
}

void PoseGraph3D::LoadFromFile(const std::string& filename) {
  // Read the file in g2o format
  std::ifstream g2o_file(filename.c_str());

  std::string type;
  while (g2o_file >> type) {
    if (type == "VERTEX_SE3:QUAT") {
      int index;
      Eigen::Vector3d p;
      Eigen::Quaterniond q;
      g2o_file >> index >> p.x() >> p.y() >> p.z() >> q.x() >> q.y() >> q.z() >> q.w();
      q.normalize();
      vertices_.push_back(std::make_shared<Vertex3D>(index, p, q));
    } else if (type == "EDGE_SE3:QUAT") {
      int a_index, b_index;
      Eigen::Vector3d p;
      Eigen::Quaterniond q;
      Eigen::Matrix<double, 6, 6> information;
      g2o_file >> a_index >> b_index >> p.x() >> p.y() >> p.z() >> q.x() >> q.y() >> q.z() >> q.w();
      q.normalize();

      for (int i = 0; i < 6 && g2o_file.good(); ++i) {
        for (int j = i; j < 6 && g2o_file.good(); ++j) {
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
      Edge3D edge(vertices_[a_index], vertices_[b_index], p, q, type, information);
      edges_.push_back(edge);
    }
  }
}

// write nodes to file to be visualized with python script
void PoseGraph3D::WriteVerticesToFile(const std::string& filename) {
  std::ofstream fp;
  fp.open(filename.c_str());
  for (auto& v : vertices_) {
    fp << v->index_ << " " << v->p_.x() << " " << v->p_.y() << " " << v->p_.z() << std::endl;
  }
}
