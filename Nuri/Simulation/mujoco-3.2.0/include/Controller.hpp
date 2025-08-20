#pragma once

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <eigen3/Eigen/Dense>

#include <mujoco/mujoco.h>
#include <iostream>

class Controller
{
    public:
        Controller(pinocchio::Model pinocchio_model);
        Eigen::VectorXd computeGravityCompensation(const mjModel* m, mjData* d);
        Eigen::VectorXd computeDotFixedPD(const mjModel* m, mjData* d, Eigen::Vector3d& fixed_pt);
        Eigen::VectorXd computeLineFixedPD(const mjModel* m, mjData* d, Eigen::Vector3d& fixed_pt, Eigen::Vector3d& fixed_vec);
    private:
        pinocchio::Model model_;
        pinocchio::Data data_;

        Eigen::MatrixXd Kp_, Kd_, Kp_task_, Kd_task_;
};