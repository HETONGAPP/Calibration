/*
 * @Author: Tong HE
 * @Date: 2021-12-08 18:15:31
 * @LastEditTime: 2021-12-19 18:40:09
 * @LastEditors: Tong HE
 * @Description: 
 * @FilePath: /undefined/home/tong/catkin_robot/src/tool_point_calibration/src/tool_point_calibration.cpp
 */
#include <tool_point_calibration/tool_point_calibration.h>
#include <ros/ros.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include<vector>
/**
 * @brief The ToolPointEstimator struct
 */
struct ToolPointEstimator
{
  ToolPointEstimator(const Eigen::Affine3d& robot_pose)
  {
      ceres::RotationMatrixToAngleAxis(robot_pose.rotation().data(), angle_axis_);
      translation_[0] = robot_pose.translation()(0);
      translation_[1] = robot_pose.translation()(1);
      translation_[2] = robot_pose.translation()(2);
  }

  template <typename T>
  bool operator()(const T* const tool, const T* const point, T* residuals) const
  {
    // Convert "angle_axis_" to type T
    T angle_axis[3];
    angle_axis[0] = T(angle_axis_[0]);
    angle_axis[1] = T(angle_axis_[1]);
    angle_axis[2] = T(angle_axis_[2]);

    // Rotate the estimated 'tool' offset and put the result into 'rotated_pt'
    T rotated_pt[3];
    ceres::AngleAxisRotatePoint(angle_axis, tool, rotated_pt);

    // Convert "translation_" to type T and calculate the tool point in the world frame
    T pos[3];
    pos[0] = rotated_pt[0] + T(translation_[0]);
    pos[1] = rotated_pt[1] + T(translation_[1]);
    pos[2] = rotated_pt[2] + T(translation_[2]);

    // Compute the error between the current estimate of the tool points
    residuals[0] = point[0] - pos[0];
    residuals[1] = point[1] - pos[1];
    residuals[2] = point[2] - pos[2];
    return true;
  }

  double angle_axis_[3];
  double translation_[3];
};


struct AngleEstimate
{
  AngleEstimate(const tool_point_calibration::Affine3dVector& robot_pose,const Eigen::Vector3d &tcp_guess)
  {
      // translation[0] = robot_pose[0].translation()(0);
      // translation[1] = robot_pose[0].translation()(1);
      // translation[2] = robot_pose[0].translation()(2);
      M1 << robot_pose[0].rotation()(0),robot_pose[0].rotation()(1),robot_pose[0].rotation()(2),
            robot_pose[0].rotation()(3),robot_pose[0].rotation()(4),robot_pose[0].rotation()(5),
            robot_pose[0].rotation()(6),robot_pose[0].rotation()(7),robot_pose[0].rotation()(8);
      

      M2 << robot_pose[1].rotation()(0),robot_pose[1].rotation()(1),robot_pose[1].rotation()(2),
            robot_pose[1].rotation()(3),robot_pose[1].rotation()(4),robot_pose[1].rotation()(5),
            robot_pose[1].rotation()(6),robot_pose[1].rotation()(7),robot_pose[1].rotation()(8);

      M3 << robot_pose[2].rotation()(0),robot_pose[2].rotation()(1),robot_pose[2].rotation()(2),
            robot_pose[2].rotation()(3),robot_pose[2].rotation()(4),robot_pose[2].rotation()(5),
            robot_pose[2].rotation()(6),robot_pose[2].rotation()(7),robot_pose[2].rotation()(8);


      
      M1 = M1*180/3.14;
      M2 = M2*180/3.14;
      M3 = M3*180/3.14;
      
      translation1 
         << robot_pose[0].translation()(0),robot_pose[0].translation()(1),robot_pose[0].translation()(2);
      translation2
         << robot_pose[1].translation()(0),robot_pose[1].translation()(1),robot_pose[1].translation()(2);
      translation3
         << robot_pose[2].translation()(0),robot_pose[2].translation()(1),robot_pose[2].translation()(2);


      
      P_otcp_To_B = (M1*tcp_guess+translation1).transpose();
      P_xtcp_To_B = (M2*tcp_guess+translation2).transpose();
      P_ztcp_To_B = (M3*tcp_guess+translation3).transpose();

      
      vector_X = P_xtcp_To_B - P_otcp_To_B;

      dire_vec_x_o = (M1.inverse()*vector_X)/vector_X.norm();

      vector_Z = P_ztcp_To_B - P_otcp_To_B;
      dire_vec_z_o = (M1.inverse()*vector_Z)/vector_Z.norm();


      dire_vec_y_o = dire_vec_z_o.transpose().cross(dire_vec_x_o.transpose());

      dire_vec_z_o = dire_vec_x_o.transpose().cross(dire_vec_y_o.transpose());
      
      ROS_INFO_STREAM("Calibrated Vector X: [" << dire_vec_x_o.transpose() << "]");
      ROS_INFO_STREAM("Calibrated Vector Y: [" << dire_vec_y_o.transpose()<< "]");
      ROS_INFO_STREAM("Calibrated Vector Z: [" << dire_vec_z_o.transpose() << "]");
  };
  
  double angle_axis_[3];
  Eigen::Vector3d translation1;
  Eigen::Vector3d translation2;
  Eigen::Vector3d translation3;
  Eigen::Matrix <double , 3 , 3> M1;
  Eigen::Matrix <double , 3 , 3> M2;
  Eigen::Matrix <double , 3 , 3> M3;

  Eigen::Vector3d P_otcp_To_B;
  Eigen::Vector3d P_xtcp_To_B;
  Eigen::Vector3d vector_X;
  Eigen::Vector3d dire_vec_x_o;

  Eigen::Vector3d P_ztcp_To_B;
  Eigen::Vector3d vector_Z;
  Eigen::Vector3d dire_vec_z_o;

  Eigen::Vector3d dire_vec_y_o;
};



tool_point_calibration::TcpCalibrationResult
tool_point_calibration::calibrateTcp(const tool_point_calibration::Affine3dVector &tool_poses,
                                     const tool_point_calibration::Affine3dVector &tool_poses1,
                                     const Eigen::Vector3d &tcp_guess,
                                     const Eigen::Vector3d &touch_pt_guess)
{
    Eigen::Vector3d internal_tcp_guess = tcp_guess;
    Eigen::Vector3d internal_touch_guess = touch_pt_guess;

    ceres::Problem problem;
    for (std::size_t i = 0; i < tool_poses.size(); ++i)
    {
        auto* cost_fn =
            new ceres::AutoDiffCostFunction<ToolPointEstimator, 3, 3, 3>(
              new ToolPointEstimator( tool_poses[i] )
            );

        problem.AddResidualBlock(cost_fn, NULL, internal_tcp_guess.data(),
                                 internal_touch_guess.data());
    }

    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);


      
    new AngleEstimate(tool_poses1,internal_tcp_guess);
    



    TcpCalibrationResult result;
    result.tcp_offset = internal_tcp_guess;
    result.touch_point = internal_touch_guess;
    result.average_residual = summary.final_cost / summary.num_parameter_blocks;
    result.converged = summary.termination_type == ceres::CONVERGENCE;

    return result;
}
