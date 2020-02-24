#include "/backend/vertex_pose.h"
#include "/backend/vertex_speedbias.h"
#include "backend/edge_imu.h"

namespace vio {
	namespace backend {
		using Sophus::SO3d;
		Vec3 EdgeImu::gravity_ = Vec3(0, 0, 9.8);//东北地坐标系,static变量

		void EdgeImu::ComputeResidual() {
			VecX param_pose_i = verticies_[0]->Paramerters();
			Qd Qi(param_pose_i[6], param_pose_i[3], param_pose_i[4], param_pose_i[5]);
			Vec3 Pi = Param_pose_i.head<3>();
			
			VecX param_v_bias_i = verticies_[1]->Paramerters();
			Vec3 Vi = param_v_bias_i.head<3>();
			Vec3 Bai = param_v_bias_i.segment(3, 3);
			Vec3 Bgi = param_v_bias_i.tail<3>();

			VecX param_pose_j = verticies_[3]->Paramerters();
			Qd Qj(param_pose_j[6], param_pose_j[3], param_pose_j[4], param_pose_j[5]);
			Vec3 Pj = Param_pose_i.head<3>();

			VecX param_v_bias_j = verticies_[4]->Paramerters();
			Vec3 Vj = param_v_bias_j.head<3>();
			Vec3 Baj = param_v_bias_j.segment(3, 3);
			Vec3 Bgj = param_v_bias_j.tail<3>();

			residual_ = pre_integration_->evaluate(Pi, Qi, Vi, Bai, Bgi, Pj, Qj, Baj, Bgj);

			SetInfomation(pre_integration_->covariance.inverse());
		}

		void EdgeImu::ComputeJacobians() {

			double sum_dt = pre_integration_->sum_dt;
			//template??????????
			Eigen::Matrix3d dp_dba = pre_integration_->jacobian.template block<3, 3>(O_P, O_BA);
			Eigen::Matrix3d dp_dbg = pre_integration_->jacobian.template block<3, 3>(O_P, O_BG);
			Eigen::Matrix3d dq_dbg = pre_integration_->jacobian.template block<3, 3>(O_R, O_BG);
			Eigen::Matrix3d dv_dba = pre_integration_->jacobian.template block<3, 3>(O_V, O_BA);
			Eigen::Matrix3d dv_dbg = pre_integration_->jacobian.template block<3, 3>(O_V, O_BG);

		}

		if (pre_integration_->jacobian.maxCoeff() > 1e8 || pre_integration_->jacobian.minCoeff() < -1e8)
		{
			// ROS_WARN("numerical unstable in preintegration");
		}

		///分别对IMU残差项进行求导，其中utility中定义了各个残差项的求法
		if (jacobian[0])
		{
			Eigen::Matrix<double, 15, 6, Eigen::RowMajor> jacobian_pose_i;///行为主，该雅可比是pi,qi,即pose i时刻的雅可比
			jacobian_pose_i.setZero();

			jacobian_pose_i_block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
			jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));
			Eigen::Quaterniond corrected_delta_q = pre_integration_->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg));
			jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
			jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));
		}
		///对vi,bai,bgi的雅可比求解
		if (jacobians[1])
		{
			Eigen::Matrix<double, 15, 9, Eigen::RowMajor> jacobian_speedbias_i;
			jacobian_speedbias_i.setZero();
			jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
			jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
			jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;
			jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration_->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
			jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
			jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
			jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;
			jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();
			jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();
			jacobians_[1] = jacobian_speedbias_i;
		}
		///对pj、qj求导
		if (jacobians[2])
		{
			Eigen::Matrix<double, 15, 6, Eigen::RowMajor> jacobian_pose_j;
			jacobian_pose_j.setZero();
			jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();
			Eigen::Quaterniond corrected_delta_q = pre_integration_->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg));
			jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
			jacobians_[2] = jacobian_pose_j;
		}
		///对vj、baj、bgj求导
		if (jacobians[3])
		{
			Eigen::Matrix<double, 15, 9, Eigen::RowMajor> jacobian_speedbias_j;
			jacobian_speedbias_j.setZero();
			jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();
			jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();
			jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

			//        jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;
			jacobians_[3] = jacobian_speedbias_j;
		}
	}
}