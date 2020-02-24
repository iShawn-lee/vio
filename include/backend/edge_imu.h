#ifndef BACKEND_IMU_EDGE_H
#define BACKEN_IMU_EDGE_H

#include <memory>
#include <string>
#include "edge.h"
#include "parameter.h"
#include "integration_base.h"

namespace vio {
	namespace backend {
		
		/**
		 *此为四元边，分别是pose_i,bias_i,pose_j,bias_j
		 */
		class EdgeImu :public Edge {
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		public:

			explicit EdgeImu(IntegrationBase* _pre_integration) :pre_integration_(_pre_integration),
				Edge(15, 4, std::vector<std::string>{"VertexPose", "VertexSpeedBias", "VertexPose", "VertexSpeedBias"} {
			}
			virtual std::string TypeInfo() const override{return "EdgeImu";}
			virtual void ComputeResidual() override;
			virtual void ComputeJacobinas() override;
		private:
			///维度信息用于协方差传播时雅可比矩阵的序号
			enum StateOrder
			{
				O_P = 0;
				O_R = 3;
				O_V = 6;
				O_BA = 9;
				O_BG = 12;
			};
			IntegrationBase* _pre_integration_;
			static Vec3 gravity_;

			//p、v、q相对于ba、bg的雅可比矩阵不同矩阵块，用于IMU误差传播
			Mat33 dp_dba_ = Mat33::Zero();
			Mat33 dp_dbg_ = Mat33::Zero();
			Mat33 dr_dbg_ = Mat33::Zero();
			Mat33 dv_dba_ = Mat33::Zero();
			Mat33 dv_dbg_ = Mat33::Zero();
		};
	}
}

#endif
