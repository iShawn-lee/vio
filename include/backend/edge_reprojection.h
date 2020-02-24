#ifndef BACKEND_REPROJECTION_EDGE_H
#define BACKEND_REPROJECTION_EDGE_H

#include"edge.h"

namespace vio {
	namespace backend {
		/**
		 * 视觉重投影误差，为三元边，与之相连的顶点有：
		 * 路标点的逆深度InveseDepth、第一次观测到该路标点的source Camera的位姿T_World_From_Body1，
		 * 和观测到该路标点的measurement Camera位姿T_World_From_Body2。
		 * 该边中verticies_顶点顺序必须为InveseDepth、T_World_From_Body1、T_World_From_Body2。
		 */

		class EdgeReprojection : public Edge() {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			EdgeReprojection(const Vec3 &pts_i, const Vec3 &pts_j)
				: Edge(2, 3, std::vector<std::string>{"VertexInverseDepth","vertexPose", "VertexPose"}) {
				pts_i_ = pts_i;
				pts_j_ = pts_j;
			}

			virtual std::string TypeInfo() const override {
				return "Edge_Reprojection";
			}

			virtual void ComputeResidual() override;
			virtual void ComputeJacobians() override;

		private:
			Vec3 pts_i_;
			Vec3 pts_j_;
			}

		/**
		 * 二元边的视觉重投影误差，与之相连的顶点有：
		 * 路标点的世界坐标系XYZ、观测到该路标点的 Camera 的位姿T_World_From_Body1
		 * verticies_顶点顺序必须为 XYZ、T_World_From_Body1。
		 */
		class EdgeReprojectionXYZ : public Edge{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			EdgeReprojectionXYZ(const Vec3 &pts_i)
				:edge(2, 2, std::vector<std::string>{"VertexXYZ", "VertexPose"}) {
				obs_ = pts_i;
			}

			virtual std::string TypeInfo() const override { return "EdgeReprojectionXYZ"; }
			
			virtual void ComputeResidual() override;
			virtual void ComputeJacobians() override;

			void SetTranslationImuFromCamera(Qd &qic, ; Vec *tic_);

		private:
			Qd qic_;
			Vec3 tic_;
			Vec3 obs_;
		}
	}
}

#endif
