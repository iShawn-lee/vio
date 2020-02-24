#ifndef BACKEND_POSE_VERTEX_H
#define BACKEND_POSE_VERTEX_H

#include"vertex.h"

namespace vio {
	namespace backend {
		/**
		 * parameters: tx, ty, tz, qx, qy, qz, qw, 7 DoF
		 * 由于优化变量是在so3流形上进行的，所以本地维度是6自由度，用左乘加法运算
		 */
		class VertexPose : public Vertex {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			VertexPose() :vertex(7,6){}

			virtual void Plus(const VecX &delta) override;

			std::string TypeInfo() const {
				return "VertexPose";
			}

		};
	}
}
#endif