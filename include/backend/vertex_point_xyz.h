#ifndef BACKEND_POINTVERTEX_H
#define BACKEND_POINTVERTEX_H

#include "vertex.h"

namespace vio {
	namespace backend {

		/**
		 * @brief 以xyz形式参数化的顶点
		 */
		class VertexPointXYZ : public Vertex {
		public:
		   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		   VertexPointXYZ() : Vertex(3) {}

		   std::string TypeInfo() const { return "VertexPointXYZ"; }
		};

	}
}

#endif
