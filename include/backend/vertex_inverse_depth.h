#ifndef BACKEND_INVERSE_DEPTH_H
#define BACKEND_INVERSE_DEPTH_H

#include "vertex.h"

namespace vio {
	namespace backend {

		/**
		 * 以逆深度形式存储的顶点
		 */
		class VertexInverseDepth : public Vertex {
		public:
		   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		    VertexInverseDepth() : Vertex(1) {}

		    virtual std::string TypeInfo() const { return "VertexInverseDepth"; }
		};

	}
}

#endif
