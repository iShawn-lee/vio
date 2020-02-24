#ifndef BACKEND_SPEEDBIASVERTEX_H
#define BACKEND_SPEEDBIASVERTEX_H

#include <memory>
#include "vertex.h"

namespace vio {
	namespace backend {

		/**
		* Speed and Bias vertex
		* parameters: v, ba, bg 9 DoF
		*/

		class VertexSpeedBias : public Vertex {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			VertexSpeedBias() : Vertex(9) {}

			std::string TypeInfo() const {
			    return "VertexSpeedBias";
			}

		};

	}
}

#endif
