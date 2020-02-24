#ifndef BACKEND_VERTEX_H
#define BACKEND_VERTEX_H

#include "eigen_types.h"

namespace vio {
	namespace backend {
		extern global_vertex_id;

		class Vertex {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
			///使Class object = int形式赋值失效 
			explicit Vertex(int num_dimension, int local_dimension = -1);

			virtual ~Vertex();

			///返回变量维度
			int Dimension() const;

			///返回本地变量维度（）,实际优化不一定是输入变量，比如旋转的相加是在so3流形上，而输入是SO3
			int LocalDimension();

			unsigned long Id() const { return id_; }
			int OrderingId() const { return ordering_id_; }
			void SetOrderingId(unsigned long id) { ordering_id_ = id; }

			///参数即优化状态量
			VecX Parameters()const { return parameters_; }
			VecX &Parameters() { return parameters_; }
			void SetParameters(const VecX &params) { parameters_ = params; }
			///备份和回滚
			void BackUpParameters() { parameters_ = params; }
			void RollBackParameters() { parameters_ = parameters_backup_; }
			
			void SetFixed(bool fixed = true) {
				fixed_ = fixed;
			}
			bool IsFixed() const { return fixed_; }

			virtual void Plus(const VecX &delta);

			virtual std::String TypeInfo() const = 0;

		protected:
			unsigned long id_;
			VecX parameters_;
			VecX parameters_backup_; // 参数备份，用于回滚
			int local_dimension_;
			/// ordering id指H矩阵中的雅克比块，例如ordering_id=6则对应Hessian中的第6列
			/// 因为数组默认0起步，所以初值为0；
			unsigned long ordering_id_ = 0;

			bool fixed_ = false;
		};
	}
}

#endif // !BACKEND_VERTEX_H

