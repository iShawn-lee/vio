#ifndef BACKEND_LOSS_FUNCTION_H
#define BACKEND_LOSS_FUNCTION_H

#include "eigen_types.h"

namespace vio {
	namespace backend {
		/**
		 * 计算加权残差，e^T*Omega*e
		 * 输出rho为
		 * rho[0]: 加权重后的残差
		 * rho[1]：rho[0]的一阶导数first derivative of the scaling function
		 * rho[2]: 二阶导数
		 * LossFunction是核函数未定的基类，可派生出各类核函数下的Loss
		*/
		
		class LossFunction {
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
			
			virtual void Compute(double err2, Eigen::Vector3d& rho) const = 0;
		};

		///不加任何核函数普通的loss
		class TrivalLoss : public LossFunction {
		public:
			virtual void Compute(double err2, Eigen::Vector3d& rho) const override
			{
				// TODO:: whether multiply 1/2
				rho[0] = err2;
				rho[1] = 1;
				rho[2] = 0;
			}
		};

		/**
		 * Huber loss
		 * Huber(e) = e^2                      if e <= delta
		 * huber(e) = delta*(2*e - delta)      if e > delta
		 */
		class HuberLoss : public LossFunction {
		public:
			explicit HuberLoss(double delta) : delta_(delta) {}

			virtual void Compute(double err2, Eigen::Vector3d& rho) const override;

		private:
			double delta_;

		};

		/**
		 * Cauchy loss
		 *
		 */
		class CauchyLoss : public LossFunction
		{
		public:
			explicit CauchyLoss(double delta) : delta_(delta) {}

			virtual void Compute(double err2, Eigen::Vector3d& rho) const override;

		private:
			double delta_;
		};

		class TukeyLoss : public LossFunction
		{
		public:
			explicit TukeyLoss(double delta) : delta_(delta) {}

			virtual void Compute(double err2, Eigen::Vector3d& rho) const override;

		private:
			double delta_;
		};
	}
}
#endif
