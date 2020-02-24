#ifndef BACKEND_EDGE_H
#define BACKEND_EDGE_H

#include<memory>
#include<string>
#include "eigen_types.h"
#include <eigen3/Eigen/Dense>
#include"loss_function.h"
#endif // !BACKEND_EDGE_H

namespace vio {
	namespace backend {
		class Vertex;
		
		class Edge {
		public:
			EIGEN_MAKE_ALIGEND_OPERATOR_NEW;

			///构造函数直接对雅可比矩阵的维度进行变化(参数传入类型检查可以不做），单参数再用explicit
			Edge(int residual_dimension, int num_verticies,
				const std::vector<std::string> &verticies_types = std::vector<std::string>());
			virtual ~edge();

			unsigned long GetId() const { return id_; }

			bool AddVertex(std::shared_ptr<Vertex> vertex) {
				verticies_.emplace_back(vertex);
				return true;
			}

			bool SetVertex(const std::shared_ptr<Vertex> &vertex) {
				verticies _= verticies;
				return true;
			}

			std::shared_ptr<Vertex> GetVertex(int i) {
				return verticies_[i];
			}

			std::vector<std::shared_ptr<Vertex>> AllVerticies() const {
				return verticies_;
			}

			size_t NumVertices() const { return verticies_size(); }

			virtual std::string TypeInfo() const = 0;

			virtual void ComputeResidual() = 0;
			virtual void ComputeJacobians() = 0;
			///计算平方误差与加了核函数的误差
			double Chi2();
			double RobustChi2();
			///返回残差
			VecX Residual() const { return residual_; }

			std::vector<MatXX> Jacobians() const { return jacobians_; }

			void SetInformation(const MatXX &information) {
				information_ = information;
				///LLT分解
				sqrt_information_ = Eigen::LLT<MatXX>(information_).matrixL().transpose();
			}

			MatXX Information() const {
				return information_;
			}

			MatXX SqrtInformation()const {
				return sqrt_information_;
			}

			void SetLossFunction(LossFunction* ptr) { lossfunction_ = ptr; }
			LossFunction* GetLossFunction() { return lossfunction_; }
			void RobustInfo(double& drho, MatXX& info) const;

			void SetObservation(const VecX &observation) {
				observation_ = observation;
			}

			VecX Observation() const { return observation_; }

			int OrderingId() const { return ordering_id_; }


		protected:
			std::vector<std::shared_ptr<Vertex> > verticies_;
			VecX observation_;

			VecX residual_;

			unsigned long id_;//edge id
			int ordering_id_;//edge id in problem

			LossFunction* lossfunction_;

			std::vector<MatXX> jacobians_;
			MatXX information_;
			MatXX sqrt_information_;
			unsigned long ordering_id_;

			std::vector<std::string> verticies_types_;  // 各顶点类型信息，用于debug

		};
	}
}

