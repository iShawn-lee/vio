#ifndef BACKEND_PROBLEM_H
#define BACKEND_PROBLEM_H

#include <unordered_map>
#include <map>
#include <memory>

#include "edge.h"
#include "vertex.h"
#include "eigen_types.h"

typedef unsigned long ulong;

namespace vio {
	namespace backend {
		typedef std::map<ulong, std::shared_ptr<Vertex> > HashVertex;
		typedef std::unordered_map<ulong, std::shared_ptr<edge> > Hashedge;
		typedef std::unordered_multimap <ulong, std::shared_ptr <edge> > HashVertexIdToEdge;

		class Problem {
		public:
			
			EIGEN_MAKE_ALIGEND_OPERATOR_NEW;

			Problem();
			~Problem();

			bool AddVertex(std::shared_ptr<Vertex> vertex);
			bool RemoveVertex(std::shared_ptr<Vertex> vertex);

			bool AddEdge(std::shared_ptr<Edge> edge);
			bool RemoveEdge(std::shared_ptr<Edge> edge);

			///取outlier的边
			// void GetOutlierEdges(std::vector<std::shared_ptr<Edge>> &outlier_edge);

			bool Solve(int iterations = 15);
			///边缘化该帧及对应路标点
			bool Marginalize(std::shared_ptr<Vertex> poseVertex,
				const std::vector<std::shared_ptr<Vertex>> &landmarkVericies);
			//bool Marginalize(const std::shared_ptr<Vertex> &poseVertex);
			//bool Matginalize(const std::shared_ptr<Vertex> &poseVertex,int pose_dim);
			
			///先进行扩容的预备矩阵
			MatXX GetHessianPrior() { return H_prior_; }
			VecX GetbPrior() { return b_prior_; }
			VecX GetErrPrior() { return err_prior_; }
			MatXX GetJtPrior() { return Jt_prior_inv_; }

			void ExtendHessiansPriorSize(int dim);

		private:
			
			///设置个顶点的顺序建立索引（准备部分）
			void SetOrdering();

			///设置新顶点的顺序（准备部分）
			void AddOrdering(std::shared_ptr<Vertex> v);
			void ComputeLambdaInitLM();

			///solve的计算部分
			void MakeHessian();
			void AddLambdatoHessianLM();
			void RemoveLambdaHessianLM();
			void SchurSBA();
			void SolveLinearEquation();
			///判断LM算法在上次迭代中是否满足误差下降等要求，以及进行lambda变化
			bool IsGoodStepInLM();
			void UpdateStates();
			///当残差变大时回滚
			void RollbackStates();
			void ComputePrior();
			/// PCG 迭代线性求解器
			VecX PCGSolver(const MatXX &A, const VecX &b, int maxIter);
			///判断类对象类型，方便计算维度使用
			bool IsPoseVertex();
			bool IsLandmarkVertex(std::shared_ptr<Vertex> vertex);

			void ResizePoseHessiansWhenAddingPose(std::shared_ptr<Vertex> v);

			///horrible debug.....
			bool CheckOrdering();
			void LogoutVectorSize();
			//得到点所连接的对应边
			std::vector<std::shared_ptr<Edge> > GetConnectedEdges(std::shared_ptr<Vertex>);
		private:

			//all verticies and edges
			HashVertex verticies_;
			Hashedge edges_;
			
			///由vertex id查找对应的edge
			HashVertexIdToEdge vertexToEdge_;

			MatXX H_prior_;
			VecX b_prior;
			VecX err_prior_;
			MatXX Jt_prior_inv_;

			double currentLambda_;
			double currenChi_;
			double stopThresholdLM_;
			double ni_;

			///正规方程参数
			MatXX Hessian_;
			VecX b_;
			VecX delta_x_;

			///SBA的pose部分，用于简化求解计算
			MatXX H_pp_schur_;
			VecX b_pp_schur_;
			///hessian的pose和landmark部分
			MatXX H_pp_;
			VecX b_pp;
			MatXX H_ll_;
			VecX b_ll_;

			///vertex排序
			ulong ordering_poses_ = 0;
			ulong ordering_landmarks_ = 0;
			ulong ordering_generic_ = 0;
			///用于构建H矩阵中位姿和路标点的对应的位置
			HashVertex idx_pose_vertices_;
			HashVertex idx_landmark_vertices_;
			/// the vertex needs to marg(vertex's ordering_id, vertex) 
			HashVertex verticies_marg;

			bool bDebug = false;
			///time to solve
			double t_hessian_cost_ = 0.0;
			double t_PCG_cost_ = 0.0;
		};
	}
}
