﻿#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include "utility/tic_toc.h"
#include "backend/problem.h"

#ifdef USE_OPENMP
#include <omp.h>
#endif

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

void writeToCSVfile(std::string name, Eigen::MatrixXd matrix) {
	std::ofstream f(name.c_str());
	f << matrix.format(CSVFormat);
}

using namespace std;
typename unsigned long ulong;
namespace vio {
	namespace backend {
		void Problem::LogoutVectorSize() {
			// LOG(INFO) <<
			// " problem::LogoutVectorSize verticies_:" << verticies_.size() <<
			// " edges:" << edges_.size();
		}

		Problem::Problem() {
			LogoutVectorSize();
			verticies_marg_.clear();
		}

		Problem::~Problem() {
			global_vertex_id = 0;
		}

		void Problem::ExtendHessiansPriorSize(int dim)
		{
			int size = H_prior_.rows() + dim;
			H_prior_.conservativeResize(size, size);
			b_prior_.conservativeResize(size);

			b_prior_.tail(dim).setZero();
			H_prior_.rightCols(dim).setZero();
			H_prior_.bottomRows(dim).setZero();
		}

		void Problem::ResizePoseHessiansWhenAddingPose(shared_ptr<Vertex> v) {
			///节点新增后，对H、b提前增加维度，并将相关位置先置0（TODO:函数调用费时间？）
			ExtendHessiansPriorSize(v->LocalDimension())
		}

		bool Problem::AddVertex(std::shared_ptr<Vertex> vertex) {
			if (verticies_.find(vertex->Id()) != verticies_.end()) {
				//LOG(WARNING) <<"Vertex"<<vertex->Id()<<"has beeen added befor";
				return false;
			}
			else {
				verticies_.insert(pair<ulong, shared_ptr<Vertex>>(vertex->Id(), vertex));
			}

			if (IsPoseVertex(vertex)) {
				if (IsPoseVertex(vertex)) {
					ResizePoseHessiansWhenAddingPose(vertex);
				}
			}
			return true;
		}

		void Problem::AddOrderingSLAM(std::shared_ptr<vio::backend::Vertex> v) {
			///设置局部pose和landmark的id以配合H与J的构建,与全局verticies区别，为了分成pose和landmark
			if (IsPoseVertex(v)) {
				v->SetOrderingId(ordering_poses_);
				idx_pose_vertices_.insert(pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
				ordering_poses_ += v->LocalDimension();
			}
			else if (IsLandmarkVertex(v)) {
				v->SetOrderingId(ordering_landmarks_);
				ordering_landmarks_ += v->LocalDimension();
				idx_landmark_vertices_.insert(pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
			}
		}

		bool Problem::IsPoseVertex(std::shared_ptr<vio::backend::Vertex> v) {
			string type = v->TypeInfo();
			return type == string("VertexPose") ||
				type == string("VertexSpeedBias");
		}

		bool Problem::IsLandmarkVertex(std::shared_ptr<vio::backend::Vertex> v) {
			string type = v->TypeInfo();
			return type == string("VertexPointXYZ") ||
				type == string("VertexInverseDepth");
		}

		bool Problem::AddEdge(shared_ptr<Edge> edge) {
			if (edges_.find(edge->Id()) == edges_.end()) {
				edges_.insert(pair<ulong, std::shared_ptr<Edge>>(edge->Id(), edge));
			}
			else {
				// LOG(WARNING) << "Edge " << edge->Id() << " has been added before!";
				return false;
			}
			//记录每个顶点对应的边
			for (auto &vertex : edge->Verticies()) {
				vertexToEdge_.insert(pair<ulong, shared_ptr<Edge>>(vertex->Id(), edge));
			}
			return true;
		}

		///返回顶点所对应的边
		vector<shared_ptr<Edge>> Problem::GetConnectedEdges(std::shared_ptr<Vertex> vertex) {
			vector<shared_ptr<Edge>> edges;
			//equal_range(firstiter,lastiter,val) 返回范围key值范围[first,last)内key等于val的子数组的迭代器（子数组为所有multimap中的重复键）
			//返回值是个pair对象，pair的first是左边界的迭代器，pair的second是右边界的迭代器。区间是左闭右开的，[左边界，右边界)
			auto range = vertexToEdge_.equal_range(vertex->Id());
			//map的迭代器first得到key值，second得到value值
			for (auto iter = range.first; iter != range.second; ++iter) {

				// 并且这个edge还需要存在，而不是已经被remove了
				if (edges_.find(iter->second->Id()) == edges_.end())
					continue;

				edges.emplace_back(iter->second);
			}
			return edges;
		}

		bool Problem::RemoveEdge(std::shared_ptr<Edge> edge) {
			//check if the edge is in map_edges_
			if (edges_.find(edge->Id()) == edges_.end()) {
				// LOG(WARNING) << "The edge " << edge->Id() << " is not in the problem!" << endl;
				return false;
			}

			edges_.erase(edge->Id());
			return true;
		}

		bool Problem::RemoveVertex(std::shared_ptr<Vertex> vertex) {
			//check if the vertex is in map_verticies_
			if (verticies_.find(vertex->Id()) == verticies_.end()) {
				// LOG(WARNING) << "The vertex " << vertex->Id() << " is not in the problem!" << endl;
				return false;
			}
			// 这里要 remove 该顶点对应的 edge.
			vector<shared_ptr<Edge>> remove_edges = GetConnectedEdges(vertex);
			for (size_t i = 0; i < remove_edges.size(); i++) {
				RemoveEdge(remove_edges[i]);
			}

			if (IsPoseVertex(vertex))
				idx_pose_vertices_.erase(vertex->Id());
			else
				idx_landmark_vertices_.erase(vertex->Id());

			vertex->SetOrderingId(-1);      // used to debug
			verticies_.erase(vertex->Id());
			vertexToEdge_.erase(vertex->Id());

			return true;
		}

		void Problem::SetOrdering() {

			/// new problem uses new parameters
			ordering_poses_ = 0;
			ordering_generic_ = 0;
			ordering_landmarks_ = 0;

			for (auto vertex : verticies_) {
				ordering_generic_ += vertex.second->LocalDimension();  // 所有的优化变量总维数
				AddOrderingSLAM(vertex.second);//统计pose和landmark的维度，为H矩阵构建做准备
			}

			// 这里把 landmark 的 ordering 重设置为加上 pose 的数量，就保持了 landmark 在后,而 pose 在前
			ulong all_pose_dimension = ordering_poses_;
			for (auto landmarkVertex : idx_landmark_vertices_) {
				landmarkVertex.second->SetOrderingId(
					landmarkVertex.second->OrderingId() + all_pose_dimension
				);
			}
			//    CHECK_EQ(CheckOrdering(), true);
		}

		void Problem::MakeHessian() {
			TicToc t_h;
			// 直接构造大的 H 矩阵
			ulong size = ordering_generic_;
			MatXX H(MatXX::Zero(size, size));
			VecX b(VecX::Zero(size));

			#ifdef USE_OPENMP
			#pragma omp parallel for num_threads(4)
			#endif
			for (auto &edge : edges_) {

				edge.second->ComputeResidual();
				edge.second->ComputeJacobians();

				// TODO:: robust cost
				auto jacobians = edge.second->Jacobians();
				auto verticies = edge.second->Verticies();
				assert(jacobians.size() == verticies.size());
				for (size_t i = 0; i < verticies.size(); ++i) {
					auto v_i = verticies[i];
					if (v_i->IsFixed()) continue;    // Hessian 里不需要添加它的信息

					auto jacobian_i = jacobians[i];
					ulong index_i = v_i->OrderingId();
					ulong dim_i = v_i->LocalDimension();

					// 鲁棒核函数会修改残差和信息矩阵，如果没有设置 robust cost function，就会返回原来的
					double drho;
					MatXX robustInfo(edge.second->Information().rows(), edge.second->Information().cols());
					edge.second->RobustInfo(drho, robustInfo);

					MatXX JtW = jacobian_i.transpose() * robustInfo;
					for (size_t j = i; j < verticies.size(); ++j) {
						auto v_j = verticies[j];

						if (v_j->IsFixed()) continue;

						auto jacobian_j = jacobians[j];
						ulong index_j = v_j->OrderingId();
						ulong dim_j = v_j->LocalDimension();

						assert(v_j->OrderingId() != -1);//debug
						MatXX hessian = JtW * jacobian_j;

						// 所有的信息矩阵叠加起来
						H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
						if (j != i) {
							// 对称的下三角
							H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();

						}
					}
					b.segment(index_i, dim_i).noalias() -= drho * jacobian_i.transpose()* edge.second->Information() * edge.second->Residual();
				}

			}
			Hessian_ = H;
			b_ = b;
			t_hessian_cost_ += t_h.toc();

			if (H_prior_.rows() > 0)
			{
				MatXX H_prior_tmp = H_prior_;
				VecX b_prior_tmp = b_prior_;

				// 遍历所有 POSE 顶点，然后设置相应的fix的顶点先验维度上值为0.
				for (auto vertex : verticies_) {
					if (IsPoseVertex(vertex.second) && vertex.second->IsFixed()) {
						int idx = vertex.second->OrderingId();
						int dim = vertex.second->LocalDimension();
						H_prior_tmp.block(idx, 0, dim, H_prior_tmp.cols()).setZero();
						H_prior_tmp.block(0, idx, H_prior_tmp.rows(), dim).setZero();
						b_prior_tmp.segment(idx, dim).setZero();
						// std::cout << " fixed prior, set the Hprior and bprior part to zero, idx: "<<idx <<" dim: "<<dim<<std::endl;
					}
				}
				///前面已经将pose放在了前面,fix处的雅可比为0，同时添加先验信息时先验部分也应设置成0
				Hessian_.topLeftCorner(ordering_poses_, ordering_poses_) += H_prior_tmp;
				b_.head(ordering_poses_) += b_prior_tmp;
			}

			delta_x_ = VecX::Zero(size);  // initial delta_x = 0_n;
		}

		///算初始lamda以及LM其他参数赋初值
		void Problem::ComputeLambdaInitLM() {
			ni_ = 2.;
			currentLambda_ = -1.;
			currentChi_ = 0.0;

			for (auto edge : edges_) {
				currentChi_ += edge.second->RobustChi2();
			}
			if (err_prior_.rows() > 0)
				currentChi_ += err_prior_.norm();
			currentChi_ *= 0.5;

			stopThresholdLM_ = 1e-6 * currentChi_;   // 迭代条件1为 误差下降 1e-6 倍

			double maxDiagonal = 0;
			ulong size = Hessian_.cols();
			assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
			for (ulong i = 0; i < size; ++i) {
				maxDiagonal = std::max(fabs(Hessian_(i, i)), maxDiagonal);
			}

			maxDiagonal = std::min(5e10, maxDiagonal);//防止过大
			double tau = 1e-5;  // 1e-5
			currentLambda_ = tau * maxDiagonal;
			//        std::cout << "currentLamba_: "<<maxDiagonal<<" "<<currentLambda_<<std::endl;
		}

		/*
		void Problem::AddLambdatoHessianLM() {
			ulong size = Hessian_.cols();
			assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
			for (ulong i = 0; i < size; ++i) {
				Hessian_(i, i) += currentLambda_;
			}
		}

		void Problem::RemoveLambdaHessianLM() {
			ulong size = Hessian_.cols();
			assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
			// TODO:: 这里不应该减去一个，数值的反复加减容易造成数值精度出问题.直接变成在舒尔补运算里加lambda
			for (ulong i = 0; i < size; ++i) {
				Hessian_(i, i) -= currentLambda_;
			}
		}
		*/

		void Problem::SolveLinearSystem() {

			//        TicToc t_Hmminv;
			// step1: schur marginalization --> Hpp, bpp
			// 利用矩阵稀疏性简化计算，分为l和m两个大块，方便求逆
			int reserve_size = ordering_poses_;
			int marg_size = ordering_landmarks_;
			MatXX Hmm = Hessian_.block(reserve_size, reserve_size, marg_size, marg_size);
			MatXX Hpm = Hessian_.block(0, reserve_size, reserve_size, marg_size);
			MatXX Hmp = Hessian_.block(reserve_size, 0, marg_size, reserve_size);
			VecX bpp = b_.segment(0, reserve_size);
			VecX bmm = b_.segment(reserve_size, marg_size);

			// Hmm 是对角线矩阵，它的求逆可以直接为对角线块分别求逆，如果是逆深度，对角线块为1维的，则直接为对角线的倒数，这里可以加速
			MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
			// TODO:: use openMP
			for (auto landmarkVertex : idx_landmark_vertices_) {
				int idx = landmarkVertex.second->OrderingId() - reserve_size;
				int size = landmarkVertex.second->LocalDimension();
				///分别对不同的对角块求逆
				Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
			}

			MatXX tempH = Hpm * Hmm_inv;
			H_pp_schur_ = Hessian_.block(0, 0, ordering_poses_, ordering_poses_) - tempH * Hmp;//C\D为0，求a不需要C\D
			b_pp_schur_ = bpp - tempH * bmm;

			// step2: solve Hpp * delta_x = bpp
			VecX delta_x_pp(VecX::Zero(reserve_size));

			for (ulong i = 0; i < ordering_poses_; ++i) {
				H_pp_schur_(i, i) += currentLambda_;              // LM Method
			}

			// TicToc t_linearsolver;
			delta_x_pp = H_pp_schur_.ldlt().solve(b_pp_schur_);//  SVec.asDiagonal() * svd.matrixV() * Ub;    
			delta_x_.head(reserve_size) = delta_x_pp;
			// std::cout << " Linear Solver Time Cost: " << t_linearsolver.toc() << std::endl;

			// step3: solve Hmm * delta_x = bmm - Hmp * delta_x_pp;
			VecX delta_x_ll(marg_size);
			delta_x_ll = Hmm_inv * (bmm - Hmp * delta_x_pp);
			delta_x_.tail(marg_size) = delta_x_ll;

			//        std::cout << "schur time cost: "<< t_Hmminv.toc()<<std::endl;
		}



		void Problem::UpdateStates() {

			// update vertex
			for (auto vertex : verticies_) {
				vertex.second->BackUpParameters();    // 保存上次的估计值

				ulong idx = vertex.second->OrderingId();
				ulong dim = vertex.second->LocalDimension();
				VecX delta = delta_x_.segment(idx, dim);
				vertex.second->Plus(delta);
			}

			// update prior
			if (err_prior_.rows() > 0) {
				// BACK UP b_prior_
				b_prior_backup_ = b_prior_;
				err_prior_backup_ = err_prior_;

				/// prior Jacobian update with first order Taylor, b' = b + H * delta x
				/// \delta x = Computes the linearized deviation from the references (linearization points)
				b_prior_ -= H_prior_ * delta_x_.head(ordering_poses_);
				err_prior_ = -Jt_prior_inv_ * b_prior_.head(ordering_poses_ - 15); //????????-15

				//        std::cout << "                : "<< b_prior_.norm()<<" " <<err_prior_.norm()<< std::endl;
				//        std::cout << "     delta_x_ ex: "<< delta_x_.head(6).norm() << std::endl;
			}
		}

		void Problem::RollbackStates() {

			// update vertex
			for (auto vertex : verticies_) {
				vertex.second->RollBackParameters();
			}

			// Roll back prior_
			if (err_prior_.rows() > 0) {
				b_prior_ = b_prior_backup_;
				err_prior_ = err_prior_backup_;
			}
		}

		bool Problem::IsGoodStepInLM() {
			//scale为预测下降值
			double scale = 0;
			scale = 0.5* delta_x_.transpose() * (currentLambda_ * delta_x_ + b_);
			scale += 1e-6;    // make sure it's non-zero :)

			// recompute residuals after update state
			double tempChi = 0.0;
			for (auto edge : edges_) {
				edge.second->ComputeResidual();
				tempChi += edge.second->RobustChi2();
			}
			if (err_prior_.size() > 0)
				tempChi += err_prior_.norm();
			tempChi *= 0.5;          // 1/2 * err^2

			double rho = (currentChi_ - tempChi) / scale;
			if (rho > 0 && isfinite(tempChi))   // last step was good, 误差在下降
			{
				double alpha = 1. - pow((2 * rho - 1), 3);
				alpha = std::min(alpha, 2. / 3.);
				double scaleFactor = (std::max)(1. / 3., alpha);
				currentLambda_ *= scaleFactor;
				ni_ = 2;
				currentChi_ = tempChi;
				return true;
			}
			else {
				currentLambda_ *= ni_;
				ni_ *= 2;
				return false;
			}
		}

		bool Problem::Solve(int iterations) {

			if (edges_.size() == 0 || verticies_.size() == 0) {
				std::cerr << "\nCannot solve problem without edges or verticies" << std::endl;
				return false;
			}

			TicToc t_solve;
			/// 统计优化变量的维数，为构建 H 矩阵做准备
			SetOrdering();
			// 遍历edge, 构建 H 矩阵
			MakeHessian();
			// LM 初始化
			ComputeLambdaInitLM();
			// LM 算法迭代求解
			bool stop = false;
			int iter = 0;
			double last_chi_ = 1e20;
			while (!stop && (iter < iterations)) {
				std::cout << "iter: " << iter << " , chi= " << currentChi_ << " , Lambda= " << currentLambda_ << std::endl;
				bool oneStepSuccess = false;
				int false_cnt = 0;
				while (!oneStepSuccess && false_cnt < 10)  // 不断尝试不同的Lambda, 直到成功迭代一步
				{
					// 在solveliner那步加了，暂时不用addlambda
					//AddLambdatoHessianLM();
					// 第四步，解线性方程
					SolveLinearSystem();
					//RemoveLambdaHessianLM();

					// 优化退出条件1： delta_x_ 很小则退出
					if (delta_x_.squaredNorm() <= 1e-6 || false_cnt > 10)
						// TODO:: 退出条件有问题, 好多次误差都没变化了还在迭代计算，应该搞一个误差不变了就中止
					{
						stop = true;
						break;
					}

					// 更新状态量
					UpdateStates();
					// 判断当前步是否可行以及 LM 的 lambda 怎么更新
					oneStepSuccess = IsGoodStepInLM();
					// 后续处理，
					if (oneStepSuccess) {
						//  std::cout << " get one step success\n";

						// 在新线性化点 构建 hessian
						MakeHessian();

						false_cnt = 0;
					}
					else {
						false_cnt++;
						RollbackStates();   // 误差没下降，回滚
					}
				}
				iter++;

				// 优化退出条件2： currentChi_ 跟上一次的 chi2 相比，下降了 1e6 倍则退出
				if (last_chi_ - currentChi_ < 1e-5)
				{
					std::cout << "sqrt(currentChi_) <= stopThresholdLM_" << std::endl;
					stop = true;
				}
				last_chi_ = currentChi_;
			}
			std::cout << "problem solve cost: " << t_solve.toc() << " ms" << std::endl;
			std::cout << "   makeHessian cost: " << t_hessian_cost_ << " ms" << std::endl;
			t_hessian_cost_ = 0.;
			return true;
		}


		///分开marg，先Marg landmark再mark frame
		bool Problem::Marginalize(const std::vector<std::shared_ptr<Vertex> > margVertexs, int pose_dim) {

			SetOrdering();
			/// 找到需要 marg 的 edge, margVertexs[0] is frame, its edge contained pre-intergration
			std::vector<shared_ptr<Edge>> marg_edges = GetConnectedEdges(margVertexs[0]);

			std::unordered_map<int, shared_ptr<Vertex>> margLandmark;
			// 构建 Hessian 的时候 pose 的顺序不变，landmark的顺序要重新设定
			int marg_landmark_size = 0;
			//    std::cout << "\n marg edge 1st id: "<< marg_edges.front()->Id() << " end id: "<<marg_edges.back()->Id()<<std::endl;
			for (size_t i = 0; i < marg_edges.size(); ++i) {
				//        std::cout << "marg edge id: "<< marg_edges[i]->Id() <<std::endl;
				auto verticies = marg_edges[i]->Verticies();
				for (auto iter : verticies) {
					if (IsLandmarkVertex(iter) && margLandmark.find(iter->Id()) == margLandmark.end()) {
						iter->SetOrderingId(pose_dim + marg_landmark_size);
						margLandmark.insert(make_pair(iter->Id(), iter));
						marg_landmark_size += iter->LocalDimension();
					}
				}
			}
			//    std::cout << "pose dim: " << pose_dim <<std::endl;
			int cols = pose_dim + marg_landmark_size;
			/// 构建误差 H 矩阵 H = H_marg + H_pp_prior
			MatXX H_marg(MatXX::Zero(cols, cols));
			VecX b_marg(VecX::Zero(cols));
			int ii = 0;
			///对marge的边进行H\b矩阵求解
			for (auto edge : marg_edges) {
				edge->ComputeResidual();
				edge->ComputeJacobians();
				auto jacobians = edge->Jacobians();
				auto verticies = edge->Verticies();
				ii++;

				assert(jacobians.size() == verticies.size());
				for (size_t i = 0; i < verticies.size(); ++i) {
					auto v_i = verticies[i];
					auto jacobian_i = jacobians[i];
					ulong index_i = v_i->OrderingId();
					ulong dim_i = v_i->LocalDimension();

					double drho;
					MatXX robustInfo(edge->Information().rows(), edge->Information().cols());
					edge->RobustInfo(drho, robustInfo);

					for (size_t j = i; j < verticies.size(); ++j) {
						auto v_j = verticies[j];
						auto jacobian_j = jacobians[j];
						ulong index_j = v_j->OrderingId();
						ulong dim_j = v_j->LocalDimension();

						MatXX hessian = jacobian_i.transpose() * robustInfo * jacobian_j;

						assert(hessian.rows() == v_i->LocalDimension() && hessian.cols() == v_j->LocalDimension());

						H_marg.block(index_i, index_j, dim_i, dim_j) += hessian;
						if (j != i) {

							H_marg.block(index_j, index_i, dim_j, dim_i) += hessian.transpose();
						}
					}
					b_marg.segment(index_i, dim_i) -= drho * jacobian_i.transpose() * edge->Information() * edge->Residual();
				}

			}
			std::cout << "edge factor cnt: " << ii << std::endl;

			/// marg landmark
			int reserve_size = pose_dim;
			if (marg_landmark_size > 0) {
				int marg_size = marg_landmark_size;
				MatXX Hmm = H_marg.block(reserve_size, reserve_size, marg_size, marg_size);
				MatXX Hpm = H_marg.block(0, reserve_size, reserve_size, marg_size);
				MatXX Hmp = H_marg.block(reserve_size, 0, marg_size, reserve_size);
				VecX bpp = b_marg.segment(0, reserve_size);
				VecX bmm = b_marg.segment(reserve_size, marg_size);

				/// 和前面的对角线求逆一样
				MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
				
				TODO:accelerate
				for (auto iter : margLandmark) {
					int idx = iter.second->OrderingId() - reserve_size;
					int size = iter.second->LocalDimension();
					Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
				}

				MatXX tempH = Hpm * Hmm_inv;
				MatXX Hpp = H_marg.block(0, 0, reserve_size, reserve_size) - tempH * Hmp;
				bpp = bpp - tempH * bmm;

				H_marg = Hpp;
				b_marg = bpp;
			}

			VecX b_prior_before = b_prior_;
			if (H_prior_.rows() > 0)
			{
				H_marg += H_prior_;
				b_marg += b_prior_;
			}

			/// marg frame and speedbias
			int marg_dim = 0;

			// index 大的先移动
			for (int k = margVertexs.size() - 1; k >= 0; --k)
			{

				int idx = margVertexs[k]->OrderingId();
				int dim = margVertexs[k]->LocalDimension();
				//        std::cout << k << " "<<idx << std::endl;
				marg_dim += dim;
				// move the marg pose to the Hmm bottown right
				// 将 row i 移动矩阵最下面
				Eigen::MatrixXd temp_rows = H_marg.block(idx, 0, dim, reserve_size);
				Eigen::MatrixXd temp_botRows = H_marg.block(idx + dim, 0, reserve_size - idx - dim, reserve_size);
				H_marg.block(idx, 0, reserve_size - idx - dim, reserve_size) = temp_botRows;
				H_marg.block(reserve_size - dim, 0, dim, reserve_size) = temp_rows;

				// 将 col i 移动矩阵最右边
				Eigen::MatrixXd temp_cols = H_marg.block(0, idx, reserve_size, dim);
				Eigen::MatrixXd temp_rightCols = H_marg.block(0, idx + dim, reserve_size, reserve_size - idx - dim);
				H_marg.block(0, idx, reserve_size, reserve_size - idx - dim) = temp_rightCols;
				H_marg.block(0, reserve_size - dim, reserve_size, dim) = temp_cols;

				Eigen::VectorXd temp_b = b_marg.segment(idx, dim);
				Eigen::VectorXd temp_btail = b_marg.segment(idx + dim, reserve_size - idx - dim);
				b_marg.segment(idx, reserve_size - idx - dim) = temp_btail;
				b_marg.segment(reserve_size - dim, dim) = temp_b;
			}

			double eps = 1e-8;
			int m2 = marg_dim;  // marg pose
			int n2 = reserve_size - marg_dim;
			///实对称阵保证可以进行特征值分解
			Eigen::MatrixXd Amm = 0.5 * (H_marg.block(n2, n2, m2, m2) + H_marg.block(n2, n2, m2, m2).transpose());
			///特征值分解
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
			///忽略特征值小的维度，简化Amm求逆运算
			Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd(
				(saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() *
				saes.eigenvectors().transpose();

			///marg掉标记的pose、speed、bias点
			Eigen::VectorXd bmm2 = b_marg.segment(n2, m2);
			Eigen::MatrixXd Arm = H_marg.block(0, n2, n2, m2);
			Eigen::MatrixXd Amr = H_marg.block(n2, 0, m2, n2);
			Eigen::MatrixXd Arr = H_marg.block(0, 0, n2, n2);
			Eigen::VectorXd brr = b_marg.segment(0, n2);
			Eigen::MatrixXd tempB = Arm * Amm_inv;
			H_prior_ = Arr - tempB * Amr;
			b_prior_ = brr - tempB * bmm2;

			///将H利用特征值分解简化
			Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(H_prior_);
			///S为H矩阵的特征值组成的向量
			Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
			Eigen::VectorXd S_inv = Eigen::VectorXd(
				(saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));
			///矩阵单个元素上全部求根号，等同于S.array().sqrt()
			Eigen::VectorXd S_sqrt = S.cwiseSqrt();
			Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
			///S变为对角矩阵（即S元素每个值一一对应对角矩阵对角线上的值）
			Jt_prior_inv_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
			err_prior_ = -Jt_prior_inv_ * b_prior_;
			///J = sqrt(sigma)*Q^T ,Q is matrix with eigenvector
			MatXX J = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
			H_prior_ = J.transpose() * J;
			MatXX tmp_h = MatXX((H_prior_.array().abs() > 1e-9).select(H_prior_.array(), 0));
			H_prior_ = tmp_h;

			// std::cout << "my marg b prior: " <<b_prior_.rows()<<" norm: "<< b_prior_.norm() << std::endl;
			// std::cout << "    error prior: " <<err_prior_.norm() << std::endl;

			// remove vertex and remove edge
			for (size_t k = 0; k < margVertexs.size(); ++k) {
				RemoveVertex(margVertexs[k]);
			}

			for (auto landmarkVertex : margLandmark) {
				RemoveVertex(landmarkVertex.second);
			}

			return true;

		}

		/*
		//dogleg法
		bool Problem::Solve(int iterations) {

			if (edges_.size() == 0 || verticies_.size() == 0) {
				std::cerr << "\nCannot solve problem without edges or verticies" << std::endl;
				return false;
			}

			TicToc t_solve;
			double e1 = 1e-6;
			double e2 = 1e-6;
			double e3 = 1e-6;
			bool stop = false;

			SetOrdering();
			// 计算最速下降法方向hsd_和系数alpha_,以及高斯牛顿法的增量方程H
			Cal_pre_hdl();
			//计算得到高斯牛顿法的步长,和LM法不同之处，失败后不需要再更新并重新解算正规方程
			SolveDlLinearSystem();

			int iter = 0;
			double last_chi_ = 1e15;
			double currentChi_ = 0;
			double radius = 1.0;

			while (!stop && iter < iterations) {
				bool oneStepSuccess = false;
				int false_cnt = 0;
				//判断rho的值是否符合要求，相应更改阈值范围
				while (!oneStepSuccess && false_cnt < 10)
				{
					// 根据上步求出的hsd和hgn确定当前步的优化方向hdl
					FindHdl(radius);

					UpdateStates();
					// 判断当前步是否可行以及dog-leg的阈值更新,更新误差
					oneStepSuccess = IsGoodStepInDL(radius，stop);

					if (oneStepSuccess) {
						//std::cout << " get one step success\n";
						// 计算新的优化方向
						Cal_pre_hdl();
						SolveLinearSystem();

						false_cnt = 0;
						// 优化退出4： 误差变化足够小
						if (last_chi_ - currentChi_ <= e3)
						{
							std::cout << "The algorithm convergence has been found with small error change" << std::endl;
							stop = true;
						}
					}
					else {
						false_cnt++;
						RollbackStates();   // 误差没下降，回滚
					}
				}
				//std::cout << "iter: " << iter << " , chi= " << currentChi_ << std::endl;

				last_chi_ = currentChi_;
				iter++;
			}
			std::cout << "problem solve cost: " << t_solve.toc() << " ms" << std::endl;
			t_hessian_cost_ = 0.;
			return true;
		}

		void Problem::Cal_pre_hdl() {
			TicToc t_h;
			ulong size = ordering_generic_;

			VecX b(VecX::Zero(size));
			MatXX H(MatXX::Zero(size, size));

			int size_J = 0;
			///统计残差项的数目
			for (auto &edge : edges_) {
				size_J += edge.second->Residual().size();
			}

			VecX J(MatXX::Zero(size_J, size));
			VecX Jg(VecX::Zero(size_J));

			int ordering_J=0;
			#ifdef USE_OPENMP
			#pragma omp parallel for num_threads(4)
			#endif
			for (auto &edge : edges_) {

				edge.second->ComputeResidual();
				edge.second->ComputeJacobians();

				auto jacobians = edge.second->Jacobians();
				auto verticies = edge.second->Verticies();
				assert(jacobians.size() == verticies.size());

				for (size_t i = 0; i < verticies.size(); ++i) {
					auto v_i = verticies[i];
					if (v_i->IsFixed()) continue;    

					auto jacobian_i = jacobians[i];
					ulong index_i = v_i->OrderingId();
					ulong dim_i = v_i->LocalDimension();

					
					J.block(ordering_J, index_i, edge.second->Residual().size(), dim_i) = jacobian_i;

					double drho;
					MatXX robustInfo(edge.second->Information().rows(), edge.second->Information().cols());
					edge.second->RobustInfo(drho, robustInfo);

					MatXX JtW = jacobian_i.transpose()* robustInfo;
					//利用对称性质赋值
					for (size_t j = i; j < verticies.size(); ++j) {
						auto v_j = verticies[j];
						auto jacobian_j = jacobians[j];
						ulong index_j = v_j->OrderingId();
						ulong dim_j = v_j->LocalDimension();

						assert(v_j->OrderingId() != -1);
						MatXX hessian = JtW * jacobian_j;

						// 所有的信息矩阵叠加起来
						H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
						if (j != i) {
							// 对称的下三角
							H.block(index_j, index_i, dim_j, dim_i).noalias() += hessian.transpose();
						}
					}
					b.segment(index_i, dim_i).noalias() -= jacobian_i.transpose() * edge.second->Residual();
				}
				ordering_J += edge.
			}
			Hessian_ = H;
			b_ = b;
			hsd_ = b;

			Jg = (J * hsd_).squaredNorm();

			alpha_ = hsd_.squaredNorm() / Jg.squaredNorm();
			t_hdl_cost_ += t_h.toc();

			hgn_ = VecX::Zero(size);  // initial hgn_x = 0_n;
		}

		void Problem::FindHdl(const double radius) {
			if (hgn_.norm() <= radius) {
				delta_x_ = hgn_;
				Hdl_choose_ = Hgn;
			}
			else if (hsd_.norm() >= radius){
				delta_x_ = hsd_ * (radius / hsd_.norm());
				Hdl_choose_ = Hsd;
			}
			else {
				double c = alpha_ * hsd_.transpose() * (hgn_ - alpha_ * hsd_);
				if (c < 0 || c == 0) {
					beta_ = (sqrt(c*c + (hgn_ - alpha_ * hsd_).squaredNorm() * ((radius*radius) - (alpha_ * hsd_).squaredNorm())) - c) / (hgn_ - alpha_ * hsd_).squaredNorm();
				}
				else {
					beta_ = (radius*radius - (alpha_ * hsd_).squaredNorm())/(sqrt(c*c + (hgn_ - alpha_ * hsd_).squaredNorm() * ((radius*radius) - (alpha_ * hsd_).squaredNorm())) + c);
				}
				delta_x_ = alpha_ * hsd_ + beta_ * (hgn_ - alpha * hsd_);
				Hdl_choose_ = Hgn_sd;
			}
		}

		bool Problem::IsGoodStepInDL(double& radius, bool& stop) {
			double scale = 0;

			// recompute residuals after update state
			double tempChi = 0.0;
			for (auto edge : edges_) {
				edge.second->ComputeResidual();
				tempChi += edge.second->RobustChi2();
			}
			if (err_prior_.size() > 0)
				tempChi += err_prior_.norm();
			tempChi *= 0.5;          // 1/2 * err^2

			if (Hdl_choose_ == Hgn) {
				scale = tempChi;
				scale += 1e-6;    // make sure it's non-zero :)
			}
			else if (Hdl_choose_ == Hsd) {
				scale = radius * (2 * alpha_*hsd_ - radius) / (2 * alpha_);
				scale += 1e-6;
			}
			else {
				scale = alpha_ * pow((1 - beta_), 2) * hsd_.squaredNorm() + beta_ * (2 - beta_)*tempChi;
				scale += 1e-6;
			}

			double rho = (currentChi_ - tempChi) / scale;
			if (rho > 0 && isfinite(tempChi))   // last step was good, 误差在下降
			{
				//退出条件1，最速梯度模足够小
				if (hsd_.norm() <= e1)
				{
					std::cout << "The algorithm convergence has been found with small hsd_" << std::endl;
					stop = true;
				}
				//退出条件2，步长足够小
				if (delta_x_.norm() <= e1)
				{
					std::cout << "The algorithm convergence has been found with small delta_x_" << std::endl;
					stop = true;
				}
				if (rho > 0.75) {
					radius = std::max(radius, 3.0 * delta_x_.norm());
				}
				else if (rho < 0.25) {
					radius /= 2.0;
					//退出条件3，阈值足够小
					if (radius < e3) {
						stop = true;
					}
				}
				currentChi_ = tempChi;
				return true;
			}
			else {
				radius /= 2.0;
				return false;
			}
		}

		void Problem::SolveDlLinearSystem() {

			//  TicToc t_Hmminv;
			// step1: schur marginalization --> Hpp, bpp
			int reserve_size = ordering_poses_;
			int marg_size = ordering_landmarks_;
			MatXX Hmm = Hessian_.block(reserve_size, reserve_size, marg_size, marg_size);
			MatXX Hpm = Hessian_.block(0, reserve_size, reserve_size, marg_size);
			MatXX Hmp = Hessian_.block(reserve_size, 0, marg_size, reserve_size);
			VecX bpp = b_.segment(0, reserve_size);
			VecX bmm = b_.segment(reserve_size, marg_size);

			MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
			// TODO:: use openMP
			for (auto landmarkVertex : idx_landmark_vertices_) {
				int idx = landmarkVertex.second->OrderingId() - reserve_size;
				int size = landmarkVertex.second->LocalDimension();
				Hmm_inv.block(idx, idx, size, size) = Hmm.block(idx, idx, size, size).inverse();
			}

			MatXX tempH = Hpm * Hmm_inv;
			H_pp_schur_ = Hessian_.block(0, 0, ordering_poses_, ordering_poses_) - tempH * Hmp;
			b_pp_schur_ = bpp - tempH * bmm;

			// step2: solve Hpp * hgn_x = bpp
			VecX hgn_pp(VecX::Zero(reserve_size));

			// TicToc t_linearsolver;
			hgn_pp = H_pp_schur_.ldlt().solve(b_pp_schur_);//  SVec.asDiagonal() * svd.matrixV() * Ub;    
			hgn_.head(reserve_size) = hgn_pp;
			// std::cout << " Linear Solver Time Cost: " << t_linearsolver.toc() << std::endl;

			// step3: solve Hmm * hgn_x = bmm - Hmp * hgn_x_pp;
			VecX hgn_x_ll(marg_size);
			hgn_ll = Hmm_inv * (bmm - Hmp * hgnpp);
			hgn_.tail(marg_size) = hgn_ll;

			//        std::cout << "schur time cost: "<< t_Hmminv.toc()<<std::endl;
		}

		*/
	}
}