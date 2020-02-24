#include <iostream>

#include "backend/vertex.h"
#include "backend/edge.h"

using namespace std;

namespace vio {
	namespace backend {
		unsigned long global_edge_id = 0;

		Edge::Edge(int residual_dimension, int num_verticies,
			const vector<string> &verticies_types) {
			residual_resize(residual_dimesion, 1);
			
			if (!verticies_types.empty())
				verticies_types_ = verticies_types;
			
			jacobians_.resize(num_verticies);
			id_ = global_edge_id++;

			Eigen::MatrixXd information(residual_dimension, residual_dimension);
			information.setIdentity();
			information_ = information;
			
			lossfunction_ = nullptr;

			//cout<<:Edge e_dim="<<redidual_dimension<<",num_verticies="<<num_verticies<<",id="<<id_<<endl;
		}

		Edge::~Edge() {}

		double Edge::Chi2() const {
			//计算residual时已经乘了sqrt_information,就不再用residual*information*residual了
			return residual_.transpose() * residual_;
		}

		double Edge::RobustChi2() const {
			double residual2 = this->Chi2;
			if (lossfunction_)
			{
				Vec3 rho;
				lossfunction_->Compute(residual2, rho);
				residual2 = rho[0];
			}
			return residual2;
		}

		//注意加了核函数之后，信息矩阵变化怎么改变,drho为一阶导数
		void Rdge::RobustInfo(double &drho, MatXX &info) const {
			if (lossfunction_) {
				double residual2 = this->Chi2();
				Vec3 rho;
				lossfunction_->Compute(residual2, rho);
				VecX weight_err = sqrt_information_ * residual_;

				MatXX robust_info(information_.rows(), information_.cols());
				robust_info.setIdentity();
				robust_info *= rho[1];
				if (rho[1] + 2 * rho[2] * e2 > 0.)
				{
					robust_info += 2 * rho[2] * weight_err * weight_err.transpose();
				}

				info = robust_info * information_;
				drho = rho[1];
			}
			else
			{
				drho = 1.0;
				info = information_;
			}
		}

		bool Edge::CheckTypeValid() {
			if (!verticies_types_.empty()) {
				// check type info
				for (size_t i = 0; i < verticies_.size(); ++i) {
					if (verticies_types_[i] != verticies_[i]->TypeInfo()) {
						cout << "Vertex type does not match, should be " << verticies_types_[i] <<
							", but set to " << verticies_[i]->TypeInfo() << endl;
						return false;
					}
				}
			}
			//用glog调试
			return true;
		}
	}
}