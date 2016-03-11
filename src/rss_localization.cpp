#include "rss_localization.h"
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <vector>

#include <ceres/ceres.h>


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::CauchyLoss;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;


using namespace std;
using namespace Eigen;


struct  TOA_Residual
{
	TOA_Residual(double dist, double x, double y) : dist_(dist), x_(x), y_(y) {}

	template <typename T> bool operator()(const T* const pos,
                                        T* residual) const 
	{
	   	//T sum = (pos[0] - T(x_) )*(pos[0] - T(x_) ) + (pos[1] - T(y_) )*(pos[1] - T(y_) ) + (pos[2] - T(z_) )*(pos[2] - T(z_) );
		T sum = (pos[0] - T(x_) )*(pos[0] - T(x_) ) + (pos[1] - T(y_) )*(pos[1] - T(y_)  ) ;
	    residual[0] = T(dist_) - sqrt(sum);
	    return true;

	}
  private:
  	const double dist_;
  	const double x_;
  	const double y_;
};

//2D position is estimated
Vector3d rss_localize(vector<Vector3d>& transmitters, vector<double>& rss)
{
	assert(transmitters.size() == rss.size());
	double targetPos[2] = {0.0, 0.0};

	Problem problem;
	for (int i = 0; i < rss.size(); ++i) 
	{
		CostFunction* cost_function =
		    new AutoDiffCostFunction<TOA_Residual, 1, 2>(
		        new TOA_Residual(
		        	rss[i], 
		        	transmitters[i](0), 
		        	transmitters[i](1) ));

		problem.AddResidualBlock(cost_function, NULL, targetPos);
	}

	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.max_num_iterations = 100;
	options.minimizer_progress_to_stdout = true;

	Solver::Summary summary;
	Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";

	Vector3d p(targetPos[0], targetPos[1], 0); 
	return p;
}


