#include "wireless_localization.h"
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


struct  wireless_Residual
{
	wireless_Residual(Vector3d AoA, double rss, Vector3d p) : AoA_(AoA), rss_(rss), p_(p) {}

	template <typename T> bool operator()(const T* const pos,
                                        T* residual) const 
	{
	    Matrix<T, 3, 1> dir = Matrix<T, 3, 1>( T(pos[0]), T(pos[1]), T(pos[2]) ) - Matrix<T, 3, 1>( T(p_(0)), T(p_(1)), T(p_(2)) );
	    Matrix<T, 3, 1> AoA = Matrix<T, 3, 1>(T(AoA_(0)), T(AoA_(1)), T(AoA_(2)) );
	    Matrix<T, 3, 3> skew;
    	skew << T(0), -AoA(2), AoA(1),
		 AoA(2), T(0),  -AoA(0),
		 -AoA(1), AoA(0), T(0);
	    Matrix<T, 3, 1> err =  skew*dir;
	    residual[0] = err(0);
	    residual[1] = err(1);
	    residual[2] = err(2);

	    T sum_ = (pos[0] - T(p_(0)) )*(pos[0] - T(p_(0)) ) + (pos[1] - T(p_(1)) )*(pos[1] - T(p_(1)) )  + (pos[2] - T(p_(2)) )*(pos[2] - T(p_(2))  ) ;
	    residual[3] = T(rss_) - sqrt(sum_);
	    return true;

	}
  private:
  	const Vector3d AoA_;
  	const double rss_;
  	const Vector3d p_;
};

//3D position is estimated
Vector3d wireless_localize(vector<Vector3d>& transmitters, vector<double>& rss, vector<Vector3d>& AoA)
{
	assert(transmitters.size() == AoA.size() && transmitters.size() == rss.size() );
	// double targetPos[3] = {0.0, 0.0, 0.0};
	Vector3d targetPos = Vector3d::Zero();
	Problem problem;
	for (int i = 0; i < AoA.size(); ++i) 
	{
		CostFunction* cost_function =
		    new AutoDiffCostFunction<wireless_Residual, 4, 3>(
		        new wireless_Residual(
		        	AoA[i], 
		        	rss[i],
		        	transmitters[i] ));

		problem.AddResidualBlock(cost_function, NULL, &targetPos(0));
	}

	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.max_num_iterations = 100;
	options.minimizer_progress_to_stdout = true;

	Solver::Summary summary;
	Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << "\n";

	return targetPos;
	// Vector3d p(targetPos[0], targetPos[1], 0); 
	// return p;
}


