#include "AoA_localization.h"
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


struct  AoA_Residual
{
	AoA_Residual(Vector3d AoA, Vector3d p) : AoA_(AoA), p_(p) {}

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
	    return true;

	}
  private:
  	const Vector3d AoA_;
  	const Vector3d p_;
};

//3D position is estimated
Vector3d AoA_localize(vector<Vector3d>& transmitters, vector<Vector3d>& AoA)
{
	assert(transmitters.size() == AoA.size());
	double targetPos[2] = {0.0, 0.0};

	Problem problem;
	for (int i = 0; i < AoA.size(); ++i) 
	{
		CostFunction* cost_function =
		    new AutoDiffCostFunction<AoA_Residual, 3, 3>(
		        new AoA_Residual(
		        	AoA[i], 
		        	transmitters[i] ));

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


