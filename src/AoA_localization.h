#ifndef __AOA_LOCALIZATION_H
#define __AOA_LOCALIZATION_H
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

Vector3d AoA_localize(vector<Vector3d>& transmitters, vector<Vector3d>& AoA);

#endif 