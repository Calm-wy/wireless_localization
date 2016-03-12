#ifndef __WIRELESS_LOCALIZATION_H
#define __WIRELESS_LOCALIZATION_H
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

Vector3d wireless_localize(vector<Vector3d>& transmitters, vector<double>& rss, vector<Vector3d>& AoA);

#endif 