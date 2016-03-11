#ifndef __RSS_LOCALIZATION_H
#define __RSS_LOCALIZATION_H
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

Vector3d rss_localize(vector<Vector3d>& transmitters, vector<double>& rss);

#endif 