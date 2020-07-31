
#ifndef HELPER_H_
#define HELPER_H_

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include "mg_msgs/PVAJ_request.h"
#include "mg_msgs/PVA_request.h"
#include "mg_msgs/PVAJS_array.h"
#include "mg_msgs/PVAJS.h"

#include <cmath>

namespace helper {

double deg2rad(const double &deg);

double rad2deg(const double &rad);



geometry_msgs::Point eigenvec2rospoint(const Eigen::Vector3d &vec);

geometry_msgs::Point eigenvec2d2rospoint(const Eigen::Vector2d &vec);

geometry_msgs::Point eigenvec2d2rospoint(const Eigen::Vector2d &vec, const double height);

geometry_msgs::Vector3 setvector3(const double &x, const double &y, const double &z);


mg_msgs::PVAJ_request get_empty_PVAJ();

mg_msgs::PVA_request get_empty_PVA();



}  // namespace helper

#endif  // MISSION_HELPER_H_
