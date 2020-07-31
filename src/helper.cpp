
// Standard includes
#include <helper.h>

namespace helper {

double deg2rad(const double &deg) {
	return deg*M_PI/180.0;
}

double rad2deg(const double &rad) {
	return rad*180.0/M_PI;
}



geometry_msgs::Point eigenvec2rospoint(const Eigen::Vector3d &vec) {
	geometry_msgs::Point Pt;
	Pt.x = vec[0]; Pt.y = vec[1]; Pt.z = vec[2];
	return Pt;
}

geometry_msgs::Point eigenvec2d2rospoint(const Eigen::Vector2d &vec) {
	geometry_msgs::Point Pt;
	Pt.x = vec[0]; Pt.y = vec[1]; Pt.z = 0.0;
	return Pt;
}

geometry_msgs::Point eigenvec2d2rospoint(const Eigen::Vector2d &vec,
	                                     const double height) {
	geometry_msgs::Point Pt;
	Pt.x = vec[0]; Pt.y = vec[1]; Pt.z = height;
	return Pt;
}

geometry_msgs::Vector3 setvector3(const double &x, const double &y, const double &z) {
	geometry_msgs::Vector3 vec3;
	vec3.x = x; vec3.y = y; vec3.z = z;
	return vec3;
}



mg_msgs::PVAJ_request get_empty_PVAJ() {
	mg_msgs::PVAJ_request PVAJ;
	PVAJ.use_pos = false;
	PVAJ.use_vel = false;
	PVAJ.use_acc = false;
	PVAJ.use_jerk = false;
	PVAJ.use_yaw = false;
	PVAJ.use_yaw_dot = false;
	return PVAJ;
}

mg_msgs::PVA_request get_empty_PVA() {
	mg_msgs::PVA_request PVA;
	PVA.use_pos = false;
	PVA.use_vel = false;
	PVA.use_acc = false;
	PVA.use_yaw = false;
	PVA.use_yaw_dot = false;
	return PVA;
}



}  // namespace helper

