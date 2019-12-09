#include "dvl_filter.hpp"

#include <stddef.h>
#include <array>
#include <cmath>
#include "geometry.hpp"

#define COS_65_DEG cos(65.0*M_PI/180.0)
#define SIN_65_DEG sin(65.0*M_PI/180.0)

// direction of each sonar transponder when we are at zero rotation
const struct HardwareStatics {
  Vec3 dir_bow;
  Vec3 dir_starboard;
  Vec3 dir_stern;
  Vec3 dir_port;
} HARDWARE_STATICS = {
    /*.dir_bow = */ {COS_65_DEG, 0, SIN_65_DEG},
	/*.dir_starboard = */ {0, COS_65_DEG, SIN_65_DEG},
	/*.dir_stern = */ {-COS_65_DEG, 0, SIN_65_DEG},
	/*.dir_starboard = */ {0, -COS_65_DEG, SIN_65_DEG},
};

DVLFilter::DVLFilter(): TinyEKF(){
	// set process noise covariances
	// todo: measure these values empirically.
	for (size_t i=0; i<Nsta; i++){
		this->setQ(i, i, .0001);
	}
	this->setQ(S_sounding_z, S_sounding_z, 0.001);

	// determined empirically to be 0.000014 for a timestep of 0.2,
	// compensated by scaling with the square root of the time step.
	this->setQ(S_imu_acc_offset_x, S_imu_acc_offset_x, 0.000007);
	this->setQ(S_imu_acc_offset_y, S_imu_acc_offset_y, 0.000007);
	this->setQ(S_imu_acc_offset_y, S_imu_acc_offset_z, 0.000007);

    // Set measurement noise covariances
	// todo: measure these values empirically.
	for (size_t i=0; i<Mobs; i++){
		this->setR(i, i, .0001);
	}
	this->setR(O_delta_time,O_delta_time,0);
}

void DVLFilter::set_mmt_variance(size_t i, double r){
	setR(i,i,r);
}

void DVLFilter::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
{
	// Here we need to create
	// ekf->fx 	Nx1		/* output of user defined f() state-transition function */
	// ekf->F 	NxN 		/* Jacobian of process model */	// Zeroed by elf_init()
	// ekf->hx 	Mx1		/* output of user defined h() measurement function */
	// ekf->H  	MxN		/* Jacobian of measurement model */	// Zeroed by elf_init()

	// carry forward the state in time - it's mostly the same
	for (size_t i=0;i<Nsta; i++)
		fx[i] = getX(i);

	fx[S_vel_x] += getX(S_acc_x) * getX(S_delta_time);
	fx[S_vel_y] += getX(S_acc_y) * getX(S_delta_time);
	fx[S_vel_z] += getX(S_acc_z) * getX(S_delta_time);
	fx[S_sounding_z] += getX(S_vel_z) * (- getX(S_delta_time));

	// obvious part of the model - the derivative of each state with respect to itself is 1
	for (size_t i=0;i<Nsta; i++)
		F[i][i] = 1.0;

	// the model is all linear, so we just need the product rule to compute its Jacobian
	// d(a*b)/d(a) = b
	// d(a*b)/d(b) = a
	F[S_vel_x][S_acc_x] = getX(S_delta_time);
	F[S_vel_y][S_acc_y] = getX(S_delta_time);
	F[S_vel_z][S_acc_z] = getX(S_delta_time);
	F[S_sounding_z][S_vel_z] = -getX(S_delta_time);
	// F[S_*][S_delta_time] is not actually zero but we're not modeling time.
	// leaving it zero just means we don't propagate error in other state estimates to the time estimate.

	// things get fancier for the measurement function. It's still mostly linear, though.
	Vec3 s_vel = {getX(S_vel_x), getX(S_vel_y), getX(S_vel_z)};
	Vec3 VEC_Z{0,0,1.0};

	QuaternionRotation quat{getX(S_rot_qi), getX(S_rot_qj), getX(S_rot_qk), getX(S_rot_qr)};
	// adjust the direction of all the sonar transponders for current rotation
	auto dir_bow = quat.rotate(HARDWARE_STATICS.dir_bow);
	auto dir_starboard = quat.rotate(HARDWARE_STATICS.dir_starboard);
	auto dir_port = quat.rotate(HARDWARE_STATICS.dir_port);
	auto dir_stern = quat.rotate(HARDWARE_STATICS.dir_stern);

	hx[O_vel_bow] = dot(s_vel, dir_bow);
	hx[O_vel_starboard] = dot(s_vel, dir_starboard);
	hx[O_vel_port] = dot(s_vel, dir_port);
	hx[O_vel_stern] = dot(s_vel, dir_stern);

	// assume the bottom is flat so the range is the hypotenuse of a triangle where the altitude is the height
	// It isn't actually, but we treat that as measurement noise.
	hx[O_range_bow] = getX(S_sounding_z) / dot(VEC_Z, dir_bow);
	hx[O_range_port] =  getX(S_sounding_z) / dot(VEC_Z, dir_port);
	hx[O_range_starboard] = getX(S_sounding_z) / dot(VEC_Z, dir_starboard);
	hx[O_range_stern] =  getX(S_sounding_z) / dot(VEC_Z, dir_stern);

	hx[O_imu_acc_x] = getX(S_acc_x) + getX(S_imu_acc_offset_x);
	hx[O_imu_acc_y] = getX(S_acc_y) + getX(S_imu_acc_offset_y);
	hx[O_imu_acc_z] = getX(S_acc_z) + getX(S_imu_acc_offset_z);

	// directly observable state
	hx[O_delta_time] = getX(S_delta_time);
	hx[O_imu_qi] = getX(S_rot_qi);
	hx[O_imu_qj] = getX(S_rot_qj);
	hx[O_imu_qk] = getX(S_rot_qk);
	hx[O_imu_qr] = getX(S_rot_qr);

	H[O_vel_bow][S_vel_x] = dir_bow.coefs[0];
	H[O_vel_bow][S_vel_y] = dir_bow.coefs[1];
	H[O_vel_bow][S_vel_z] = dir_bow.coefs[2];
	H[O_vel_port][S_vel_x] = dir_port.coefs[0];
	H[O_vel_port][S_vel_y] = dir_port.coefs[1];
	H[O_vel_port][S_vel_z] = dir_port.coefs[2];
	H[O_vel_starboard][S_vel_x] = dir_starboard.coefs[0];
	H[O_vel_starboard][S_vel_y] = dir_starboard.coefs[1];
	H[O_vel_starboard][S_vel_z] = dir_starboard.coefs[2];
	H[O_vel_stern][S_vel_x] = dir_stern.coefs[0];
	H[O_vel_stern][S_vel_y] = dir_stern.coefs[1];
	H[O_vel_stern][S_vel_z] = dir_stern.coefs[2];
	// H[O_vel_*][S_rot_q*] is not actually zero. We're pretending it is since we are not modeling IMU rotation;
	// just assuming that it is errorless

	H[O_range_bow][S_sounding_z] = 1 / dot(VEC_Z, dir_bow);
	H[O_range_port][S_sounding_z] = 1 / dot(VEC_Z, dir_port);
	H[O_range_starboard][S_sounding_z] = 1 / dot(VEC_Z, dir_starboard);
	H[O_range_stern][S_sounding_z] = 1 / dot(VEC_Z, dir_stern);

	H[O_imu_acc_x][S_acc_x] = 1.0;
	H[O_imu_acc_y][S_acc_y] = 1.0;
	H[O_imu_acc_z][S_acc_z] = 1.0;
	H[O_imu_acc_x][S_imu_acc_offset_x] = 1.0;
	H[O_imu_acc_y][S_imu_acc_offset_y] = 1.0;
	H[O_imu_acc_z][S_imu_acc_offset_z] = 1.0;
	// directly observable state
	H[O_delta_time][S_delta_time] = 1.0;
	H[O_imu_qi][S_rot_qi] = 1.0;
	H[O_imu_qj][S_rot_qj] = 1.0;
	H[O_imu_qk][S_rot_qk] = 1.0;
	H[O_imu_qr][S_rot_qr] = 1.0;
}
