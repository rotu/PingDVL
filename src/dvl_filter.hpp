#pragma once

#include <stdio.h>
#include <stdlib.h>

enum Observations_type
{
	O_vel_bow,				// velocity observed
	O_vel_starboard,
	O_vel_port,
	O_vel_stern,
	O_range_bow,				// range observed
	O_range_starboard,
	O_range_port,
	O_range_stern,
	O_imu_acc_y, //acceleration forward obvserved 	// acceleration observed
	O_imu_acc_x,
	O_imu_acc_z,
	O_imu_qi,
	O_imu_qj,
	O_imu_qk,
	O_imu_qr,
	O_delta_time,

	Mobs	// wants to be last and is not really a member
};

enum States_type
{
	S_delta_time,
	S_vel_x, // velocity state
	S_vel_y,
	S_vel_z,
	S_acc_x, // acceleration state
	S_acc_y,
	S_acc_z,
	S_imu_acc_offset_x, // the IMU has additive accleration error which is fairly steady in time.
	S_imu_acc_offset_y,
	S_imu_acc_offset_z,
	S_sounding_z, // depth of the ocean bed
	S_rot_qi, // rotation of the whole shebang
	S_rot_qj,
	S_rot_qk,
	S_rot_qr,

	Nsta	// wants to be last and is not really a member
};

// assumes Mobs and Nsta
#include "TinyEKF.h"

class DVLFilter : public TinyEKF {
public:
	DVLFilter();

	/// Set the variance for future measurements.
	/// r = 0 means the data is completely known
	/// r = s*s means the data has standard deviation s
	/// r = HUGE_VAL means the data is completely untrusted
	void set_mmt_variance(size_t i, double r);

protected:

    /**
     * Implement this function for your EKF model.
     * @param fx gets output of state-transition function <i>f(x<sub>0 .. n-1</sub>)</i>
     * @param F gets <i>n &times; n</i> Jacobian of <i>f(x)</i>
     * @param hx gets output of observation function <i>h(x<sub>0 .. n-1</sub>)</i>
     * @param H gets <i>m &times; n</i> Jacobian of <i>h(x)</i>
     */
    virtual void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) override;
};
