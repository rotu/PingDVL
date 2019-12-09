#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <cmath>

#include "dvl_filter.hpp"
#include "geometry.hpp"
#include <cassert>

struct Row {
	float delta_t;
	int imu_ok;
	char imu_cal;
	float roll;
	float pitch;
	float yaw;
	float acc_longitudinal;
	float acc_transverse;
	float acc_vertical;
	float dvl_ok;
	float vel_bow;
	float vel_starboard;
	float vel_port;
	float vel_stern;
	float range_bow;
	float range_starboard;
	float range_port;
	float range_stern;
	char status_bow;
	char status_starboard;
	char status_port;
	char status_stern;
};

const double RADIANS_PER_DEGREE = M_PI / 180.0;

int main(int argc, char ** argv){
	std::ifstream dvl_log("data/DVL-log.csv");
	std::string line;

	DVLFilter filter;
	std::getline(dvl_log, line); // ignore first line, which has headers

	while (std::getline(dvl_log, line)){
		Row row;

		std::sscanf(
				line.c_str(),
				"%f,"
				"%d,%c,"
				"%f,%f,%f,"
				"%f,%f,%f,"
				"%f,"
				"%f,%f,%f,%f,"
				"%f,%f,%f,%f,"
				"%c,%c,%c,%c",
				&row.delta_t,
				&row.imu_ok, &row.imu_cal,
				&row.roll, &row.pitch, &row.yaw,
				&row.acc_longitudinal, &row.acc_transverse, &row.acc_vertical,
				&row.dvl_ok,
				&row.vel_bow, &row.vel_starboard, &row.vel_port, &row.vel_stern,
				&row.range_bow, &row.range_starboard, &row.range_port, &row.range_stern,
				&row.status_bow, &row.status_starboard, &row.status_port, &row.status_stern
		);

		std::array<double,Mobs> obs;
		obs[O_vel_bow] = row.vel_bow;
		obs[O_vel_starboard] = row.vel_starboard;
		obs[O_vel_port] = row.vel_port;
		obs[O_vel_stern] = row.vel_stern;
		obs[O_range_bow] = row.range_bow;
		obs[O_range_starboard] = row.range_starboard;
		obs[O_range_port] = row.range_port;
		obs[O_range_stern] = row.range_stern;
		obs[O_delta_time] = row.delta_t;

		obs[O_vel_starboard] = row.vel_starboard;

		const double VARIANCE_UNTRUSTED_DATA = HUGE_VAL;
		const double VARIANCE_IMU_ROTATION = 1E-7;
		const double VARIANCE_IMU_ACCEL = 0.00014;
		const double VARIANCE_RANGE = 0.00039;
		const double VARIANCE_VEL = 0.0042;

		if (row.imu_ok != 0){
			for (auto i:{O_imu_qi,O_imu_qj,O_imu_qk, O_imu_qr,O_imu_acc_x,O_imu_acc_y,O_imu_acc_z}){
				filter.set_mmt_variance(i, VARIANCE_UNTRUSTED_DATA);
			}
		}
		else {
			auto imu_rotation = QuaternionRotation::from_ypr(
					row.yaw*RADIANS_PER_DEGREE,
					row.pitch*RADIANS_PER_DEGREE,
					row.roll*RADIANS_PER_DEGREE);
			obs[O_imu_qi] = imu_rotation.qi;
			obs[O_imu_qj] = imu_rotation.qj;
			obs[O_imu_qk] = imu_rotation.qk;
			obs[O_imu_qr] = imu_rotation.qr;
			obs[O_imu_acc_y] = row.acc_longitudinal;
			obs[O_imu_acc_x] = row.acc_transverse;
			obs[O_imu_acc_z] = row.acc_vertical;
			for (auto i:{O_imu_qi,O_imu_qj,O_imu_qk, O_imu_qr}){
				filter.set_mmt_variance(i, VARIANCE_IMU_ROTATION);
			}
			for (auto i:{O_imu_acc_x,O_imu_acc_y,O_imu_acc_z}){
				filter.set_mmt_variance(i, VARIANCE_IMU_ACCEL);
			}
		}

		filter.set_mmt_variance(O_range_bow, row.status_bow=='L' ? VARIANCE_RANGE : VARIANCE_UNTRUSTED_DATA);
		filter.set_mmt_variance(O_range_starboard, row.status_starboard== 'L' ? VARIANCE_RANGE : VARIANCE_UNTRUSTED_DATA);
		filter.set_mmt_variance(O_range_port, row.status_port== 'L' ? VARIANCE_RANGE : VARIANCE_UNTRUSTED_DATA);
		filter.set_mmt_variance(O_range_stern, row.status_stern== 'L' ? VARIANCE_RANGE : VARIANCE_UNTRUSTED_DATA);

		filter.set_mmt_variance(O_vel_bow, row.status_bow== 'L' ? VARIANCE_VEL : VARIANCE_UNTRUSTED_DATA);
		filter.set_mmt_variance(O_vel_starboard, row.status_starboard == 'L' ? VARIANCE_VEL : VARIANCE_UNTRUSTED_DATA);
		filter.set_mmt_variance(O_vel_port, row.status_port== 'L' ? VARIANCE_VEL : VARIANCE_UNTRUSTED_DATA);
		filter.set_mmt_variance(O_vel_stern, row.status_stern == 'L' ? VARIANCE_VEL : VARIANCE_UNTRUSTED_DATA);

		if (!filter.step(obs.data())){
			std::printf("oops.");
			assert(false);
		}

		std::printf("%f, %f, %f\n", filter.getX(S_vel_x), filter.getX(S_vel_y),filter.getX(S_vel_z));
		//std::printf("%f, %f, %f, %f, %f, %f\n", obs[O_imu_acc_x] ,filter.getX(S_acc_x), obs[O_imu_acc_y],filter.getX(S_acc_y), obs[O_imu_acc_z],filter.getX(S_acc_z));
		//std::printf("%f, %f, %f, %f, %f, %c, %c, %c, %c \n", row.range_bow, row.range_starboard, row.range_port, row.range_stern, filter.getX(S_sounding_z));
		//		for (size_t i=0; i<Nsta;i++){
		//			std::printf("%f,",filter.getX(i));
		//		}
		//		std::printf("\n");
	}
	dvl_log.close();
	return 0;
}
