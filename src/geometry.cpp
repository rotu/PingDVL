#pragma once
#include "geometry.hpp"
#include <cmath>
Vec3::Vec3(double x, double y, double z): coefs{x,y,z}{	};

double dot(const Vec3 &a, const Vec3 &b){
	double result = 0.0;
	for (size_t i=0;i<3;i++){
		result += a.coefs[i]*b.coefs[i];
	}
	return result;
};

QuaternionRotation::QuaternionRotation (double qi,double qj, double qk, double qr) : qi(qi),qj(qj),qk(qk),qr(qr){}

 QuaternionRotation QuaternionRotation:: from_ypr(double yaw, double pitch, double roll){
	    double cy = std::cos(yaw * 0.5);
	    double sy = std::sin(yaw * 0.5);
	    double cp = std::cos(pitch * 0.5);
	    double sp = std::sin(pitch * 0.5);
	    double cr = std::cos(roll * 0.5);
	    double sr = std::sin(roll * 0.5);

		return QuaternionRotation{
		    cy * cp * sr - sy * sp * cr,
			sy * cp * sr + cy * sp * cr,
			sy * cp * cr - cy * sp * sr,
			cy * cp * cr + sy * sp * sr,
		};
	}

QuaternionRotation QuaternionRotation::conjugate() const{
		return QuaternionRotation{-qi,-qj,-qk,+qr};
	}

	Vec3 QuaternionRotation::rotate(const Vec3 v) const {
		double s = std::sqrt(qi*qi+qj*qj+qk*qk+qr*qr);
		double x = v.coefs[0];
		double y = v.coefs[1];
		double z = v.coefs[2];
		Vec3 result {
			(1-2*s*(qj*qj+qk*qk))*x + 2*s*(qi*qj-qk*qr)*y + 2*s*(qi*qk+qj*qr)*z,
			2*s*(qi*qj + qk *qr) *x + (1-2*s*(qi*qi+qk*qk))*y + (2*s*(qj*qk-qi*qr))*z,
			2*s*(qi*qk-qj*qr) * x + 2*s*(qj*qk + qi*qr) * y + (1-2*s*(qi*qi+qj*qj))*z
		};
		return result;
	}
