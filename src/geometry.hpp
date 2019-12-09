#pragma once
#include <array>

struct Vec3 {
	std::array<double,3> coefs;
	Vec3(double x, double y, double z);
};

double dot(const Vec3 &a, const Vec3 &b);

struct QuaternionRotation{
	double qi;
	double qj;
	double qk;
	double qr;

	QuaternionRotation (double qi, double qj, double qk, double qr);

	static QuaternionRotation from_ypr(double yaw, double pitch, double roll);

	QuaternionRotation conjugate() const;
	Vec3 rotate(const Vec3 v) const;
};
