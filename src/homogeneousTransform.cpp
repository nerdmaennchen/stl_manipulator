#include "homogeneousTransform.h"
#include "QuaternionHelper.h"

namespace kinematicTree {
namespace utils {

arma::mat44 getTransform(double dx, double dy, double dz, double alphaX, double alphaY, double alphaZ) {
	arma::mat44 translation = arma::eye(4, 4);
	translation(0, 3) = dx;
	translation(1, 3) = dy;
	translation(2, 3) = dz;

	/* align z axis to current joint this might incorporate two rotations (first the rotation around x-axis is performed, then around the y axis and around the z axis) */
	arma::mat44 rotX = arma::eye(4, 4);
	arma::mat44 rotY = arma::eye(4, 4);
	arma::mat44 rotZ = arma::eye(4, 4);

	if (std::abs(alphaX) >= std::numeric_limits<double>::epsilon())
	{
		const double cRotX = std::cos(alphaX);
		const double sRotX = std::sin(alphaX);
		rotX(1, 1) = cRotX;
		rotX(1, 2) = -sRotX;
		rotX(2, 1) = sRotX;
		rotX(2, 2) = cRotX;
	}

	if (std::abs(alphaY) >= std::numeric_limits<double>::epsilon())
	{
		const double cRotY = std::cos(alphaY);
		const double sRotY = std::sin(alphaY);
		rotY(0, 0) = cRotY;
		rotY(0, 2) = sRotY;
		rotY(2, 0) = -sRotY;
		rotY(2, 2) = cRotY;
	}

	if (std::abs(alphaZ) >= std::numeric_limits<double>::epsilon())
	{
		const double cRotZ = std::cos(alphaZ);
		const double sRotZ = std::sin(alphaZ);
		rotZ(0, 0) = cRotZ;
		rotZ(0, 1) = -sRotZ;
		rotZ(1, 0) = sRotZ;
		rotZ(1, 1) = cRotZ;
	}

	return translation * rotX * rotY * rotZ;
}

arma::mat44 invertHomogeneous(arma::mat44 const& _mat) {
	arma::mat44 ret = arma::eye(4, 4);
	ret.submat(0, 0, 2, 2) = _mat.submat(0, 0, 2, 2).t();
	ret.col(3).rows(0, 2) = -ret.submat(0, 0, 2, 2) * _mat.col(3).rows(0, 2);
	return ret;
}


arma::mat44 interpolateTransform(arma::mat44 const& t1, arma::mat44 const& t2, double t)
{
	arma::mat44 interpolatedTransform = arma::eye(4, 4);

	Quaternion q1 = quaternionFromMatrix(t1.submat(0, 0, 2, 2));
	Quaternion q2 = quaternionFromMatrix(t2.submat(0, 0, 2, 2));
	Quaternion qt = slerp(q1, q2, t);

	interpolatedTransform.submat(0, 0, 2, 2) = quaternionToMatrix(qt);
	interpolatedTransform.col(3).rows(0, 2) = (1. - t) * t1.col(3).rows(0, 2) + t * t2.col(3).rows(0, 2);

	return interpolatedTransform;
}

}
}
