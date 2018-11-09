#include "QuaternionHelper.h"

namespace kinematicTree {
namespace utils {

Quaternion quatMul(Quaternion const& q1, Quaternion const& q2)
{
	Quaternion ret;
	ret(0) = ((q1(0) * q2(0)) - (q1(1) * q2(1)) - (q1(2) * q2(2)) - (q1(3) * q2(3)));
	ret(1) = ((q1(0) * q2(1)) + (q1(1) * q2(0)) + (q1(2) * q2(3)) - (q1(3) * q2(2)));
	ret(2) = ((q1(0) * q2(2)) - (q1(1) * q2(3)) + (q1(2) * q2(0)) + (q1(3) * q2(1)));
	ret(3) = ((q1(0) * q2(3)) + (q1(1) * q2(2)) - (q1(2) * q2(1)) + (q1(3) * q2(0)));
	return ret;
}


Quaternion conjungate(Quaternion const& q)
{
	Quaternion ret = q;
	ret.rows(1, 3) = - ret.rows(1, 3);
	return ret;
}

Quaternion quaternionFromMatrix(arma::mat33 const& mat)
{
	Quaternion ret;
	double trace = mat(0, 0) + mat(1, 1) + mat(2, 2);
	if (trace > 0) {
		float s = sqrt(trace+1.0) * 2.;
		ret(0) = 0.25 * s;
		ret(1) = (mat(2, 1) - mat(1, 2)) / s;
		ret(2) = (mat(0, 2) - mat(2, 0)) / s;
		ret(3) = (mat(1, 0) - mat(0, 1)) / s;
	} else if (mat(0, 0) > mat(1, 1) && mat(0, 0) > mat(2, 2)) {
		double s = std::sqrt(1 + mat(0, 0) - mat(1, 1) - mat(2, 2)) * 2.;
		ret(0) = (mat(2, 1) - mat(1, 2)) / s;
		ret(1) = 0.25 * s;
		ret(2) = (mat(0, 1) + mat(1, 0)) / s;
		ret(3) = (mat(0, 2) + mat(2, 0)) / s;
	} else if (mat(1, 1) > mat(2, 2)) {
		double s = std::sqrt(1 + mat(1, 1) - mat(0, 0) - mat(2, 2)) * 2.;
		ret(0) = (mat(0, 2) - mat(2, 0)) / s;
		ret(1) = (mat(0, 1) + mat(1, 0)) / s;
		ret(2) = 0.25 * s;
		ret(3) = (mat(1, 2) + mat(2, 1)) / s;
	} else {
		double s = std::sqrt(1 + mat(2, 2) - mat(0, 0) - mat(1, 1)) * 2.;
		ret(0) = (mat(1, 0) - mat(0, 1)) / s;
		ret(1) = (mat(0, 2) + mat(2, 0)) / s;
		ret(2) = (mat(1, 2) + mat(2, 1)) / s;
		ret(3) = 0.25 * s;
	}
	return ret;
}

arma::mat33 quaternionToMatrix(Quaternion const& q)
{
	arma::mat33 mat;
	mat(0, 0) = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
	mat(1, 1) = 1 - 2 * (q(1) * q(1) + q(3) * q(3));
	mat(2, 2) = 1 - 2 * (q(1) * q(1) + q(2) * q(2));

	mat(0, 1) = 2 * (q(1) * q(2) - q(0) * q(3));
	mat(0, 2) = 2 * (q(1) * q(3) + q(0) * q(2));
	mat(1, 2) = 2 * (q(2) * q(3) - q(0) * q(1));

	mat(1, 0) = 2 * (q(1) * q(2) + q(0) * q(3));
	mat(2, 0) = 2 * (q(1) * q(3) - q(0) * q(2));
	mat(2, 1) = 2 * (q(2) * q(3) + q(0) * q(1));
	return mat;
}


Quaternion quaternionBetweenVectors(arma::colvec3 const& vec1, arma::colvec3 const& vec2)
{
	arma::colvec vec1_n = vec1 * 1. / arma::norm(vec1, 2);
	arma::colvec vec2_n = vec2 * 1. / arma::norm(vec2, 2);
	Quaternion ret;

	ret(0) = 1 + arma::dot(vec1_n, vec2_n);
	ret.rows(1, 3) = arma::cross(vec1_n, vec2_n);

	const double retNorm = arma::norm(ret, 2);
	if (retNorm < .000001) {
		ret.zeros();
		ret(0) = 1;
	}
	normalizeQuaternion(ret);

	return ret;
}


Quaternion slerp(Quaternion const& q1, Quaternion const& q2, double t)
{
	double dotProd = arma::dot(q1, q2);
	dotProd = std::max(-1., std::min(1., dotProd));

	const double theta = std::abs(std::acos(dotProd));

	if (theta < .0001) { // if the quaternions are to parallel return one of the inputs
		return q2;
	}

	const double sinTheta = std::sin(theta);

	Quaternion ret = std::sin((1 - t) * theta) / sinTheta * q1 + std::sin(t * theta) / sinTheta * q2;
	normalizeQuaternion(ret);
	return ret;
}

arma::colvec3 rodriguezFromQuaternion(Quaternion const& q)
{
	return arma::asin(q.rows(1, 3));
}

} /* namespace utils */
} /* namespace kinematicTree */
