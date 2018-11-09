#pragma once

#include <armadillo>

namespace kinematicTree {
namespace utils {

// format: w x y z
using Quaternion = arma::colvec4;

inline void normalizeQuaternion(Quaternion &quat) {
	quat *= 1. / arma::norm(quat, 2);
}

Quaternion quatMul(Quaternion const& q1, Quaternion const& q2);
Quaternion conjungate(Quaternion const& q);

Quaternion quaternionFromMatrix(arma::mat33 const& mat);
arma::mat33 quaternionToMatrix(Quaternion const& quat);

Quaternion quaternionBetweenVectors(arma::colvec3 const& vec1, arma::colvec3 const& vec2);
Quaternion slerp(Quaternion const& q1, Quaternion const& q2, double t);

arma::colvec3 rodriguezFromQuaternion(Quaternion const& q);

} /* namespace utils */
} /* namespace kinematicTree */
