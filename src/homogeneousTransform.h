#pragma once

#include <armadillo>

namespace kinematicTree {
	namespace utils {

	arma::mat44 getTransform(double dx, double dy, double dz, double alphaX, double alphaY, double alphaZ);

	arma::mat44 invertHomogeneous(arma::mat44 const& _mat);

	arma::mat44 interpolateTransform(arma::mat44 const& t1, arma::mat44 const& t2, double t);

	}
}

