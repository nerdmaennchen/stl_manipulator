#include <armadillo>
#include "sargparse/Parameter.h"

namespace 
{

arma::mat44 makeTransform(double dx, double dy, double dz, double alphaX, double alphaY, double alphaZ) {
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

arma::mat44 transform = arma::eye(4,4);

sargp::Parameter<double> rotX {1., "rotX", "how much to rotate around the x axis (degrees)", []{ transform = makeTransform(0, 0, 0, *rotX * M_PI/180., 0, 0) * transform; }};
sargp::Parameter<double> rotY {1., "rotY", "how much to rotate around the y axis (degrees)", []{ transform = makeTransform(0, 0, 0, 0, *rotY * M_PI/180., 0) * transform; }};
sargp::Parameter<double> rotZ {1., "rotZ", "how much to rotate around the z axis (degrees)", []{ transform = makeTransform(0, 0, 0, 0, 0, *rotZ * M_PI/180.) * transform; }};

sargp::Parameter<double> movX {1., "movX", "how much to move along the x axis (degrees)", []{ transform = makeTransform(*movX, 0, 0, 0, 0, 0) * transform; }};
sargp::Parameter<double> movY {1., "movY", "how much to move along the y axis (degrees)", []{ transform = makeTransform(0, *movY, 0, 0, 0, 0) * transform; }};
sargp::Parameter<double> movZ {1., "movZ", "how much to move along the z axis (degrees)", []{ transform = makeTransform(0, 0, *movZ, 0, 0, 0) * transform; }};

sargp::Parameter<double> scale {1., "scale", "scale the STL", []{ transform = arma::mat44{{*scale, 0, 0, 0}, {0, *scale, 0, 0}, {0, 0, *scale, 0}, {0, 0, 0, 1}} * transform;}};

sargp::Parameter<double> scaleX {1., "scaleX", "scale the x components of the STL", []{ transform = arma::mat44{{*scaleX, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} * transform;}};
sargp::Parameter<double> scaleY {1., "scaleY", "scale the y components of the STL", []{ transform = arma::mat44{{1, 0, 0, 0}, {0, *scaleY, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} * transform;}};
sargp::Parameter<double> scaleZ {1., "scaleZ", "scale the z components of the STL", []{ transform = arma::mat44{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, *scaleZ, 0}, {0, 0, 0, 1}} * transform;}};

}

arma::mat44 const& getTransform() {
    return transform;
}