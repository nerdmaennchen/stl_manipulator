#pragma once

#include <armadillo>
#include <array>

struct Tetrahedron {
    Tetrahedron(std::array<arma::colvec3, 3> const& vertices);

    double volume;
    arma::colvec3 com;
    arma::mat33 normed_inertia_tensor;
};
