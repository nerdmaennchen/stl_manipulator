#include "sargparse/Parameter.h"
#include "sargparse/File.h"

#include "stl/STLParser.h"

#include "tetrahedron.h"

#include <iostream>

#include <armadillo>


void print_mass_properties();
sargp::Command cmd{"mass_properties", "show mass properties", print_mass_properties};

auto totalMass = cmd.Parameter<std::optional<double>>({}, "total_mass", "total mass of the object (used to calculate mass properties)");
auto density = cmd.Parameter<std::optional<double>>({}, "density", "density of the object (used to calculate mass properties)");

auto inFile = cmd.Parameter<sargp::File>("", "in", "the file to read from");

auto skew(arma::colvec3 const& v) {
	return arma::mat33{
		{0, -v(2), v(1)},
		{v(2), 0, -v(0)},
		{-v(1), v(0), 0},
	};
}

void print_mass_properties() {
	if (not inFile) {
		throw std::runtime_error("in has to be specified!");
	}
	kinematicTree::visual::stl::STLParser parser;
	auto mesh = parser.parse(*inFile);

	arma::mat33 inertia_tensor = arma::zeros(3, 3);
	double totalVolume{};
	arma::colvec3 com{};

	for (auto const& facet : mesh.getFacets()) {
		if (facet.mVertices.size() < 3) {
			continue;
		}
		auto const& base = facet.mVertices[0];
		for (auto i{1}; i < facet.mVertices.size()-1; ++i) {
			auto tetrahedron = Tetrahedron{{base, facet.mVertices[i], facet.mVertices[i+1]}};
			
			totalVolume += tetrahedron.volume;
			com += tetrahedron.com * tetrahedron.volume;
			inertia_tensor += tetrahedron.normed_inertia_tensor;
		}
    }

	com = com / totalVolume;

	std::cout << "volume: " << totalVolume << "\n";
	com.print("COM");
	std::cout << "\n\n";

	if (*density) {
		auto totMass = totalVolume * **density;
		std::cout << "properties by given density: (" << **density << ")\n\n";
		std::cout << "total mass: " << totMass << "\n";
		arma::mat33 I = inertia_tensor * **density;

		// move the inertia_tensor to the COM
		I += totMass * skew(com) * skew(com);

		(I * **density).print("inertia tensor");
	}

	if (*totalMass) {
		auto density = totalVolume / **totalMass;
		std::cout << "properties by given total mass: (" << **totalMass << ")\n\n";
		std::cout << "density: " << density << "\n";
		arma::mat33 I = inertia_tensor * density;

		// move the inertia_tensor to the COM
		I += **totalMass * skew(com) * skew(com);

		(I * density).print("inertia tensor");
	}
}


