#include "parameter/Parameter.h"
#include "parameter/ArgumentParsing.h"

#include "homogeneousTransform.h"
#include "stl/STLParser.h"

#include <iostream>

namespace {

parameter::Parameter<std::optional<std::string>> printHelp{{}, "help", "print this help add a string which will be used in a grep-like search through the parameters"};


arma::mat44 transform = arma::eye(4,4);

parameter::Parameter<double> rotX {1., "rotX", "how much to rotate around the x axis (degrees)", []{ transform = kinematicTree::utils::getTransform(0, 0, 0, rotX * M_PI/180., 0, 0) * transform; }};
parameter::Parameter<double> rotY {1., "rotY", "how much to rotate around the y axis (degrees)", []{ transform = kinematicTree::utils::getTransform(0, 0, 0, 0, rotY * M_PI/180., 0) * transform; }};
parameter::Parameter<double> rotZ {1., "rotZ", "how much to rotate around the z axis (degrees)", []{ transform = kinematicTree::utils::getTransform(0, 0, 0, 0, 0, rotZ * M_PI/180.) * transform; }};

parameter::Parameter<double> movX {1., "movX", "how much to move along the x axis (degrees)", []{ transform = kinematicTree::utils::getTransform(movX, 0, 0, 0, 0, 0) * transform; }};
parameter::Parameter<double> movY {1., "movY", "how much to move along the y axis (degrees)", []{ transform = kinematicTree::utils::getTransform(0, movY, 0, 0, 0, 0) * transform; }};
parameter::Parameter<double> movZ {1., "movZ", "how much to move along the z axis (degrees)", []{ transform = kinematicTree::utils::getTransform(0, 0, movZ, 0, 0, 0) * transform; }};

parameter::Parameter<double> scale {1., "scale", "scale the STL", []{ transform = arma::mat44{{scale, 0, 0, 0}, {0, scale, 0, 0}, {0, 0, scale, 0}, {0, 0, 0, 1}} * transform;}};

parameter::Parameter<double> scaleX {1., "scaleX", "scale the x components of the STL", []{ transform = arma::mat44{{scaleX, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} * transform;}};
parameter::Parameter<double> scaleY {1., "scaleY", "scale the y components of the STL", []{ transform = arma::mat44{{1, 0, 0, 0}, {0, scaleY, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} * transform;}};
parameter::Parameter<double> scaleZ {1., "scaleZ", "scale the z components of the STL", []{ transform = arma::mat44{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, scaleZ, 0}, {0, 0, 0, 1}} * transform;}};

parameter::Parameter<std::string> inFile  {"", "in", "the file to read from"};
parameter::Parameter<std::string> outFile {"", "out", "the file to write to"};

parameter::Flag invertVertexFanning {"invert_vertex_fanning", "reorder the vertices of all facets"};

}

int main(int argc, char** argv)
{
	if (std::string(argv[argc-1]) == "--bash_completion") {
		auto hints = parameter::getNextArgHint(argc-2, argv+1);
		for (auto const& hint : hints) {
			std::cout << hint << " ";
		}
		return 0;
	}
	// pass everything except the name of the application
	parameter::parseArguments(argc-1, argv+1);

	if (printHelp.isSpecified()) {
		std::cout << parameter::generateHelpString(std::regex{".*" + printHelp.get().value_or("") + ".*"}) << std::endl;
		return 0;
	}

	if (not inFile.isSpecified()) {
		throw std::runtime_error("in has to be specified!");
	}
	if (not outFile.isSpecified()) {
		throw std::runtime_error("out has to be specified!");
	}

	kinematicTree::visual::stl::STLParser parser;
	auto mesh = parser.parse(inFile);
	std::cout << transform << std::endl;
	mesh.applyTransform(transform);

	if (invertVertexFanning) {
		mesh.invertVertexFanning();
	}

	parser.dump(mesh, outFile);
	std::cout << "done" << std::endl;

	return 0;
}
