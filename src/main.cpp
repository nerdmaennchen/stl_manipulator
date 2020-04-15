#include "sargparse/Parameter.h"
#include "sargparse/ArgumentParsing.h"

#include "homogeneousTransform.h"
#include "stl/STLParser.h"

#include <iostream>

namespace {

sargp::Parameter<std::optional<std::string>> printHelp{{}, "help", "print this help add a string which will be used in a grep-like search through the parameters"};


arma::mat44 transform = arma::eye(4,4);

sargp::Parameter<double> rotX {1., "rotX", "how much to rotate around the x axis (degrees)", []{ transform = kinematicTree::utils::getTransform(0, 0, 0, *rotX * M_PI/180., 0, 0) * transform; }};
sargp::Parameter<double> rotY {1., "rotY", "how much to rotate around the y axis (degrees)", []{ transform = kinematicTree::utils::getTransform(0, 0, 0, 0, *rotY * M_PI/180., 0) * transform; }};
sargp::Parameter<double> rotZ {1., "rotZ", "how much to rotate around the z axis (degrees)", []{ transform = kinematicTree::utils::getTransform(0, 0, 0, 0, 0, *rotZ * M_PI/180.) * transform; }};

sargp::Parameter<double> movX {1., "movX", "how much to move along the x axis (degrees)", []{ transform = kinematicTree::utils::getTransform(*movX, 0, 0, 0, 0, 0) * transform; }};
sargp::Parameter<double> movY {1., "movY", "how much to move along the y axis (degrees)", []{ transform = kinematicTree::utils::getTransform(0, *movY, 0, 0, 0, 0) * transform; }};
sargp::Parameter<double> movZ {1., "movZ", "how much to move along the z axis (degrees)", []{ transform = kinematicTree::utils::getTransform(0, 0, *movZ, 0, 0, 0) * transform; }};

sargp::Parameter<double> scale {1., "scale", "scale the STL", []{ transform = arma::mat44{{*scale, 0, 0, 0}, {0, *scale, 0, 0}, {0, 0, *scale, 0}, {0, 0, 0, 1}} * transform;}};

sargp::Parameter<double> scaleX {1., "scaleX", "scale the x components of the STL", []{ transform = arma::mat44{{*scaleX, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} * transform;}};
sargp::Parameter<double> scaleY {1., "scaleY", "scale the y components of the STL", []{ transform = arma::mat44{{1, 0, 0, 0}, {0, *scaleY, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} * transform;}};
sargp::Parameter<double> scaleZ {1., "scaleZ", "scale the z components of the STL", []{ transform = arma::mat44{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, *scaleZ, 0}, {0, 0, 0, 1}} * transform;}};

sargp::Parameter<std::string> inFile  {"", "in", "the file to read from"};
sargp::Parameter<std::string> outFile {"", "out", "the file to write to"};

sargp::Flag invertVertexFanning {"invert_vertex_fanning", "reorder the vertices of all facets"};
sargp::Flag addInvertedVerteces {"add_inverted_vertices", "add reordered vertices of all facets (effectively copies all vertives once)"};

}

int main(int argc, char** argv)
{
	if (std::string(argv[argc-1]) == "--compgen") {
		std::cout << sargp::compgen(argc-2, argv+1);
		return 0;
	}
	// pass everything except the name of the application
	sargp::parseArguments(argc-1, argv+1);

	if (printHelp) {
		std::cout << sargp::generateHelpString(std::regex{".*" + printHelp.get().value_or("") + ".*"}) << std::endl;
		return 0;
	}

	if (not inFile) {
		throw std::runtime_error("in has to be specified!");
	}
	if (not outFile) {
		throw std::runtime_error("out has to be specified!");
	}

	kinematicTree::visual::stl::STLParser parser;
	auto mesh = parser.parse(*inFile);
	std::cout << transform << std::endl;
	mesh.applyTransform(transform);

	if (addInvertedVerteces) {
		auto meshInverted = mesh;
		meshInverted.invertVertexFanning();
		for (auto const& facet : meshInverted.getFacets()) {
			mesh.addFacet(facet);
		}
	}

	if (invertVertexFanning) {
		mesh.invertVertexFanning();
	}

	parser.dump(mesh, *outFile);
	std::cout << "done" << std::endl;

	return 0;
}
