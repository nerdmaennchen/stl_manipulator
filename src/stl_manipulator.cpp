#include "sargparse/Parameter.h"
#include "sargparse/File.h"
#include "sargparse/ArgumentParsing.h"

#include "stl/STLParser.h"

#include "transformer.h"
#include "global_parameters.h"

#include <iostream>

namespace {

sargp::Parameter<std::string> outFile {"", "out", "the file to write to", []{}, sargp::completeFile("stl", sargp::File::Single)};

sargp::Flag invertVertexFanning {"invert_vertex_fanning", "reorder the vertices of all facets"};
sargp::Flag addInvertedVerteces {"add_inverted_vertices", "add reordered vertices of all facets (effectively copies all vertives once)"};

void stl_manipulator()
{
	auto const& subC = sargp::getDefaultCommand().getSubCommands();
	if (std::any_of(begin(subC), end(subC), [](auto const& c) -> bool {return *c;})) {
		return;
	}
	if (not inFiles) {
		throw std::runtime_error("in has to be specified!");
	}
	if (not outFile) {
		throw std::runtime_error("out has to be specified!");
	}

    
    kinematicTree::visual::stl::STLParser parser;
    kinematicTree::visual::mesh::Mesh mesh;
    for (auto const& file : *inFiles) {
        std::cout << "loading : " << file << "\n";
        auto subMesh = parser.parse(file);
		for (auto const& facet : subMesh.getFacets()) {
			mesh.addFacet(facet);
		}
    }
	auto transform = getTransform();
    transform.print("applying transform");
    mesh.applyTransform(transform);

	if (addInvertedVerteces) {
        std::cout << "adding inverted vertices to the output mesh\n";
		auto meshInverted = mesh;
		meshInverted.invertVertexFanning();
		for (auto const& facet : meshInverted.getFacets()) {
			mesh.addFacet(facet);
		}
	}

	if (invertVertexFanning) {
        std::cout << "inverting the fanning\n";
		mesh.invertVertexFanning();
	}

	auto const inf = std::numeric_limits<double>::infinity();
	arma::colvec3 bb_min {inf, inf, inf};
	arma::colvec3 bb_max {-inf, -inf, -inf};
	for (auto const& facet : mesh.getFacets()) {
		for (auto const& vertex : facet.mVertices) {
			bb_min = arma::min(bb_min, vertex);
			bb_max = arma::max(bb_max, vertex);
		}
	}

	std::cout << "\n\nbounding box min: \n" << bb_min;

	std::cout << "\n\nbounding box max: \n" << bb_max;

	std::cout << "\n\ndimensions: \n" << (bb_max - bb_min);

    std::cout << "writing output file " << *outFile << "\n";
	parser.dump(mesh, *outFile);
	std::cout << "done" << std::endl;
}


sargp::Task task{stl_manipulator};

}