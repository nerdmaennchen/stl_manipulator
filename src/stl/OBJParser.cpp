/*
 * OBJParser.cpp
 *
 *  Created on: 22.04.2016
 *      Author: flutz
 */

#include "OBJParser.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <array>
#include <vector>
#include <algorithm>

#include <armadillo>

namespace kinematicTree {
namespace visual {
namespace obj {

namespace {

arma::colvec3 parseVector(std::stringstream& stream) {
	arma::colvec3 vector;
	int i(0);
	while ((not stream.eof()) && (i < 3)) {
		double val;
		stream >> val;
		vector(i) = val;
		++i;
	}
	return vector;
}

std::vector<int> parseFacet(std::stringstream& stream) {
	std::vector<int> retVector;
	std::string line = stream.str().substr(stream.tellg());
	std::replace(line.begin(), line.end(), '/', ' ');
	std::stringstream lineStream(line);
	int idx;
	while (not lineStream.eof()) {
		lineStream >> idx;
		retVector.push_back(idx - 1);
	}
	return retVector;
}

std::string readWord(std::stringstream& stream) {
	std::string word;
	stream >> word;
	return word;
}

}


std::string OBJParser::getMaterialFileName(std::string file)
{
	std::ifstream stlFile(file);
	kinematicTree::visual::mesh::Mesh mesh;
	std::vector<std::string> lines;
	std::string line;
	std::string token;
	std::string mtlFileName = "";
	while (std::getline(stlFile, line, '\n')) {
		std::stringstream lineStream(line);
		token = readWord(lineStream);
		if (token == "mtllib") {
			mtlFileName = readWord(lineStream);
			return mtlFileName;
		}
	}
	return mtlFileName;
}

kinematicTree::visual::mesh::Mesh OBJParser::parse(std::string file)
{
	std::ifstream stlFile(file);
	kinematicTree::visual::mesh::Mesh mesh;
	std::vector<std::string> lines;
	std::string line;
	std::vector<arma::colvec3> vertices;
	std::vector<std::vector<int>> vertexIndexes;
	std::string token;
	while (std::getline(stlFile, line, '\n')) {
		std::stringstream lineStream(line);
		token = readWord(lineStream);
		if (token == "v") {
			vertices.push_back(parseVector(lineStream));
		} else if (token == "f") {
			vertexIndexes.emplace_back(parseFacet(lineStream));
		}
	}

	for (auto const& v : vertexIndexes) {
		int idx = 1;
		int len = v.size();
		while (idx < len - 1) {
			kinematicTree::visual::mesh::Facet facet;
			facet.mVertices.push_back(vertices[v[0]]);
			for (int i = 0; i < 2; ++i) {
				int vertexIdx = v[idx + i ];
				facet.mVertices.push_back(vertices[vertexIdx]);
			}
			mesh.addFacet(facet);
			idx += 1;
		}
	}

	return mesh;
}

} /* namespace obj */
} /* namespace visual */
} /* namespace kinematicTree */
