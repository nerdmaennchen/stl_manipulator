/*
 * STLParser.cpp
 *
 *  Created on: 30.11.2015
 *      Author: lutz
 */

#include "STLParser.h"
#include <iostream>
#include <fstream>
#include <string>

#include <armadillo>
#include <cassert>
#include <array>



namespace kinematicTree
{
namespace visual
{
namespace stl
{

STLParser::STLParser()
{
}

STLParser::~STLParser()
{
}

// helper parsers
namespace {
	arma::colvec3 parseVector(std::ifstream& fileStream) {
		arma::colvec3 vector;
		int i(0);
		while ((not fileStream.eof()) && (i < 3)) {
			double val;
			fileStream >> val;
			vector(i) = val;
			++i;
		}
		return vector;
	}

	std::string readWord(std::ifstream& fileStream) {
		std::string word;
		fileStream >> word;
		return word;
	}

	void parseWord(std::ifstream& fileStream, std::string const& word) {
		std::string w;
		fileStream >> w;
		if (w != word) {
			throw (w == word);
		}
	}

	bool tryParseWord(std::ifstream& fileStream, std::string const& word) {
		std::string w;
		int pos = fileStream.tellg();
		fileStream >> w;

		const bool match = w == word;
		if (not match) {
			fileStream.seekg(pos);
		}

		return match;
	}

	bool parseVertex(std::ifstream& fileStream, arma::colvec3& vertex) {
		if (tryParseWord(fileStream, "vertex")) {
			vertex = parseVector(fileStream);
			return true;
		}
		return false;
	}

	bool parseFacetASCII(std::ifstream& fileStream, kinematicTree::visual::mesh::Facet& facet) {
		if (tryParseWord(fileStream, "facet")) {
			parseWord(fileStream, "normal");
			facet.mNormal = parseVector(fileStream);
			parseWord(fileStream, "outer");
			parseWord(fileStream, "loop");

			arma::colvec3 vertex;
			while (parseVertex(fileStream, vertex)) {
				facet.mVertices.push_back(vertex);
			}
			assert(facet.mVertices.size() == 3);
			parseWord(fileStream, "endloop");
			parseWord(fileStream, "endfacet");
			return true;
		} else {
			return false;
		}
	}

	kinematicTree::visual::mesh::Facet parseFacetBinary(std::ifstream& fileStream) {
		kinematicTree::visual::mesh::Facet ret;
		std::array<float, 3> normalVec;
		std::array<float, 9> vertices;
		uint16_t attrs;

		fileStream.read((char*)normalVec.data(), sizeof(normalVec));
		fileStream.read((char*)vertices.data(), sizeof(vertices));
		fileStream.read((char*)&attrs, sizeof(attrs));

		for (uint i(0); i < 3; ++i) {
			ret.mNormal(i) = normalVec[i];
			arma::colvec3 vertex;
			for (uint j(0); j < 3; ++j) {
				vertex(j) = vertices[3 * i + j];
			}
			ret.mVertices.push_back(vertex);
		}

		return ret;
	}

	kinematicTree::visual::mesh::Mesh parseASCII(std::ifstream& fileStream) {
		kinematicTree::visual::mesh::Mesh mesh;
		mesh.setName(readWord(fileStream)); // the next string is a dummy string
		// parse the entire solid
		while (1) {
			kinematicTree::visual::mesh::Facet facet;
			if (parseFacetASCII(fileStream, facet)) {
				mesh.addFacet(facet);
			} else {
				break;
			}
		}
		parseWord(fileStream, "endsolid");
		parseWord(fileStream, mesh.getName());
		return mesh;
	}


	kinematicTree::visual::mesh::Mesh parseBinary(std::ifstream& fileStream) {
		kinematicTree::visual::mesh::Mesh mesh;
		mesh.setName(""); // the next string is a dummy string
		char header[80];
		fileStream.read(header, sizeof(header));
		uint32_t numTriangles;
		fileStream.read((char*)&numTriangles, sizeof(numTriangles));

		std::streampos curPos = fileStream.tellg();
		fileStream.seekg( 0, std::ios::end );
		std::streampos endPos = fileStream.tellg();
		int expectedBytes = numTriangles * (12 * sizeof(float) + 2);
		assert(endPos - curPos == expectedBytes);
		fileStream.seekg(curPos);
		for (uint i(0); i < numTriangles; ++i) {
			mesh.addFacet(parseFacetBinary(fileStream));
		}
		return mesh;
	}
}

kinematicTree::visual::mesh::Mesh STLParser::parse(std::string const& file)
{
	std::ifstream stlFile(file);
	kinematicTree::visual::mesh::Mesh mesh;
	if (not stlFile.eof()) {
		if (tryParseWord(stlFile, "solid")) {
			try {
				mesh = parseASCII(stlFile);
			} catch (...) {
				stlFile.seekg(0);
				mesh = parseBinary(stlFile);
			}
		} else {
			mesh =parseBinary(stlFile);
		}
	}
	return mesh;
}


void STLParser::dump(kinematicTree::visual::mesh::Mesh const& mesh, std::string const& file) {
	std::ofstream fileStream(file);
	char header[80] = {};
	fileStream.write(header, sizeof(header));
	uint32_t numTriangles = mesh.getFacets().size();
	fileStream.write((char const*)&numTriangles, sizeof(numTriangles));

	for (auto const& facet : mesh.getFacets()) {
		uint16_t attrs{};

		for (uint i(0); i < 3; ++i) {
			float f = facet.mNormal(i);
			fileStream.write((char const*)&f, sizeof(f));
		}

		for (uint i(0); i < 3; ++i) {
			for (uint j(0); j < 3; ++j) {
				float f = facet.mVertices[i](j);
				fileStream.write((char const*)&f, sizeof(f));
			}
		}
		fileStream.write((char const*)&attrs, sizeof(attrs));
	}
}

}
}
}
