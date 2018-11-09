#pragma once

/*
 *  Created on: 30.11.2015
 *      Author: lutz
 */

#include <vector>
#include <armadillo>
#include <string>

namespace kinematicTree
{
namespace visual
{
namespace mesh
{

class Facet {
public:
	std::vector<arma::colvec3> mVertices;
	arma::colvec3 mNormal;
};

class Mesh
{
public:
	Mesh();
	virtual ~Mesh();

	void addFacet(Facet const& facet);
	void setName(std::string const& name);
	std::string getName() const;

	void applyTransform(arma::mat44 const& transform);

	std::vector<Facet> const& getFacets() const;

	void invertVertexFanning();

private:
	std::vector<Facet> mFacets;

	std::vector<arma::colvec3> mVertices;
	std::vector<uint> mVertexIndices;
	std::string mName;
};

}
}
}
