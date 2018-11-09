/*
 * Mesh.cpp
 *
 *  Created on: 30.11.2015
 *      Author: lutz
 */

#include "Mesh.h"
#include <algorithm>

namespace kinematicTree
{
namespace visual
{
namespace mesh
{

Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}


void Mesh::addFacet(Facet const& facet)
{
	mFacets.push_back(facet);
	for (arma::colvec3 const& vertex: facet.mVertices) {
		mVertices.push_back(vertex);
		mVertexIndices.push_back(mVertices.size()- 1);
	}
}

void Mesh::setName(std::string const& name)
{
	mName = name;
}


std::string Mesh::getName() const
{
	return mName;
}

std::vector<Facet> const& Mesh::getFacets() const
{
	return mFacets;
}

void Mesh::applyTransform(arma::mat44 const& transform)
{
	for (arma::colvec3 & vertex : mVertices) {
		vertex = transform.submat(0, 0, 2, 2) * vertex + transform.submat(0, 3, 2, 3);
	}
	for (auto& facet : mFacets) {
		for (arma::colvec3 & vertex : facet.mVertices) {
			vertex = transform.submat(0, 0, 2, 2) * vertex + transform.submat(0, 3, 2, 3);
		}
		facet.mNormal = transform.submat(0, 0, 2, 2) * facet.mNormal;
	}

	if (arma::det(transform.submat(0,  0, 2, 2)) < 0) {
		invertVertexFanning();
	}
}


void Mesh::invertVertexFanning() {
	for (auto& facet : mFacets) {
		std::swap(facet.mVertices[0], facet.mVertices[1]);
	}
}

}
}
}
