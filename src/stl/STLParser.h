#pragma once

#include "Mesh.h"

namespace kinematicTree
{
namespace visual
{
namespace stl
{

class STLParser
{
public:
	STLParser();
	virtual ~STLParser();

	kinematicTree::visual::mesh::Mesh parse(std::string const& file);

	void dump(kinematicTree::visual::mesh::Mesh const&, std::string const& file);
};

}
}
}
