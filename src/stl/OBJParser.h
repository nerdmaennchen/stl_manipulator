#pragma once

#include "Mesh.h"

namespace kinematicTree {
namespace visual {
namespace obj {

class OBJParser {
public:
	OBJParser(){}

	std::string getMaterialFileName(std::string file);
	kinematicTree::visual::mesh::Mesh parse(std::string file);
};

} /* namespace obj */
} /* namespace visual */
} /* namespace kinematicTree */
