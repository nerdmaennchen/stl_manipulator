#pragma once

#include "sargparse/Parameter.h"

inline sargp::Parameter<std::vector<std::string>> inFiles  {{}, "in", "the file(s) to read from", []{}, sargp::completeFile("stl", sargp::File::Multi)};
