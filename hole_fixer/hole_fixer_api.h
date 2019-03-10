#pragma once
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstring>
#include <cerrno>
#ifdef HOLE_FIXER_EXPORTS
#define HOLE_FIXER_API __declspec(dllexport) 
#else
#define HOLE_FIXER_API __declspec(dllimport) 
#endif

HOLE_FIXER_API void createHolePatch(std::vector<std::vector<float>> &holeEdges,
									std::vector<std::vector<float>> &samplePoints,
									std::vector<std::vector<float>> &patchVertices,
									std::vector<std::vector<float>> &patchNormals,
									std::vector<std::vector<int>> &patchIndices);
HOLE_FIXER_API void createHoleSamplePoints(std::vector<std::vector<std::vector<float>>> &holeSamples);