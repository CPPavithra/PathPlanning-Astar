#ifndef QUADTREECOMMON_H
#define QUADTREECOMMON_H

#include <vector>
#include <rerun.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "quadtree.h"

extern Point center;
extern float rootSize;
extern QuadtreeNode* lowQuadtree;
extern QuadtreeNode* midQuadtree;
extern QuadtreeNode* highQuadtree;

#endif // QUADTREECOMMON_H

