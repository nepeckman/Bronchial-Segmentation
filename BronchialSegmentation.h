#ifndef BRONCHIAL_SEGMENTATION_H
#define BRONCHIAL_SEGMENTATION_H
#include </usr/local/Cellar/eigen/3.3.0/include/eigen3/Eigen/Dense>
#include <unordered_set>
#include "itkImage.h"

typedef itk::Image<float, 2> Image2DType;
typedef itk::Image<float, 3> Image3DType;

struct Link;
struct Node{
	Image3DType::IndexType *index;
	Link *links;
	Node *next;
	Node *previous;
	int processed;
};

struct Link{
	Node *to;
	Node *from;
	Link *next;
};

struct Line3D {
	Eigen::Vector3d pt1;
	Eigen::Vector3d pt2;
};

struct Cylinder{
	Line3D* line;
	double radius;
};

struct Cube{
  Eigen::Vector3d center;
  double halfSide;
};

typedef std::vector< std::unordered_set< Node* >* > ObjectVectorType;
Node* buildGraph(Image3DType*);

#endif
