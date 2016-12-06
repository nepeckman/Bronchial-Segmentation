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

typedef std::vector< std::unordered_set< Node* >* > ObjectVectorType;
Node* findNode(Image3DType::IndexType, Node*);
Node* buildGraph(Image3DType*);
int findIndex(Image3DType::SizeType, Image3DType::IndexType);
int isEqual(Image3DType::IndexType, Image3DType::IndexType);
bool distanceEqual(double, double);
double pointDistance(Eigen::Vector3d &, Eigen::Vector3d &);
double magnitude(Eigen::Vector3d &);
bool pointIsOnSegment(Eigen::Vector3d &, Line3D*);
double ptToLineDistance(Eigen::Vector3d &, Line3D*);
Eigen::Vector3d moveAlongLine(Line3D*, double);
Eigen::Vector3d ptOnLine(Eigen::Vector3d&, Line3D*);
bool ptIsInCylinder(Eigen::Vector3d &, Cylinder*);
