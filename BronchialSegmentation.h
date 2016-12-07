#ifndef BRONCHIAL_SEGMENTATION_H
#define BRONCHIAL_SEGMENTATION_H
#include <Eigen/Dense>
#include <unordered_set>
#include "itkImage.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"
#include "itkThresholdImageFilter.h"
#include "itkImageFileWriter.h"
#include <itksys/SystemTools.hxx>
#include "itkNumericSeriesFileNames.h"
#include "itkImageSeriesWriter.h"
#include "itkExceptionObject.h"
#include "CollisionDetection.h"

typedef itk::Image<float, 2> Image2DType;
typedef itk::Image<float, 3> Image3DType;
typedef itk::ImageSeriesReader<Image3DType> ReaderType;
typedef itk::GDCMImageIO ImageIOType;
typedef itk::GDCMSeriesFileNames NamesGeneratorType;
typedef std::vector<std::string> SeriesIDContainer;
typedef std::vector<std::string> FileNamesContainer;
typedef itk::ThresholdImageFilter<Image3DType> ThresholdImageFilterType;
typedef itk::NumericSeriesFileNames OutputNamesGeneratorType;
typedef itk::ImageSeriesWriter<Image3DType, Image2DType> SeriesWriterType;

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

class BronchialSegmentation : public CollisionDetection {
public:
	Image3DType* segmentedImage;
	std::vector< Node* > nodeVector;
	ReaderType::Pointer reader;
	ImageIOType::Pointer dicomIO;
	Image3DType::SizeType size;
	bool inCollision(const std::vector<Eigen::Vector3d> & point1s,
			   					 const std::vector<Eigen::Vector3d> & point2s,
			   			 		 const std::vector<double> & radii,
			   			 		 std::vector<int> & indices) const;
	BronchialSegmentation(int argc, char** argv);

	bool ptIsInCylinder(const Eigen::Vector3d &pt, Cylinder* cylinder) const;
	void visualize(int argc, char** argv);
	Eigen::Vector3d ptOnLine(const Eigen::Vector3d &pt, Line3D* line) const;
	Eigen::Vector3d moveAlongLine(Line3D* line, double distance) const;
	double ptToLineDistance(const Eigen::Vector3d &pt, Line3D* line) const;
	bool pointIsOnSegment(const Eigen::Vector3d &pt, Line3D *line) const;
	double magnitude(const Eigen::Vector3d &pt) const;
	double pointDistance(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2) const;
	bool distanceEqual(double d1, double d2) const;
	int findIndex(Image3DType::SizeType size, Image3DType::IndexType index) const;
	int findIndex(Image3DType::SizeType size, int x, int y, int z) const;
	int isEqual(Image3DType::IndexType index1, Image3DType::IndexType index2) const;
	double cylinderMaxDistance(Cylinder* cylinder) const;
	Cube* boundingCube(Cylinder* cylinder) const;
	Eigen::Vector3d lowestPoint(Cube* cube) const;
	Eigen::Vector3d highestPoint(Cube* cube) const;
  std::vector< Node* > getGraph();
};
#endif
