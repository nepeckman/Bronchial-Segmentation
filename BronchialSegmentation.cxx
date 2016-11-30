#include <iostream>
#include <string>
#include <vector>
#include <unordered_set>
#include <queue>
#include <algorithm>
#include <cmath>
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

struct Point3D {
	double x;
	double y;
	double z;
};

struct Line3D {
	Point3D* pt1;
	Point3D* pt2;
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
double ptDistance(Point3D*, Point3D*);
double magnitude(Point3D*);
Point3D* crossProduct(Point3D*, Point3D*);
Point3D* addPoints(Point3D*, Point3D*);
Point3D* subPoints(Point3D*, Point3D*);
Point3D* scaleVector(Point3D*, double);
bool ptIsOnSegment(Point3D*, Line3D*);
double ptToSegmentDistance(Point3D*, Line3D*);
Point3D* moveAlongLine(Line3D*, double);
Point3D* ptOnLine(Point3D*, Line3D*);

int main(int argc, char** argv)
{
		
	// create name generator
	std::cout << "Creating name generator" << std::endl;
	typedef itk::GDCMSeriesFileNames NamesGeneratorType;
	NamesGeneratorType::Pointer nameGenerator = NamesGeneratorType::New();
	nameGenerator->SetDirectory(argv[1]);

	// create dicom reader
	std::cout << "Creating dicom reader" << std::endl;
	typedef itk::ImageSeriesReader<Image3DType> ReaderType;
	ReaderType::Pointer reader = ReaderType::New();
	typedef itk::GDCMImageIO ImageIOType;
	ImageIOType::Pointer dicomIO = ImageIOType::New();
	reader->SetImageIO(dicomIO);

	// get series IDs
	std::cout << "Creating dicom series ID" << std::endl;
	typedef std::vector<std::string> SeriesIDContainer;
	const SeriesIDContainer &seriesUID = nameGenerator->GetSeriesUIDs();

	// get dicom series
	std::cout << "Getting dicom series" << std::endl;
	typedef std::vector<std::string> FileNamesContainer;
	FileNamesContainer fileNames;
	fileNames = nameGenerator->GetFileNames(seriesUID.begin()->c_str());
	reader->SetFileNames(fileNames);

	reader->Update();

	// read image data
	std::cout << "Reading image data" << std::endl;
	Image3DType::Pointer image = reader->GetOutput();
	Image3DType::RegionType region = image->GetLargestPossibleRegion();
	Image3DType::SizeType size = region.GetSize();

	// create threshold filter
	std::cout << "Creating threshold filter" << std::endl;
	int maxIntensity, minIntensity;
	if(argc < 5)
	{
		minIntensity = 0;
		maxIntensity = 65536;
	} else 
	{
		minIntensity = std::stoi(argv[3]);
		maxIntensity = std::stoi(argv[4]);
	}
	std::cout << "Threshold below \"" << minIntensity << "\"" << std::endl;
	std::cout << "Threshold above \"" << maxIntensity << "\"" << std::endl;
	typedef itk::ThresholdImageFilter<Image3DType> ThresholdImageFilterType;
	ThresholdImageFilterType::Pointer thresholdFilter = ThresholdImageFilterType::New();
	thresholdFilter->ThresholdOutside(minIntensity, maxIntensity); 
	thresholdFilter->SetOutsideValue(0);
	thresholdFilter->SetInput(reader->GetOutput());
	thresholdFilter->Update();

	Image3DType *segmentedImage = thresholdFilter->GetOutput();

	//  make graph
	//  intialize nodes	
	std::cout << "Initializing graph nodes" << std::endl;
	Node *tail;
	Node *current;
	Node *head;

	std::vector< Node* > nodeVector;
	for(int i = 0; i < size[0]; i++)
	{
		for(int j = 0; j < size[1]; j++)
		{
			for(int k = 0; k < size[2]; k++)
			{
				Image3DType::IndexType *index = new Image3DType::IndexType;
				(*index)[0] = i;
				(*index)[1] = j;
				(*index)[2] = k;
				Image3DType::PixelType pixelValue = segmentedImage->GetPixel(*index);
				if(pixelValue > 0)
				{
					tail = new Node;
					nodeVector.push_back(tail);
					tail->index = index;
					tail->previous = current;
					tail->processed = 0;
					if(!head)
					{
						head = tail;
					} else
					{
						current->next = tail;
					}
					current = tail;
				} else
				{
					nodeVector.push_back(NULL);
				}
			}
		}
	}

	// connect nodes
	std::cout << "Connecting graph nodes" << std::endl;
	current = head;
	while(current)
	{
		Image3DType::IndexType currentIndex = *(current->index);
		for(int i = currentIndex[0] - 1; i < currentIndex[0] + 2; i++)
		{
			for(int j = currentIndex[1] - 1; j < currentIndex[1] + 2; j++)
			{
				for(int k = currentIndex[2] - 1; k < currentIndex[2] + 2; k++)
				{
					if( i > -1 && j > -1 && k > -1)
					{
						Image3DType::IndexType* adjacentIndex = new Image3DType::IndexType;
						(*adjacentIndex)[0] = i;
						(*adjacentIndex)[1] = j;
						(*adjacentIndex)[2] = k;
						Node* adjacentNode = nodeVector[findIndex(size, *adjacentIndex)];
						if( (currentIndex[0] != i || currentIndex[1] != j || currentIndex[2] != k) && 
						(adjacentNode != NULL))
						{
							Link *link = new Link;
							link->from = current;
							link->to = adjacentNode;
							link->next = current->links;
							current->links = link;
						}
						delete adjacentIndex;
					}
				}
			}
		}
		current = current->next;	
	}

	std::cout << "Finding connected objects" << std::endl;
	ObjectVectorType objects;
	for(std::vector< Node*>::size_type i = 0; i != nodeVector.size(); i++)
	{
		Node* nCurrent = nodeVector[i];
		if(nCurrent != NULL && !nCurrent->processed)
		{
			std::unordered_set< Node* >* object = new std::unordered_set< Node*>;
			objects.push_back(object);
			std::queue< Node*> queue;
			queue.push(nCurrent);
			object->insert(nCurrent);
			nCurrent->processed = 1;
			while(!queue.empty())
			{
				Node* current = queue.front();
				queue.pop();
				Link* lCurrent = current->links;
				while(lCurrent)
				{
					Node* node = lCurrent->to;
					if(!node->processed)
					{
						queue.push(node);
						object->insert(node);
						node->processed = 1;
					}
					lCurrent = lCurrent->next;
				}
			}
		}
	}

	std::cout << "Cleaning small objects" << std::endl;
	for(ObjectVectorType::size_type i = 0; i != objects.size(); i++)
	{
		std::unordered_set< Node* > object = *objects[i];
		if(object.size() > 500000 || object.size() < 100000)
		{
			for(std::unordered_set< Node*>::iterator it = object.begin(); it != object.end(); ++it)
			{
				Node* n = *(it);
				Image3DType::IndexType itIndex = *(n->index);
				nodeVector[findIndex(size, itIndex)] = NULL;
				segmentedImage->SetPixel(itIndex, 0);
			}
		}
	}

	// make output directory
	std::string writedir;
	if(argc < 2)
	{
		writedir = "out";
	} else
	{
		writedir = argv[2];
	}
	std::cout << "Write directory is \"" << writedir << "\"" << std::endl;


	itksys::SystemTools::MakeDirectory(writedir);

	// generate the file names
	typedef itk::NumericSeriesFileNames OutputNamesGeneratorType;
	OutputNamesGeneratorType::Pointer outputNames = OutputNamesGeneratorType::New();
	std::string seriesFormat = std::string(writedir) + "/image-%05d.dcm";
	outputNames->SetSeriesFormat(seriesFormat.c_str());
	outputNames->SetStartIndex(1);
	outputNames->SetEndIndex(size[2]);

	// create series writer
	reader->Update();
	typedef itk::ImageSeriesWriter<Image3DType, Image2DType> SeriesWriterType;

	SeriesWriterType::Pointer seriesWriter = SeriesWriterType::New();
	seriesWriter->SetInput(segmentedImage);
	seriesWriter->SetImageIO(dicomIO);
	seriesWriter->SetFileNames(outputNames->GetFileNames());
	seriesWriter->SetMetaDataDictionaryArray(reader->GetMetaDataDictionaryArray());

	try
	{
		seriesWriter->Update();
	}
	catch(itk::ExceptionObject & excp)
	{
		std::cerr << "Exception throw while writing the series" << std::endl;
		std::cerr << excp << std::endl;
		return 1;
	}
	return 0;
}

int isEqual(Image3DType::IndexType index1, Image3DType::IndexType index2)
{
	return (index1[0] == index2[0] && index1[1] == index2[1] && index1[2] == index2[2]);
}

Node* buildGraph(Image3DType* image)
{
	Image3DType::RegionType region = image->GetLargestPossibleRegion();
	Image3DType::SizeType size = region.GetSize();

	std::cout << size << std::endl;
	return NULL;
}

int findIndex(Image3DType::SizeType size, Image3DType::IndexType index)
{
	return index[0] * size[1] * size[2] + index[1] * size[2] + index[2];
}

bool ptEqual(Point3D* pt1, Point3D* pt2)
{
	double diffx = std::abs(pt1->x - pt2->x);
	double diffy = std::abs(pt1->y - pt2->y);
	double diffz = std::abs(pt1->z - pt2->z);
	return diffx < 0.01 && diffy < 0.01 && diffz < 0.01;
}

bool distanceEqual(double d1, double d2)
{
	return std::abs(d1 - d2) < 0.01;
}

double ptDistance(Point3D* pt1, Point3D* pt2)
{
	double diffx = std::abs(pt1->x - pt2->x);
	double diffy = std::abs(pt1->y - pt2->y);
	double diffz = std::abs(pt1->z - pt2->z);
	return std::sqrt(std::pow(diffx, 2) + std::pow(diffy, 2) + std::pow(diffz, 2));
}

Point3D* addPoints(Point3D* pt1, Point3D* pt2)
{
	Point3D* rv = new Point3D;
	rv->x = pt1->x + pt2->x;
	rv->y = pt1->y + pt2->y;
	rv->z = pt1->z + pt2->z;
	return rv;
}

Point3D* subPoints(Point3D* pt1, Point3D* pt2)
{
	Point3D* rv = new Point3D;
	rv->x = pt1->x - pt2->x;
	rv->y = pt1->y - pt2->y;
	rv->z = pt1->z - pt2->z;
	return rv;
}

Point3D* scaleVector(Point3D* pt, double c)
{
	Point3D* rv = new Point3D;
	rv->x = pt->x * c;
	rv->y = pt->y * c;
	rv->z = pt->z * c;
	return rv;
}

Point3D* crossProduct(Point3D* pt1, Point3D* pt2)
{
	Point3D* rv = new Point3D;
	rv->x = pt1->y * pt2->z - pt1->z * pt2->y;
	rv->y = pt1->z * pt2->x - pt1->x * pt2->z;
	rv->z = pt1->x * pt2->y - pt1->y * pt2->x;
	return rv;
}

double magnitude(Point3D* pt)
{
	return std::sqrt(std::pow(pt->x, 2) + std::pow(pt->y, 2) + std::pow(pt->z, 2));
}

bool ptIsOnSegment(Point3D* pt, Line3D* line)
{
	return distanceEqual(ptDistance(pt, line->pt1) + ptDistance(pt, line->pt2), ptDistance(line->pt1, line->pt2));
}

double ptToLineDistance(Point3D* pt, Line3D* line)
{
	double numerator = magnitude(crossProduct(subPoints(pt, line->pt1), subPoints(pt, line->pt2)));
	double denominator = magnitude(subPoints(line->pt2, line->pt1));
	return numerator/denominator;
}

Point3D* moveAlongLine(Line3D* line, double distance)
{
	Point3D* vector = subPoints(line->pt2, line->pt1);
	Point3D* movedVector = scaleVector(vector, distance/ptDistance(line->pt1, line->pt2));
	return addPoints(line->pt1, movedVector);
}

Point3D* ptOnLine(Point3D* pt, Line3D* line)
{
	double ptToSeg = ptToSegmentDistance(pt, line);
	double linePtToIntersection = std::sqrt(std::pow(ptDistance(line->pt1, pt), 2) - std::pow(ptToSeg, 2));
	return moveAlongLine(line, linePtToIntersection);
}
