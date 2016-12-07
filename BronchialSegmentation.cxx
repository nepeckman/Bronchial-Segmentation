#include "BronchialSegmentation.h"
#include "CollisionDetection.h"
#include <iostream>
#include <string>
#include <vector>
#include <unordered_set>
#include <Eigen/Dense>
#include <queue>
#include <algorithm>
#include <cmath>
#include <itksys/SystemTools.hxx>
#include "itkExceptionObject.h"

BronchialSegmentation::BronchialSegmentation(int argc, char** argv)
{
	// create name generator
	std::cout << "Creating name generator" << std::endl;
	NamesGeneratorType::Pointer nameGenerator = NamesGeneratorType::New();
	nameGenerator->SetDirectory(argv[1]);

	// create dicom reader
	std::cout << "Creating dicom reader" << std::endl;
	reader = ReaderType::New();
	dicomIO = ImageIOType::New();
	reader->SetImageIO(dicomIO);

	// get series IDs
	std::cout << "Creating dicom series ID" << std::endl;
	const SeriesIDContainer &seriesUID = nameGenerator->GetSeriesUIDs();

	// get dicom series
	std::cout << "Getting dicom series" << std::endl;
	FileNamesContainer fileNames;
	fileNames = nameGenerator->GetFileNames(seriesUID.begin()->c_str());
	reader->SetFileNames(fileNames);
	reader->Update();

	// read image data
	std::cout << "Reading image data" << std::endl;
	Image3DType::Pointer image = reader->GetOutput();
	Image3DType::RegionType region = image->GetLargestPossibleRegion();
	size = region.GetSize();

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
	ThresholdImageFilterType::Pointer thresholdFilter = ThresholdImageFilterType::New();
	thresholdFilter->ThresholdOutside(minIntensity, maxIntensity);
	thresholdFilter->SetOutsideValue(0);
	thresholdFilter->SetInput(reader->GetOutput());
	thresholdFilter->Update();

	segmentedImage = thresholdFilter->GetOutput();

	//  make graph
	//  intialize nodes

	std::cout << "Initializing graph nodes" << std::endl;
	Node *tail = nullptr;
	Node *current = nullptr;
	Node *head = nullptr;

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
					tail->next = nullptr;
					tail->processed = 0;
					if(!head)
					{
						head = tail;
						current = tail;
					} else
					{
						current->next = tail;
					}
					current = tail;
				} else
				{
					nodeVector.push_back(nullptr);
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
						(adjacentNode != nullptr))
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
		if(nCurrent != nullptr && !nCurrent->processed)
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
				nodeVector[findIndex(size, itIndex)] = nullptr;
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
		OutputNamesGeneratorType::Pointer outputNames = OutputNamesGeneratorType::New();
		std::string seriesFormat = std::string(writedir) + "/image-%05d.dcm";
		outputNames->SetSeriesFormat(seriesFormat.c_str());
		outputNames->SetStartIndex(1);
		outputNames->SetEndIndex(size[2]);

		// create series writer
		reader->Update();

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
		}
}

int BronchialSegmentation::isEqual(Image3DType::IndexType index1, Image3DType::IndexType index2) const
{
	return (index1[0] == index2[0] && index1[1] == index2[1] && index1[2] == index2[2]);
}

int BronchialSegmentation::findIndex(Image3DType::SizeType size, Image3DType::IndexType index) const
{
	return index[0] * size[1] * size[2] + index[1] * size[2] + index[2];
}

int BronchialSegmentation::findIndex(Image3DType::SizeType size, int x, int y, int z) const
{
	return x * size[1] * size[2] + y * size[2] + z;
}

bool BronchialSegmentation::distanceEqual(double d1, double d2) const
{
	return std::abs(d1 - d2) < 0.01;
}

double BronchialSegmentation::pointDistance(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2) const
{
	double diffx = std::abs(pt1(0) - pt2(0));
	double diffy = std::abs(pt1(1) - pt2(1));
	double diffz = std::abs(pt1(2) - pt2(2));
	return std::sqrt(std::pow(diffx, 2) + std::pow(diffy, 2) + std::pow(diffz, 2));
}

double BronchialSegmentation::magnitude(const Eigen::Vector3d &pt) const
{
	return std::sqrt(std::pow(pt(0), 2) + std::pow(pt(1), 2) + std::pow(pt(2), 2));
}

bool BronchialSegmentation::pointIsOnSegment(const Eigen::Vector3d &pt, Line3D *line) const
{
	return distanceEqual(pointDistance(pt, line->pt1) + pointDistance(pt, line->pt2), pointDistance(line->pt1, line->pt2));
}

double BronchialSegmentation::ptToLineDistance(const Eigen::Vector3d &pt, Line3D* line) const
{
	double numerator = magnitude((pt - line->pt1).cross(pt - line->pt2));
	double denominator = magnitude(line->pt2 - line->pt1);
	return numerator/denominator;
}

Eigen::Vector3d BronchialSegmentation::moveAlongLine(Line3D* line, double distance) const
{
	Eigen::Vector3d vector = line->pt2 - line->pt1;
	Eigen::Vector3d movedVector = vector * distance/pointDistance(line->pt1, line->pt2);
	return line->pt1 + movedVector;
}

Eigen::Vector3d BronchialSegmentation::ptOnLine(const Eigen::Vector3d &pt, Line3D* line) const
{
	double ptToSeg = ptToLineDistance(pt, line);
	double linePtToIntersection = std::sqrt(std::pow(pointDistance(line->pt1, pt), 2) - std::pow(ptToSeg, 2));
	return moveAlongLine(line, linePtToIntersection);
}

bool BronchialSegmentation::ptIsInCylinder(const Eigen::Vector3d &pt, Cylinder* cylinder) const
{
	Eigen::Vector3d intersection = ptOnLine(pt, cylinder->line);
	if(pointIsOnSegment(intersection, cylinder->line))
	{
		return pointDistance(intersection, pt) < cylinder->radius;
	} else
	{
		return false;
	}
}

double BronchialSegmentation::cylinderMaxDistance(Cylinder* cylinder) const
{
	Eigen::Vector3d midpoint = (cylinder->line->pt1 + cylinder->line->pt2)/2;
	double a = pointDistance(midpoint, cylinder->line->pt2);
	double b = cylinder->radius;
	return std::sqrt(std::pow(a, 2) + std::pow(b, 2));
}

Cube* BronchialSegmentation::boundingCube(Cylinder* cylinder) const
{
	double halfSide = cylinderMaxDistance(cylinder);
	Eigen::Vector3d midpoint = (cylinder->line->pt1 + cylinder->line->pt2)/2;
	Cube* box = new Cube;
	box->halfSide = halfSide;
	box->center = midpoint;
	return box;
}

Eigen::Vector3d BronchialSegmentation::lowestPoint(Cube* cube) const
{
	Eigen::Vector3d lowest;
	lowest(0) = cube->center(0) - cube->halfSide;
	lowest(1) = cube->center(1) - cube->halfSide;
	lowest(2) = cube->center(2) - cube->halfSide;
	return lowest;
}
Eigen::Vector3d BronchialSegmentation::highestPoint(Cube* cube) const
{
	Eigen::Vector3d highest;
	highest(0) = cube->center(0) + cube->halfSide;
	highest(1) = cube->center(1) + cube->halfSide;
	highest(2) = cube->center(2) + cube->halfSide;
	return highest;
}

bool BronchialSegmentation::inCollision(const std::vector<Eigen::Vector3d> & point1s,
								 const std::vector<Eigen::Vector3d> & point2s,
								 const std::vector<double> & radii,
								 std::vector<int> & indices) const
{
		for(std::vector< Eigen::Vector3d >::size_type i = 0; i != point1s.size(); i++)
		{
			Cylinder* cylinder = new Cylinder;
			cylinder->line = new Line3D;
			cylinder->line->pt1 = point1s[i];
			cylinder->line->pt2 = point2s[i];
			cylinder->radius = radii[i];
			Cube* cube = boundingCube(cylinder);
			Eigen::Vector3d lowest = lowestPoint(cube);
			Eigen::Vector3d highest = highestPoint(cube);
			int x1 = (int) std::floor(lowest(0));
			int y1 = (int) std::floor(lowest(1));
			int z1 = (int) std::floor(lowest(2));
			int x2 = (int) std::ceil(highest(0));
			int y2 = (int) std::ceil(highest(1));
			int z2 = (int) std::ceil(highest(2));
			for(int x = x1; x <= x2; x++)
			{
				for(int y = y1; y <= y2; y++)
				{
					for(int z = z1; z <= z2; z++)
					{
						Eigen::Vector3d point;
						point(0) = x;
						point(1) = y;
						point(2) = z;
						if(ptIsInCylinder(point, cylinder) && nodeVector[findIndex(size, x, y, z)])
						{
							return true;
						}
					}
				}
			}
			delete cylinder->line;
			delete cylinder;
		}
		return false;
}

std::vector< Node* > BronchialSegmentation::getGraph()
{
	return nodeVector;
}
