#include "CollisionDetection.h"
#include "BronchialSegmentation.h"
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <itksys/SystemTools.hxx>
#include "itkExceptionObject.h"

struct Box{
  std::vector<int> pt1;
  std::vector<int> pt2;
};

bool boxIsClear(Box*, std::vector< Node*>, Image3DType::SizeType);
bool clearDownZ(Box*, std::vector< Node*>, Image3DType::SizeType);
bool clearDownX(Box*, std::vector< Node*>, Image3DType::SizeType);
bool clearDownY(Box*, std::vector< Node*>, Image3DType::SizeType);
bool clearUpX(Box*, std::vector< Node*>, Image3DType::SizeType);
bool clearUpY(Box*, std::vector< Node*>, Image3DType::SizeType);
Box* moveDownZ(Box*);
Box* moveDownX(Box*);
Box* moveDownY(Box*);
Box* moveUpX(Box*);
Box* moveUpY(Box*);
void outputBox(Box*, Image3DType* image);
Eigen::Vector3d randFilledPoint(std::vector< Node* >);
Eigen::Vector3d randVector(double);
Eigen::Vector3d randPointInRadius(Eigen::Vector3d, double);
void testRandCylinder(BronchialSegmentation*, std::vector< Node* >, double);
std::vector<Box*> randPath(std::vector<Node*>, Image3DType::SizeType);
void testRandPath(BronchialSegmentation*, std::vector<Node*>, int);
void visualizePath(std::vector<Box*>);

int main(int argc, char** argv)
{
  BronchialSegmentation detector(argc, argv);
  std::vector< Node* > graph = detector.getGraph();
  std::cout << "Testing 50 Random Cylinders in Collision with the Object" << std::endl;
  for(int i = 0; i < 50; i++)
  {
    testRandCylinder(&detector, graph, 4);
  }
  std::cout << "Testing 10 Short Clear Random Paths" << std::endl;
  for(int i = 0; i < 10; i++)
  {
    testRandPath(&detector, graph, 10);
  }
  std::cout << "Testing 3 Long Clear Random Paths" << std::endl;
  for(int i = 0; i < 3; i++)
  {
    testRandPath(&detector, graph, 75);
  }

  return 0;
}

bool pointInImage(std::vector< int > pt, Image3DType::SizeType size)
{
  return (pt[0] > 0 && pt[0] < size[0] && pt[1] > 0 && pt[1] < size[1] && pt[2] > 0 && pt[2] < size[2]);
}

bool boxIsClear(Box* box, std::vector< Node*> graph, Image3DType::SizeType size)
{
  if(!pointInImage(box->pt1, size) || !pointInImage(box->pt2, size))
  {
    return false;
  }
  for(int x = box->pt1[0]; x <= box->pt2[0]; x++)
  {
    for(int y = box->pt1[1]; y <= box->pt2[1]; y++)
    {
      for(int z = box->pt1[2]; z >= box->pt2[2]; z--)
      {
        if(graph[x * size[1] * size[2] + y * size[2] + z])
        {
          return false;
        }
      }
    }
  }
  return true;
}

Box* moveBox(Box* box, int dx, int dy, int dz)
{
  Box* rv = new Box;
  rv->pt1.push_back(box->pt1[0] + dx);
  rv->pt1.push_back(box->pt1[1] + dy);
  rv->pt1.push_back(box->pt1[2] + dz);
  rv->pt2.push_back(box->pt2[0] + dx);
  rv->pt2.push_back(box->pt2[1] + dy);
  rv->pt2.push_back(box->pt2[2] + dz);
  return rv;
}

Box* moveDownX(Box* box)
{
  return moveBox(box, -1, 0, 0);
}

Box* moveDownY(Box* box)
{
  return moveBox(box, 0, -1, 0);
}

Box* moveDownZ(Box* box)
{
  return moveBox(box, 0, 0, -1);
}

Box* moveUpX(Box* box)
{
  return moveBox(box, 1, 0, 0);
}

Box* moveUpY(Box* box)
{
  return moveBox(box, 0, 1, 0);
}

void outputBox(Box* box, Image3DType* image)
{
  for(int i = box->pt1[0]; i < box->pt2[0]; i++)
  {
    for(int j = box->pt1[1]; j < box->pt2[1]; j++)
    {
      for(int k = box->pt1[2]; k > box->pt2[2]; k--)
      {
        Image3DType::IndexType index;
        index[0] = i;
        index[1] = j;
        index[2] = k;
        image->SetPixel(index, 30000);
      }
    }
  }
}

bool clearDownZ(Box* box, std::vector< Node*> graph, Image3DType::SizeType size)
{
  if(!pointInImage(box->pt1, size) || !pointInImage(box->pt2, size))
  {
    return false;
  }
  int z = box->pt2[2] - 1;
  for(int x = box->pt1[0]; x <= box->pt2[0]; x++)
  {
    for(int y = box->pt1[1]; y <= box->pt2[1]; y++)
    {
      if(graph[x * size[1] * size[2] + y * size[2] + z])
      {
        return false;
      }
    }
  }
  return true;
}

bool clearDownX(Box* box, std::vector< Node*> graph, Image3DType::SizeType size)
{
  if(!pointInImage(box->pt1, size) || !pointInImage(box->pt2, size))
  {
    return false;
  }
  int x = box->pt1[0] - 1;
  for(int z = box->pt2[2]; z >= box->pt1[2]; z++)
  {
    for(int y = box->pt1[1]; y <= box->pt2[1]; y++)
    {
      if(graph[x * size[1] * size[2] + y * size[2] + z])
      {
        return false;
      }
    }
  }
  return true;
}

bool clearDownY(Box* box, std::vector< Node*> graph, Image3DType::SizeType size)
{
  if(!pointInImage(box->pt1, size) || !pointInImage(box->pt2, size))
  {
    return false;
  }
  int y = box->pt1[1] - 1;
  for(int x = box->pt1[0]; x <= box->pt2[0]; x++)
  {
    for(int z = box->pt2[2]; z >= box->pt1[2]; z++)
    {
      if(graph[x * size[1] * size[2] + y * size[2] + z])
      {
        return false;
      }
    }
  }
  return true;
}

bool clearUpX(Box* box, std::vector< Node*> graph, Image3DType::SizeType size)
{
  if(!pointInImage(box->pt1, size) || !pointInImage(box->pt2, size))
  {
    return false;
  }
  int x = box->pt2[0] + 1;
  for(int z = box->pt2[2]; 2 >= box->pt1[2]; z++)
  {
    for(int y = box->pt1[1]; y <= box->pt2[1]; y++)
    {
      if(graph[x * size[1] * size[2] + y * size[2] + z])
      {
        return false;
      }
    }
  }
  return true;
}

bool clearUpY(Box* box, std::vector< Node*> graph, Image3DType::SizeType size)
{
  if(!pointInImage(box->pt1, size) || !pointInImage(box->pt2, size))
  {
    return false;
  }
  int y = box->pt2[2] + 1;
  for(int x = box->pt1[0]; x <= box->pt2[0]; x++)
  {
    for(int z = box->pt2[2]; z >= box->pt1[2]; z++)
    {
      if(graph[x * size[1] * size[2] + y * size[2] + z])
      {
        return false;
      }
    }
  }
  return true;
}

Eigen::Vector3d randFilledPoint(std::vector< Node* > graph)
{
  Node* node;
  do
  {
    int idx = rand() % graph.size();
    node = graph[idx];
  }while(!node);
  Eigen::Vector3d rv;
  rv(0) = (*node->index)[0];
  rv(1) = (*node->index)[1];
  rv(2) = (*node->index)[2];
  return rv;
}

Eigen::Vector3d randVector(double maxMagnitude)
{
  Eigen::Vector3d rv;
  int total = (int) std::pow(std::floor(maxMagnitude), 2);
  for(int i = 0; i < 3; i++)
  {
    rv(i) = (rand() % total);
    total = total - rv(i);
    int randsign = ( rand() % 4) - 2;
    rv(i) = randsign * std::sqrt(rv(i));
  }
  return rv;
}

Eigen::Vector3d randPointInRadius(Eigen::Vector3d pt, double radius)
{
  Eigen::Vector3d v = randVector(radius);
  return v + pt;
}

void testRandCylinder(BronchialSegmentation* detector, std::vector< Node*> graph, double radius)
{
  Eigen::Vector3d filled = randFilledPoint(graph);
  Eigen::Vector3d midpoint = randPointInRadius(filled, radius);
  Eigen::Vector3d line = randVector(radius * 2);
  std::vector<Eigen::Vector3d> point1s;
  point1s.push_back(midpoint + line);
  std::vector<Eigen::Vector3d> point2s;
  point2s.push_back(midpoint - line);
  std::vector<double> radii;
  radii.push_back(radius);
  std::vector<int> indices;
  bool result = detector->inCollision(point1s, point2s, radii, indices);
  if(result)
  {
    std::cout << "Cylinder is in collision (success)" << std::endl;
  } else
  {
    std::cout << "Cylinder is clear (faliure)" << std::endl;
  }
}

std::vector<Box*> randPath(std::vector<Node*> graph, Image3DType::SizeType size, int length)
{
  Box* start = new Box;
  start->pt1.push_back(270);
  start->pt1.push_back(300);
  start->pt1.push_back(387);
  start->pt2.push_back(276);
  start->pt2.push_back(306);
  start->pt2.push_back(377);
  std::vector< Box* > path;
  path.push_back(start);
  bool movePossible = true;
  int counter = 0;
  while (movePossible && counter < length) {
    //std::cout << path.back()->pt1[0] << " " << path.back()->pt1[1] << " "  << path.back()->pt1[2] << " "  <<
      //            path.back()->pt2[0] << " "  << path.back()->pt2[1] << " "  << path.back()->pt2[2] << " "  <<
        //          std::endl;
    std::vector< Box* > clearMoves;
    if(clearDownX(path.back(), graph, size))
    {
      clearMoves.push_back(moveDownX(path.back()));
    }
    if(clearDownY(path.back(), graph, size))
    {
      clearMoves.push_back(moveDownY(path.back()));
    }
    if(clearDownZ(path.back(), graph, size))
    {
      clearMoves.push_back(moveDownZ(path.back()));
    }
    if(clearUpX(path.back(), graph, size))
    {
      clearMoves.push_back(moveUpX(path.back()));
    }
    if(clearUpY(path.back(), graph, size))
    {
      clearMoves.push_back(moveUpY(path.back()));
    }
    if(clearMoves.size() == 0)
    {
      movePossible = false;
    } else
    {
      int move = rand() % (clearMoves.size() - 1);
      path.push_back(clearMoves[move]);
    }
    counter++;
  }
  return path;
}

void testRandPath(BronchialSegmentation* detector, std::vector< Node*> graph, int length)
{
  Image3DType::SizeType size;
  size[0] = 512;
  size[1] = 512;
  size[2] = 388;
  std::vector<Box*> path = randPath(graph, size, length);
  std::vector<Eigen::Vector3d> point1s;
  std::vector<Eigen::Vector3d> point2s;
  std::vector<double> radii;
  std::vector<int> indices;
  for(std::vector<Box*>::size_type i = 0; i != path.size(); i++)
  {
    Eigen::Vector3d pt1;
    Eigen::Vector3d pt2;
    pt1(0) = (path[i]->pt1[0] + path[i]->pt2[0])/2;
    pt1(1) = (path[i]->pt1[1] + path[i]->pt2[1])/2;
    pt1(2) = path[i]->pt1[2];
    pt2(0) = (path[i]->pt1[0] + path[i]->pt2[0])/2;
    pt2(1) = (path[i]->pt1[1] + path[i]->pt2[1])/2;
    pt2(2) = path[i]->pt2[2];
    point1s.push_back(pt1);
    point2s.push_back(pt2);
    radii.push_back(std::abs(path[i]->pt1[0] - path[i]->pt2[0]));
  }
  bool result = detector->inCollision(point1s, point2s, radii, indices);
  if(!result)
  {
    std::cout << "Path is clear (success)" << std::endl;
  } else
  {
    std::cout << "Path is not clear (failure)" << std::endl;
  }
}

void visualizePath(std::vector<Box*> path)
{
  ReaderType::Pointer reader;
	ImageIOType::Pointer dicomIO;
	Image3DType::SizeType size;
  std::cout << "Creating name generator" << std::endl;
	NamesGeneratorType::Pointer nameGenerator = NamesGeneratorType::New();
	nameGenerator->SetDirectory("out");

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

  for(std::vector< Box* >::size_type i = 0; i != path.size(); i++)
  {
    outputBox(path[i], image);
  }

  itksys::SystemTools::MakeDirectory("visualization");

		// generate the file names
		OutputNamesGeneratorType::Pointer outputNames = OutputNamesGeneratorType::New();
		std::string seriesFormat = std::string("visualization") + "/image-%05d.dcm";
		outputNames->SetSeriesFormat(seriesFormat.c_str());
		outputNames->SetStartIndex(1);
		outputNames->SetEndIndex(size[2]);

		// create series writer
		reader->Update();

		SeriesWriterType::Pointer seriesWriter = SeriesWriterType::New();
		seriesWriter->SetInput(image);
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
