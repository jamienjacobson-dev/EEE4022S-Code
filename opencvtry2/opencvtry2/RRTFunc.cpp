#include<opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <list>
#include <stdio.h>
#include<stdlib.h> 
#include <math.h>
#include<random>
#include<iostream>
#include "node.h"
#include <time.h>
#include "RRTFunc.h"
using namespace std;
using namespace cv;

int getBranchLength()
{
	int branch;
	cout << "Enter branch length Delta (): ";
	char temp[100];
	cin >> temp;
	try
	{
		branch = atoi(temp);
	}
	catch (const std::exception&)
	{
		cout << "Error! Please Enter a valid number\n";
		getBranchLength();
	}
	return branch;
}
Mat loadImage(const char* file)
{
	ifstream f(file);
	if (f)
	{
		Mat img = imread(file);
		return img;
	}
	else
	{
		cout << "Error: File Not Found!";
		Mat M(2, 2, CV_8UC3, Scalar(255, 255, 255));
		return M;
	}
}

void DisplayImage(Mat img, string winname)
{
	namedWindow(winname, WINDOW_NORMAL);
	resizeWindow(winname, Size(img.cols, img.rows));
	imshow(winname, img);
	waitKey(1);
}

list<node> createNodeList(Mat img, list<node>::iterator it)
{
	try
	{
		int ID = 0;
		list<node> NodeList;
		it = NodeList.begin();
		for (int i = 0; i < img.rows; i++)
		{
			for (int j = 0; j < img.cols; j++)
			{

				//black tiles are obstacles
				Vec3b col = img.at<Vec3b>(Point(i, j));
				uint32_t h = 0;
				if ((col[0] == 0) && (col[1] == 0)&(col[2] == 0))
				{
					h = -1;
				}
				else if ((col[0] == 255) && (col[1] == 255)&(col[2] == 255))
				{
					h = 500; //base cost
				}
				else
				{
					h = 800;
				}
				node temp = node(ID, i, j, h); //a temporary cost
				NodeList.insert(it, temp);
				ID++; //new ID

			}
		}
		return NodeList;
	}
	catch (cv::Exception & e)
	{
		cout << e.msg;

	}
	return {};
}

void DisplayList(list<node> NodeList)
{
	for (node Node : NodeList)
	{
		cout << Node.to_String() + "\n";
	}
}

double getEuclideanDistance(node final, node initial)
{
	double dx2 = pow(final.getX() - initial.getX(), 2);
	double dy2 = pow(final.getY() - initial.getY(), 2);
	double ed2 = dx2 + dy2;
	return sqrt(ed2);
}

double getEuclideanDistance(Point final, Point initial)
{
	double dx2 = pow(final.x - initial.x, 2);
	double dy2 = pow(final.y - initial.y, 2);
	double ed2 = static_cast<double>(dx2 + dy2);
	return sqrt(ed2);
}

node getNode(list<node> nodeList, int id)
{
	node temp = node();
	for (node Node : nodeList)
	{
		if (Node.getNodeID() == id)
		{
			return Node;
		}
	}
	return temp;
}



list<node> getNeighbors(node n, list<node> nodeList)
{
	list<node> neigbours;
	list<node>::iterator in = neigbours.begin();
	for (node Node : nodeList)
	{
		//neigbours are either one up, one down, one left
		//safety: obstacle, out of bounds
		if ((abs(Node.getX() - n.getX()) == 1) || (abs(Node.getY() - n.getY()) == 1))
		{
			neigbours.insert(in, Node);
		}
	}

	return neigbours;
}

node findClosest(node tar, list<node> NodeList)
{
	int dist = -1;
	node closest = node(0, 0, 0, 0);
	for (node Node : NodeList)
	{
		int temp = static_cast<int>(getEuclideanDistance(tar, Node));
		if ((temp < dist) || (dist == -1))
		{
			dist = temp;
			closest = Node;
		}
	}
	return closest;
}

bool contains(list<node> Nodelist, node n)
{
	if (Nodelist.empty())
	{
		return false;
	}
	for (node Node : Nodelist)
	{
		if (Node.equals(n))
		{
			return true;
		}
	}
	return false;
}



Point extendBranch(int delta, node start, node end, Mat img)
{
	LineIterator it(img, Point(start.getX(), start.getY()), Point(end.getX(), end.getY()));
	Point P = Point(start.getX(), start.getY());
	for (int i = 0; i <= delta; i++)
	{
		P = it.pos();
		it.operator++();
	}
	return P;
}

list<node> Returnpath(list<node> tree)
{
	list<node> path;
	list<node>::iterator it = path.begin();
	list<node>::iterator treeIt = tree.end();
	//start at back
	node s = tree.back();
	path.push_back(s);
	while (s.getNodeID() != (int)'S')
	{
		//get node from prev refference
		node temp = getNode(tree, s.getPrevNode());
		path.push_back(temp);
		s = temp;
	}
	return path;

}

void drawPath(Mat img, list<node> nodeList, string winname, Vec3b color)
{

	//nodeList.pop_front();
	list<node>::iterator it = nodeList.begin();
	node prev = *it;
	advance(it, 1);
	node curr;
	while (curr.getNodeID() != 83)
	{
		curr = *it;
		//draw a line for path
		line(img, Point(prev.getX(), prev.getY()), Point(curr.getX(), curr.getY()), color, 2);
		//advance and safe 
		prev = curr;
		advance(it, 1);
	}

	curr = nodeList.back();
	line(img, Point(prev.getX(), prev.getY()), Point(curr.getX(), curr.getY()), color, 2);
}
bool isObstacle(Vec3b col)
{
	return (col[0] < 10) && (col[1] < 10) && (col[2] < 10 );
}

bool isBlue(Vec3b col)
{
	bool b = (col[0] > 98) && (col[0] < 250); //if fits in color threshold
	bool g = (col[1] > 50) && (col[1] < 180);
	bool r = (col[2] >= 35) && (col[2] < 150);

	return b && g&&r;
	//return  (abs(col[0]-col[1]) <15)&&(col[0]> col[2])&&(col[1]> col[2])&&(col[2] <60);
}



list<node> MidPointAverageOptimization(list<node> nodeList, Mat img)
{
	list<node> newPath;
	list<node>::iterator newIt = newPath.begin();

	list<node>::iterator prevIt = nodeList.begin();

	node Prev = *prevIt;
	//insert end poitn into list
	newPath.insert(newIt, Prev);
	prevIt++;
	int prevId = Prev.getNodeID();
	while ((prevIt) != nodeList.end())
	{
		
		node current = *prevIt;
		//find mid point
		Point mid = Point((Prev.getX() + current.getX()) / 2, (Prev.getY() + current.getY()) / 2);
		node Temp;
		//create a node at that point
		if (current.getNodeID() == 83 )//prevent the end point from moving
		{
			Temp = getNodeAt(img, mid.x, mid.y);
		}
		else { 
			Temp = getNodeAt(img, mid.x, mid.y, current.getNodeID()); 
		} 
		
		//add to list
		Temp.setPrev_Node(prevId);
		newPath.insert(newIt, Temp);

		//set previous nodes
		Prev = current;
		prevId = Temp.getNodeID();
		prevIt++;
	}
	//insert end point
	newPath.insert(newIt, Prev);
	return newPath;
}
//Beziere Functions

list<Point> getObstacleList(Mat img)
{
	//get edges
	list<Point> obstacles;
	list<Point>::iterator it = obstacles.begin();
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			if (isObstacle(img.at<Vec3b>(Point(j, i))))
			{
				// do an edge check
				Point p = Point(j, i);
				if (isEdge(p, img))
				{
					obstacles.insert(it, p);

				}
				//getEdge
			}
		}
	}
	return obstacles;
}

bool isEdge(Point p, Mat img)
{
	//check if on boundary

	for (int j = -1; j <= 1; j++)
	{
		for (int i = -1; i <= 1; i++)
		{
			int x = p.x + i;
			int y = p.y + j;
			if (((x >= 0) && (y >= 0)) && ((x < img.cols) && (y < img.rows)))
			{
				Vec3b col = img.at<Vec3b>(Point(x, y));

				bool isWhite = (col[0] > 200) && (col[1] > 200) && (col[2] > 200);
				if (isWhite)
				{
					return true;
				}
			}
		}
	}
	return false;
}
Point findClosest(list<Point> pointList, Point tar)
{
	Point closest = pointList.front();
	double dist = getEuclideanDistance(tar, closest);
	for (Point point : pointList)
	{
		double temp = getEuclideanDistance(point, tar);
		if (temp < dist)
		{
			dist = temp;
			closest = point;
		}

	}
	return closest;
}

double CubicBezierPoint(int p0, int c1, int c2, int p3, double t)
{

	double s = 1 - t;
	double q_t = p0 * pow(s, 3) + 3 * c1 * t*pow(s, 2) + 3 * c2 * pow(t, 2)*s + p3 * pow(t, 3);
	return q_t;
}

double getPathCost(Mat img, list<node> nodeList, double safety, node end)
{
	double fcost = 0;
	list<Point> obsList = getObstacleList(img);
	node Prev = nodeList.front();
	for (node Node : nodeList)
	{
		//declare h and g costs
		double h, g;
		//step 1 determine the tile cost i.e proportionality cost
		Point nPoint = Point(Node.getX(), Node.getY());
		Point obs = findClosest(obsList, nPoint);

		double dist = getEuclideanDistance(obs, nPoint);
		double k = dist / safety;

		if (k < 1)
		{
			h = 500 + k * 9500;
		}
		else {
			h = 500;
		}
		h += getEuclideanDistance(Prev, Node);

		//step 2: determine g cost

		g = getEuclideanDistance(Node, end);
		//add to f cost
		fcost += g + h;

		Prev = Node;
	}
	return fcost;
}

//get new path 


