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




//Global Variables
Vec3b green = { 0,255,0 };
Vec3b white = { 255,255,255 };
Vec3b black = {0,0,0};
Vec3b blue =  { 255,0,0 };
Vec3b red =   { 0,0,255 };
Vec3b yellow = { 0,255,255 };
int sx,sy;
int ex, ey;
int ID;
int k = 100;
int t = 1000; //time in ms
int delta = 20;
double PI = 3.14159265358979323846;
Mat disp;
Point mouseClick;
string imwindow = "imagewindow";
string dispwindow = "dispwindow";
int failedNodes = 0;
clock_t Timestart;
bool timeout;
int main()
{
//______________________________________________________
	//make 2 pictures, one for drawing, one for checking
	const char* file = "satelite.png";
	disp = loadImage(file);	
	//Mat img = disp.clone();
	Mat img = sateliteToCSpace(disp,imwindow);
	//sample from img, write to disp
	int cols = img.cols;
	int rows = img.rows;
	//set start and finish point
	node start = setStartPoint(img);
	DisplayImage(disp, imwindow);
	waitKey(500);
	node end = setEndPoint(img);
	DisplayImage(disp, imwindow);
	// set length of branch
	delta = getBranchLength();
	cout << "Press Enter to start"<<endl;
	cin.ignore();
	//________________________
	//************************
	//ALGORYTHM IMPLEMENTATION
	//************************
	//_______________________

	//Starting Points 
	
	timeout = false;
	Timestart = clock();

	//Path Search
	list<node> tree = RRT(start, end, rows, cols, img, imwindow);
	double time = static_cast<double>(clock() - Timestart)/CLOCKS_PER_SEC;
	if (!timeout)
	{
		printf("Found solution in %.2fs\nFailed Nodes: %d\n Successfull Nodes %d\n", time, failedNodes, tree.size());
	}
	else {
		printf("Failed to Find Solution. Timout in %.2fs\n", time);
	}
	
	//Path Generation
	Timestart = clock();
	list<node> path = Returnpath(tree);
	time = static_cast<double>(clock() - Timestart) / CLOCKS_PER_SEC;
	printf("Returned Path in %.2fs\n", time);
	drawPath(disp, path, dispwindow, red);
	imwrite("C:\\Users\\user\\Documents\\University\\4th Year\\EEE4022S\\Write up\\Final\\template\\TestResults\\bezierpath_k_"+to_string(delta)+".jpg", disp);
	DisplayImage(disp, dispwindow);
	//double cost = getPathCost(img, path, 20, end);
	//printf("Path cost: %.2fs\n", cost);

	//Optmization
	Vec3b col[] = {yellow,green,blue,Vec3b(255,0,255),Vec3b(0,125,255),Vec3b(10,180,59)};
	list<node> newPath;
	
	/*for (int i = 0; i < 6; i++)
	{
		newPath = alternativeRoute(path, delta, img); 
		double newcost = getPathCost(img, newPath, 20, end);
		if (newcost < cost)
		{
			path = newPath;
		}
		printf("new Path cost: %.2fs\n Contains %d nodes\n", newcost, newPath.size());
			
	}
	newPath = removeCorners(newPath, img);*/
	
	//Smoothing

	//Midpointn average
	//for (int i = 0; i < 10; i++)
	//{
	//	newPath = MidPointAverageOptimization(path, img);
	//	drawPath(disp, newPath, dispwindow, col[i]);
	//	path = newPath;

	//}
	//imwrite("C:\\Users\\user\\Documents\\University\\4th Year\\EEE4022S\\Write up\\Final\\template\\TestResults\\mpaitr_k" + to_string(delta) + ".jpg", disp);
	disp = img.clone();
	BezierPathSmooth(path, img);
	//imwrite("C:\\Users\\user\\Documents\\University\\4th Year\\EEE4022S\\Write up\\Final\\template\\TestResults\\bezierfinal_k" + to_string(delta) + ".jpg", disp);
	DisplayImage(disp, dispwindow);
	waitKey(0);
	cin.ignore();
	return(0);

}
static void onMouse(int event, int x, int y, int flags, void* ptr)
{
	Point *p = (Point*)ptr;
	if (event == EVENT_LBUTTONDOWN)
	{
		p->x = x;
		p->y = y;
	}
}

node setStartPoint(Mat img)
{
	cout << "Please Select a Starting Point x,y : ";
	Point p = getCoordFromMouse(img, imwindow);
	if (((p.x < img.cols) && (p.x > 0)) && ((p.y < img.rows) && (p.y > 0)))
	{
		cout << "Starting at: (" <<p.x <<","<< p.y << ")\n";
		circle(disp, p, 3, green, FILLED);
		return node((int)'S', p.x, p.y, 0); //S denotes Start
	}
	else
	{

		cout << "Error: Coordinates do not exist please try again: \n";
		setStartPoint(img);
		return node();
	}
}

node setEndPoint(Mat img)
{

	cout << "Please Select an End point x,y : ";
	Point p = getCoordFromMouse(img, imwindow);
	if (((p.x < img.cols)&& (p.x > 0)) && ((p.y < img.rows) && (p.y > 0)))
	{
		cout << "Final Destination: (" << p.x << "," << p.y << ")\n";
		circle(disp,p,3,red,FILLED);
		node n = node((int)'T', p.x, p.y, -1);
		return n; // 'T' denotes target.end node has infinite cost
	}
	else
	{
		cout << "Error: Coordinates do not exist please try again: \n";
		setEndPoint(img);
		return node(0, 0, 0, 0);
	}
}

node getNodeAt(Mat img, int x, int y, int id)
{
	//get color at point
	node temp;
	Vec3b color = img.at<Vec3b>(Point(x, y));
	//assign cost
	uint32_t h = 500;
	if (color == black)
	{
		h = -1;
	}
	//check if node ID is not reserved
	if (id == -1)
	{
		if (ID == (int)'S')
		{
			ID += 2;
		}
	if (ID == (int)'T')
	{
		ID++;
	}
	temp = node(ID, x, y, h);
	ID++;
	}
	else
	{
	 temp = node(id, x, y, h);
	}

	return temp;
	
}

bool isCollision(Mat img,node start,node end)
{
	LineIterator it(img, Point(start.getX(), start.getY()), Point(end.getX(), end.getY()));
	
	for (int i = 1; i < it.count; i++) //project along a line
	{
		//get point along line
		
		Point p = it.pos();
		//check color
	
		
		if (isObstacle(img.at<Vec3b>(p)))
		{
			cout << "collision! Discarding point...\n";
			disp.at<Vec3b>(p) = yellow;
			return true;
		}
		disp.at<Vec3b>(p) = yellow;
		it.operator++();
	}
	return false;

}

Point getCoordFromMouse(Mat img, string winname)
{
	namedWindow(winname, 0);
	resizeWindow(winname, Size(img.cols, img.rows));

	//set the callback function for any mouse event
	Point p = Point(-1,-1);
	setMouseCallback(winname, onMouse, &p);
	imshow(winname, img);
	while (1)
	{
		if ((p.x > 0) && (p.y > 0))
		{
			Vec3b col = img.at<Vec3b>(p);
			if (!isObstacle(col))
			{
				break;
			}
			else
			{
				cout << "Error! can't select an obstacle as a waypoint. Please try again" << endl;
				p = Point(-1, -1);
			}
		}
		waitKey(1);
	}
	return p;
}


//Beziere Functions
void BezierPathSmooth(list<node> nodeList, Mat img)
{
	list<node>::iterator it = nodeList.begin();

	int rem = nodeList.size() % 4;
	int length = nodeList.size() / 4;

	if (length > 4)
	{
		Point parray[4];
		//for single case
		int j = 0;
		while ((nodeList.size() - j) > rem +1 )
		{
			if (j > 1)
			{
				it--;
			}
			for (int i = 0; i < 4; i++)
			{
				
				
				node temp = *it;
				parray[i] = Point(temp.getX(), temp.getY());
				it++;
				j++;
			}
			if(j>4)
			{ 
			it++;
			}
			
			double t = 0;
			Point prev = Point(parray[0].x, parray[0].y);
			while (t <= 1)
			{
				//calculate delta 
				double delta = 1 / abs(static_cast<double>((parray[0].x - parray[3].x)));
				int y = CubicBezierPoint(parray[0].y, parray[1].y, parray[2].y, parray[3].y, t);
				int x = CubicBezierPoint(parray[0].x, parray[1].x, parray[2].x, parray[3].x, t);
				//plot
				line(disp, Point(x, y), prev, Vec3b(0, 125, 255), 3);
				DisplayImage(disp, dispwindow);
				//increment
				t += delta;
				prev = Point(x, y);
			}
		}

	}
	//determine remainder
	//TO:DO send plot to new function plotBezier
	// add rem
	//determine remainder size
	//get start point

	if (rem > 0)
	{

		node temp = *(it);
		Point c1 = Point(0, 0);
		Point c2 = Point(0, 0);
		Point v0 = Point(temp.getX(), temp.getY());

		if (rem > 1)
		{
			temp = *(++it);
			c1.x = temp.getX();
			c1.y = temp.getY();
			//for 3 points, use quadratic bezier
			if (rem > 2)
			{
				advance(it, 1);
				temp = *it;
				c2.x = temp.getX();
				c2.y = temp.getY();
			}
			advance(it, 1);
			temp = *it;
			Point v3 = Point(temp.getX(), temp.getY());
			Point parray[] = { v0,c1,c2,v3 };
			plotBezier(disp, parray, Vec3b(0, 125, 255));
		}
	}
	

}


//get new path 

list<node> alternativeRoute(list<node> prevPath, int delta,Mat img)
{
	//iterator for old path
	list<node>::iterator prevIt = prevPath.begin();
	//initialise new path list
	list<node> newPath;
	list<node>::iterator newIt = newPath.begin(); 
	
	// first and last node stay static 
	// and work backwards get the first node and advance the list
	node Prev = *prevIt;
	advance(prevIt, 1);
	newPath.insert(newIt, Prev);

	random_device rd;
	mt19937 gen(rd());
	uniform_int_distribution<> rannum(-delta, delta);
	for ( int i = 1; i < prevPath.size()-1; i++)
	{
		// take a sample around a node
		node current = *prevIt;
		//generate random number
	
		bool isClear = false;
		//5 second timeout
		clock_t start = clock();
		while(static_cast<int>((clock()-start))/CLOCKS_PER_SEC < 5)//if no other node availiable then time out 
		{
			//generate random number
			int x = current.getX() + rannum(gen);
			int y = current.getY() + rannum(gen);
			//safe check bool
			bool xsafe = (x > 0) && (x < img.cols);
			bool ysafe = (y > 0) && (y < img.rows);
			if (xsafe&&ysafe) 
			{
				if (!isObstacle(img.at<Vec3b>(Point(x, y))))
				{
					node temp = getNodeAt(img, x, y, current.getNodeID());
					//check if collision free
					if (!isCollision(img, Prev, temp))
					{
						//assign node and break
						current = temp;
						break;
					}
				}
				else
				{
					x = current.getX();
						y = current.getY();
				}
			}
		}
		current.setID(Prev.getNodeID());
		newPath.insert(newIt, current);
		advance(prevIt, 1);
		Prev = current;

	}
	newPath.insert(newIt, prevPath.back());
	
	
	return newPath;
}
//algorythm function
list<node> RRT(node start, node end, int rows , int cols, Mat img, string winname)
{
	list<node> tree;     //empty list of paths
	list<node>::iterator it = tree.begin();
	ID = 0;
	//start at begining node,
	tree.insert(it, start);
	node current = start;
	//flag to indicate path has been found
	bool found = false;
	while (found == false) //loop for a specified amount of time
	{
		//time out condition
		//if ((static_cast<double>(clock() - Timestart) / CLOCKS_PER_SEC) >= 300)
		//{
		//	timeout = true;
		//	cout << "Timeout could not find path" << endl;
		//	return tree;
		//}

		//find closest node to end

		node temp = findClosest(end, tree);
		if ((getEuclideanDistance(temp, end) < delta)||(!isCollision(img, temp, end)))
		{
			//update flag
			found = true;
			temp.setPrev_Node(current.getNodeID());
			current = temp;
			tree.insert(it, current);
			line(disp, Point(current.getX(), current.getY()), Point(end.getX(), end.getY()), blue);
			circle(disp, Point(end.getX(), end.getY()), 3, green, FILLED);//replace with elipse.at<Vec3b>(Point(end.getX(), end.getY())) = black;
			end.setPrev_Node(current.getNodeID());
			tree.insert(it, end);
			DisplayImage(disp,dispwindow);
		}
		else
		{
			int x = rand() % (cols - 1);  //generate node off of random position in the space
			int y = rand() % (rows - 1);
			try
			{

				node rand = getNodeAt(img, x, y);

				//draw a tcircle to indicate possible node
				//obstacle check

				if (!contains(tree, rand))
				{
					//find closest node in tree 
					if (tree.size() > 1)
					{

						current = findClosest(rand, tree);
					}

					//check if distance < delta
					if (getEuclideanDistance(rand, current) > delta)
					{

						// extend branch of current by delta and set as rand

						try {
							Point P = extendBranch(delta, current, rand, img);
							//draw new coordinates
							circle(disp, P, 3, Vec3b(255, 255, 0), FILLED);
							node temp = getNodeAt(img, P.x, P.y);
							//check for collision
							//is point obstacle
							if (temp.isObstacle())
							{
								cout << "new point is an obstacle" << endl;
								failedNodes++;
								continue;
							}
							//does a collision occur along the edge
							if (isCollision(img, current, temp))
							{
								cout << "Collision Detected. Obstical between current and Increment." << endl;
								failedNodes++;
								continue; //continue to next iteration of the loop

							}
							else
							{
								rand = temp;
							}

						}
						catch (std::exception &e)
						{
							(void)e;
							cout << "Error! x or y are out of bounds" << endl;
							failedNodes++;
							continue;
							//rand doesnt change
						}
					}
					else {
						//check point
						circle(disp, Point(rand.getX(), rand.getY()), 3, Vec3b(255, 0, 255), FILLED);
						if ((rand.isObstacle()) || (isCollision(img, current, rand)))
						{
							cout << "Collision Detected Obstical between current and random node" << endl;
							failedNodes++;
							continue;
						}

					}
					//check if a collisiong occurs
					line(disp, Point(current.getX(), current.getY()), Point(rand.getX(), rand.getY()), blue);
					circle(disp, Point(rand.getX(), rand.getY()), 3, green, FILLED);
					rand.setPrev_Node(current.getNodeID());
					tree.insert(it, rand);
					//set this node to current node
					current = rand;
					DisplayImage(disp, dispwindow);
				}

			}
			catch (const std::exception & e)
			{

				cout << "Error creating Node: " << e.what() << endl;
				failedNodes++;
			}
		}
		
	}
	return tree;
}

list<node> removeCorners(list<node> prevPath, Mat img)
{
	list<node> newPath;
	list<node>::iterator it = newPath.begin();
	list<node>::iterator previt = prevPath.begin();
	newPath.insert(it, prevPath.front());
	node prev = *previt;
	advance(previt, 1);

	int count = 0;
	while(prev.getNodeID() != 83)
	{
		//grab a node
		node current = *previt;
		 //temporary node
		 //for remaining length of list
		for (int i = 0; i < prevPath.size() -count; i++ )
		{
			//break if collision
			node temp = *previt;
			if (isCollision(img, prev, temp))
			{
				break;
			}
			advance(previt, 1);
			current = temp;
			count++;
			
		}
		//set reff to prev node and add to list
		current.setPrev_Node(prev.getNodeID());
		newPath.insert(it,current);
		prev = current;
	}

	return newPath;
}

Mat sateliteToCSpace(Mat img, string imwin)
{
	Mat temp = Mat(img.rows, img.cols, CV_8UC3);

	for (int i = 0; i < img.cols - 1; i++)
	{
		for (int j = 0; j < img.rows - 1; j++)
		{
			if (isBlue(img.at<Vec3b>(Point(i, j))))
			{
				temp.at<Vec3b>(Point(i, j)) = white;
			}
			else
			{
				temp.at<Vec3b>(Point(i, j)) = black;
			}

		}
	}
	return temp;
}

void plotBezier(Mat img, Point parray[],Vec3b col)
{
	double t = 0;
	Point prev = Point(parray[0].x, parray[0].y);
	while (t <= 1)
	{
		//calculate delta 
		double delta = 1 / abs(static_cast<double>((parray[0].x - parray[3].x)));
		int y = CubicBezierPoint(parray[0].y, parray[1].y, parray[2].y, parray[3].y, t);
		int x = CubicBezierPoint(parray[0].x, parray[1].x, parray[2].x, parray[3].x, t);
		//plot
		line(disp, Point(x, y), prev, col, 3);
		DisplayImage(disp, dispwindow);
		//increment
		t += delta;
		prev = Point(x, y);
	}
}