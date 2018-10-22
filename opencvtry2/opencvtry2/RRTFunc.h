#pragma once
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

using namespace std;
using namespace cv;
#ifndef RRTFUNC_H
#define RRTFUNC_H
//Function Prototypes
Mat loadImage(const char* file);
void DisplayImage(Mat img, string winname);
list<node> createNodeList(Mat img, list<node>::iterator it);
void DisplayList(list<node> NodeList);
node setStartPoint(Mat img);
node setEndPoint(Mat img);
double getEuclideanDistance(node final, node initial);
bool isCollision(Mat img, node start, node end);

node getNodeAt(Mat img, int x, int y, int id = -1);
bool contains(list<node> Nodelist, node n);
list<node> getNeighbors(node n, list<node> nodeList);
node findClosest(node tar, list<node> NodeList);
Point extendBranch(int delta, node start, node end, Mat img);
bool isObstacle(Vec3b col);
list<node> Returnpath(list<node> tree);
void drawPath(Mat img, list<node> nodeList, string winname, Vec3b color);
list<node> RRT(node start, node end, int rows, int cols, Mat img, string winname);

Mat sateliteToCSpace(Mat img, string imwin);
bool isBlue(Vec3b col);
Point getCoordFromMouse(Mat img, string winname);
list<node> MidPointAverageOptimization(list<node> nodeList, Mat img);
double CubicBezierPoint(int p0, int c1, int c2, int p3, double t);
void BezierPathSmooth(list<node> nodeList, Mat img);
void plotBezier(Mat img, Point parray[], Vec3b col);

double getEuclideanDistance(Point final, Point initial);
list<Point> getObstacleList(Mat img);
Point findClosest(list<Point> pointList, Point tar);
bool isEdge(Point p, Mat img);
double getPathCost(Mat img, list<node> nodeList, double safety, node end);
list<node> alternativeRoute(list<node> prevPath, int delta, Mat img);
int getBranchLength();
list<node> removeCorners(list<node> prevPath, Mat img);
#endif