//
// Created by Jamie on 09/11/18.
//

#include <cstdint>
#include <string>
#ifndef CPP_CODE_NODE1_H
#define CPP_CODE_NODE1_H



using namespace std;

class node
{
	//Attributes
private:
	//node identifier
	int ID;
	//coordinates
	int x;
	int y;
	//previous node
	int prev_node;
	//cost
	uint32_t h; //tile cost
//________________________________________________________

public:

	node(int ID, int x, int y, int h); //create a node with Ref and coordinates (initial prev nodes to -1)
	node();

	int getX();        //returns coordinates in the form (x,y);

	int getY();

	int getPrevNode();

	int getNodeID();

	void setCoordinates(int x, int y);

	void setPrev_Node(int ID);

	void setHCost(uint32_t h);
	void setID(int ID);

	bool isObstacle();
	bool equals(node n);

	uint32_t getPathCost();

	string to_String();

	//returns -1 if obstacle, 0 if not
};
#endif CPP_CODE_NODE1_H