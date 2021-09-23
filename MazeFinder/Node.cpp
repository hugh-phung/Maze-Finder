#include "Node.h"
#include <iostream>


Node::Node(int row, int col, int dist_traveled)
{
    this->row = row;
    this->col = col;
    this->dist_traveled = dist_traveled;
}

Node::~Node(){
}

Node::Node(Node& other)
{
    this->row = other.getRow();
    this->col = other.getCol();
    this->dist_traveled = other.getDistanceTraveled();

}

int Node::getRow(){
    return row;
}

int Node::getCol(){
    return col;
}

int Node::getDistanceTraveled(){
    return dist_traveled;
}

void Node::setDistanceTraveled(int dist_traveled)
{
    this->dist_traveled = dist_traveled;
}

int Node::getEstimatedDist2Goal(Node* goal){
    // Estimated distance = dist_traveled + Manhattan distance from p to G

    int manhattan_dist = abs((col - goal->getCol())) + abs((row - goal->getRow()));
    int estimated_dist = dist_traveled + manhattan_dist;

    return estimated_dist;
}
    
//--------------------------------                             