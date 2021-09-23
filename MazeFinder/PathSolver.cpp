#include "PathSolver.h"
#include <iostream>

#define UP "UP"
#define DOWN "DOWN"
#define LEFT "LEFT"
#define RIGHT "RIGHT"

PathSolver::PathSolver(){
    maxArraySize = 0;
    openList = new NodeList();
    nodesExplored = new NodeList();
    newNodesExplored = new NodeList();
}

PathSolver::~PathSolver(){
    delete openList;
    delete nodesExplored;
    delete newNodesExplored;
}

void PathSolver::setRowsCols(int rows, int cols)
{
    this->rows = rows;
    this->cols = cols;
    this->maxArraySize = rows * cols * 4;
}

void PathSolver::forwardSearch(Env env){

    // Set the max array size of the arrays using the value from main
    openList->setMaxArraySize(maxArraySize);
    nodesExplored->setMaxArraySize(maxArraySize);
    newNodesExplored->setMaxArraySize(maxArraySize);


    Node* startNode = getNodeWithChar(env, SYMBOL_START);
    Node* goalNode = getNodeWithChar(env, SYMBOL_GOAL);



    // Initially contains node with Start symbol
    openList->addElement(startNode);
    Node* currNode = openList->getNode(0);

    // Repeat loop until reaches goal node
    while (!((currNode->getCol() == goalNode->getCol()) && (currNode->getRow() == goalNode->getRow())))
    {
        // Selecting node from list with smallest distance
        currNode = getNodeWithSmallestDist(openList, goalNode);
        
        // Check the surrounding nodes of the selected node
        checkEnvForwardSearch(env, currNode);

        // Adding selected node p to the explored list
        if (!duplicateExists(currNode, nodesExplored))
        {
            nodesExplored->addElement(currNode); 
        }
    }
    delete startNode;
    startNode = nullptr;
    delete goalNode;
    goalNode = nullptr;
}

NodeList* PathSolver::getNodesExplored(){

    NodeList* nodesExploredCopy = new NodeList(*nodesExplored);

    return nodesExploredCopy;
}


// Back Tracking Algorithm:
// Finds the goal node by selecting the last element in the list
// Then works its way back finding a surrounding node with one less distance

// Pseudocode goes as follow:
// Input env - the environment
// Input goalNode - goal location node
// Let newNodeExplored be the list to store the shortest path back to node with 0 distance (the start node) in nodesExplored 
// for each node in newNodesExplored:
//      Select the node 'selectedNode' from newNodesExplored
//      Find the surrounding node that has one less distance and add it to newNodesExplored
// end
// Reverse the order of newNodesExplored
NodeList* PathSolver::getPath(Env env){

    // Start off with the goal node
    Node* goalNode = nodesExplored->getNode(nodesExplored->getLength()-1);
    newNodesExplored->addElement(goalNode);

    for (int i = 0; i < newNodesExplored->getLength(); ++i)
    {
        // This method will return a deep copy of the explored list by backtracking.
        Node* selectedNode = newNodesExplored->getNode(i);
        checkEnvBackTrack(env, selectedNode);
    }

    // Reverses list so it is returned in order.
    newNodesExplored->reverseList();

    NodeList* newNodesExploredCopy = new NodeList(*newNodesExplored);

    return newNodesExploredCopy;
}

// -----------------------------------------------------------------------------------

// Creates and returns the node with the given character.
Node* PathSolver::getNodeWithChar(Env env, char character)
{
    Node* destNode = nullptr;
    // std::cout<< rows << " " << cols << std::endl;
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            if (env[i][j] == character)
            {
                destNode = new Node(i, j, 0);
            }
        }
    }
    return destNode;
}

// This method returns the node with the smallest estimated distance.
Node* PathSolver::getNodeWithSmallestDist(NodeList* nodeList, Node* goalNode)
{
    Node* smallestDistNode = nodeList->getNode(0);
    Node* tempNode = nullptr;

    // Traverses the open list
    for (int i = 0; i < nodeList->getLength(); ++i)
    {
        tempNode = nodeList->getNode(i);

        // Looks for a node than has not been covered.
        if (!duplicateExists(tempNode, nodesExplored))
        {
            // If a node contains a smaller distance set that as the smallest node.
            if (smallestDistNode->getEstimatedDist2Goal(goalNode) >= tempNode->getEstimatedDist2Goal(goalNode))
            {
                smallestDistNode = tempNode;
            }
            // If a node has no other option to to move to a space at the expense of a larger distance.
            else if(duplicateExists(smallestDistNode, nodesExplored))
            {
                smallestDistNode = tempNode;
            }
        }
    }

    return smallestDistNode;
}

// Checks if the node already exists within the given list.
bool PathSolver::duplicateExists(Node* node, NodeList* nodeList)
{
    bool flag = false;
    for (int i = 0; i < nodeList->getLength(); ++i)
    { 
        // Compares each element of the node with the column and row.
        if (nodeList->getNode(i) != nullptr && nodeList->getNode(i)->getCol() == node->getCol() && nodeList->getNode(i)->getRow() == node->getRow())
        {
            flag = true;
        }
        
    }
    return flag;
}

// Checks if the node already exists within the given list, this time also comparing the distance.
bool PathSolver::duplicateExistsWithDist(Node* node, NodeList* nodeList)
{
    bool flag = false;
    for (int i = 0; i < nodeList->getLength(); ++i)
    {
        // Compares each element of the node with the column, row and distance.
        if (nodeList->getNode(i) != nullptr && nodeList->getNode(i)->getCol() == node->getCol() && nodeList->getNode(i)->getRow() == node->getRow() && nodeList->getNode(i)->getDistanceTraveled() == node->getDistanceTraveled())
        {
            flag = true;
        }
        
    }
    return flag;
}

// If the new node exists in the open list and hasn't already been added in the closed list, add it.
void PathSolver::addIfNoDuplicatesForwardSearch(Node* newPotentialNode)
{
    if (!duplicateExists(newPotentialNode, openList) && !duplicateExists(newPotentialNode, nodesExplored))
    {
        newPotentialNode->setDistanceTraveled(newPotentialNode->getDistanceTraveled()+1);
        openList->addElement(newPotentialNode);
    }
    delete newPotentialNode;
    newPotentialNode = nullptr;
}

// Checks surrounding area , UP, DOWN, LEFT, RIGHT of selected node to see if it's available.
// If it is, it adds that new node to the open list.
void PathSolver::checkEnvForwardSearch(Env env, Node* currNode)
{
    Node* newPotentialNode = nullptr;

    // UP of node
    if (checkDirection(UP, env, currNode) && !duplicateExists(currNode, nodesExplored))
    {
        newPotentialNode = new Node(currNode->getRow()-1, currNode->getCol(), currNode->getDistanceTraveled());
        addIfNoDuplicatesForwardSearch(newPotentialNode);
    }

    // DOWN of node
    if (checkDirection(DOWN, env, currNode) && !duplicateExists(currNode, nodesExplored))
    {
        newPotentialNode = new Node(currNode->getRow()+1, currNode->getCol(), currNode->getDistanceTraveled());
        addIfNoDuplicatesForwardSearch(newPotentialNode);
    }

    // LEFT of node
    if (checkDirection(LEFT, env, currNode) && !duplicateExists(currNode, nodesExplored))
    {
        newPotentialNode = new Node(currNode->getRow(), currNode->getCol()-1, currNode->getDistanceTraveled());
        addIfNoDuplicatesForwardSearch(newPotentialNode);
    }
    
    // RIGHT of node
    if (checkDirection(RIGHT, env, currNode) && !duplicateExists(currNode, nodesExplored))
    {
        newPotentialNode = new Node(currNode->getRow(), currNode->getCol()+1 , currNode->getDistanceTraveled());
        addIfNoDuplicatesForwardSearch(newPotentialNode);
    }

}

// If the new node exists in the explored list and hasn't already been added in the new back tracking list, add it.
// Also sets a boolean value to prevent more than one node at a time being added.
void PathSolver::addIfNoDuplicatesBackTrack(Node* exploredNode, bool* alreadyAdded)
{
    if (duplicateExistsWithDist(exploredNode, nodesExplored) && !duplicateExists(exploredNode, newNodesExplored))
    {   
        newNodesExplored->addElement(exploredNode);
        *alreadyAdded = true;

    }
    delete exploredNode;
    exploredNode = nullptr;   
}

// Main logic for back tracking algorithm.
// Searches for a node that exists within the explored list then returns it if it is one less distance.
// Fun fact: The solution to a major bug in this section came to me in a dream hahaha.
void PathSolver::checkEnvBackTrack(Env env, Node* currNode)
{
    Node* exploredNode = nullptr;
    bool alreadyAdded = false;

    // UP of node
    if (checkDirection(UP, env, currNode) && !alreadyAdded)
    {
        exploredNode = new Node(currNode->getRow()-1, currNode->getCol(), currNode->getDistanceTraveled()-1);
        addIfNoDuplicatesBackTrack(exploredNode, &alreadyAdded);
    }

    // DOWN of node
    if (checkDirection(DOWN, env, currNode) && !alreadyAdded)
    {

        exploredNode = new Node(currNode->getRow()+1, currNode->getCol(), currNode->getDistanceTraveled()-1);
        addIfNoDuplicatesBackTrack(exploredNode, &alreadyAdded);   
    }

    // LEFT of node
    if (checkDirection(LEFT, env, currNode) && !alreadyAdded)
    {
        exploredNode = new Node(currNode->getRow(), currNode->getCol()-1, currNode->getDistanceTraveled()-1);
        addIfNoDuplicatesBackTrack(exploredNode, &alreadyAdded);
    }

    // RIGHT of node
    if (checkDirection(RIGHT, env, currNode) && !alreadyAdded)
    {
        exploredNode = new Node(currNode->getRow(), currNode->getCol()+1, currNode->getDistanceTraveled()-1);
        addIfNoDuplicatesBackTrack(exploredNode, &alreadyAdded); 
    }
    
}

// Checks up, down, left and right of envrionment to see if there is a space.
bool PathSolver::checkDirection(std::string direction, Env env, Node* currNode)
{    
    bool flag = false;
    if (direction == UP)
    {
        if (env[currNode->getRow()-1][currNode->getCol()] != SYMBOL_WALL)
        {
            flag = true;
        }
    }

    else if (direction == DOWN)
    {
        if (env[currNode->getRow()+1][currNode->getCol()] != SYMBOL_WALL)
        {
            flag = true;
        }
    }

    else if (direction == LEFT)
    {
        if (env[currNode->getRow()][currNode->getCol()-1] != SYMBOL_WALL)
        {
            flag = true;
        }
    }

    else if (direction == RIGHT)
    {
        if (env[currNode->getRow()][currNode->getCol()+1] != SYMBOL_WALL)
        {
            flag = true;
        }
    }

    return flag;
}

