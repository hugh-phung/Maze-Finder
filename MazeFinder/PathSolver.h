#ifndef COSC_ASSIGN_ONE_PATHSOLVER
#define COSC_ASSIGN_ONE_PATHSOLVER 

#include "Node.h"
#include "NodeList.h"
#include "Types.h"
#include <string>

class PathSolver{
public:

    // Constructor/Destructor
    PathSolver();
    ~PathSolver();

    // Execute forward search algorithm
    // To be implemented for Milestone 2
    void forwardSearch(Env env);

    // Get a DEEP COPY of the explored NodeList in forward search
    // To be implemented for Milestone 2
    NodeList* getNodesExplored();

    // Execute backtracking and Get a DEEP COPY of the path the 
    // robot should travel
    // To be implemented for Milestone 3
    NodeList* getPath(Env env);


    void setRowsCols(int rows, int cols);


private:

    // Nodes explored in forward search algorithm
    NodeList* nodesExplored;

    // Potential nodes
    NodeList* openList;

    // Nodes to be returned after back tracking
    NodeList* newNodesExplored;

    // Max array size to set lists
    int maxArraySize;
    int rows;
    int cols;

    // Returns node with selected character
    Node* getNodeWithChar(Env env, char character);

     // This method returns the node with the smallest estimated distance
    Node* getNodeWithSmallestDist(NodeList* nodeList, Node* goalNode);

    // Check if node already exists in given list 
    bool duplicateExists(Node* node, NodeList* nodeList);
    // Check if node already exists in given list including distance
    bool duplicateExistsWithDist(Node* node, NodeList* nodeList);


    // This contains the list-adding code for checkEnvForwardSearch(). Checks for duplicates
    void addIfNoDuplicatesForwardSearch(Node* newPotentialNode);

    // This checks the surrounding nodes of the selected one and adds it to the open list
    void checkEnvForwardSearch(Env env, Node* node);

    // This contains the list-adding code for checkEnvBackTrack(). Checks for duplicates
    void addIfNoDuplicatesBackTrack(Node* newPotentialNode, bool* alreadyAdded);

    // This method checks the closed list for surrounding nodes in the backtracking algorithm
    void checkEnvBackTrack(Env env, Node* currNode);

    // Checks surrounding direction of node to see if it isn't a wall
    bool checkDirection(std::string direction, Env env, Node* currNode);
  


};




#endif //COSC_ASSIGN_ONE_PATHSOLVER