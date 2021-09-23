/* 
* ********************************************************************************************************************
* Hugh Trung-Hieu Phung
* s3842508
* Advanced Programming Techniques 
* Assignment 1: Implementing a path planning algorithm

* The approach I have taken for the main algorithm involves a loop in which
* the surrounding area around a selected node is checked, up, down, left and right.
* If there isn't a wall symbol in the way, that free space is added to an open list.
* Once all directions are checked for the node, the one with the least distance is added to a closed list.
* This will contain the paths taken by the robot in the maze.
* MANY problems were encountered throughout, though some of the notable issues caused segmentation faults
* Oh my goodness were there a lot of segmentation faults.
* These were usually fixed by shortening array exploration to just the length of the array. 
* Another instance of a seg fault was caused by incorrect boolean logic in the main algorithm loop.
* Another major error that took days to fix was the back tracking algorithm in Milestone 3. When it came to involving
* distances and duplicates, it caused issues. And when that was fixed, dealing with an environment with
* loops caused even more problems. This was fixed by adding a simple boolean value to indicate whether a
* node had been added.
* For Milestone 4, I used the provided methods as a template for setting up the environment. 
* The method to create the environment gave me the most trouble. To expand the rows and cols, a default
* dimension size was given, and based on how big the input was those dimensions were sized accordingly.
* ********************************************************************************************************************
*/

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <string>

#include "Types.h"
#include "Node.h"
#include "NodeList.h"
#include "PathSolver.h"

// Helper test functions
void testNode();
void testNodeList();


// Read a environment from standard input.
void readEnvStdin(Env env, int* actualRows, int* actualCols);

// Print out a Environment to standard output with path.
// To be implemented for Milestone 3
void printEnvStdout(Env env, NodeList* solution, int rows, int cols);

void initialiseEnv(Env newEnv, int rows, int cols);
void copyEnv(Env oldEnv, Env newEnv, int rows, int cols);
void copyEnvBack(Env env, Env newEnv, int rows);
void resizeEnv(Env env, int rows_index, int cols_index, int* rows, int* cols);
void expandCols(int rows, int* cols, Env oldEnv);
void expandRows(int* rows, int cols, Env oldEnv);



int main(int argc, char** argv){
    // THESE ARE SOME EXAMPLE FUNCTIONS TO HELP TEST YOUR CODE
    // AS YOU WORK ON MILESTONE 2. YOU CAN UPDATE THEM YOURSELF
    // AS YOU GO ALONG.
    // COMMENT THESE OUT BEFORE YOU SUBMIT!!!
    // std::cout << "TESTING - COMMENT THE OUT TESTING BEFORE YOU SUBMIT!!!" << std::endl;
    // testNode();
    // testNodeList();
    // std::cout << "DONE TESTING" << std::endl << std::endl;

    // Load Environment 
    Env env = nullptr;

    // Setting DEFAULT values for dimensions
    int actualRows = ENV_DIM;
    int actualCols = ENV_DIM;

    // Initiating the dimensions of the 2D envrionment
    if (actualRows >= 0 && actualCols >= 0) {

        env = new char*[actualRows];

        for (int i = 0; i != actualRows; ++i) {
            env[i] = new char[actualCols];
        }
    }

    // Takes in cin input as the environment
    readEnvStdin(env, &actualRows, &actualCols);

    // Solve using forwardSearch
    // THIS WILL ONLY WORK IF YOU'VE FINISHED MILESTONE 2
    PathSolver* pathSolver = new PathSolver();
    pathSolver->setRowsCols(actualRows, actualCols);
    pathSolver->forwardSearch(env);


    NodeList* exploredPositions = nullptr;

    exploredPositions = pathSolver->getNodesExplored();

    // // // // Get the path
    // // // // THIS WILL ONLY WORK IF YOU'VE FINISHED MILESTONE 3

    NodeList* solution = pathSolver->getPath(env);

    printEnvStdout(env, solution, actualRows, actualCols);

    delete pathSolver;
    delete exploredPositions;
    delete solution;

}

void readEnvStdin(Env env, int* actualRows, int* actualCols){
    
    // These dimensions here represent the maximum rows and cols that will be incremented
    int rows = ENV_DIM;
    int cols = ENV_DIM;

    char c;
    bool stop = false;

    int row_index = 0;
    int col_index = 0;
    while(std::cin.get(c))
    {            
        if (!std::cin.eof())
        {
            // Counts the number of cols
            // Once the character reaches the end of the first row
            // If the number of cols in the input is greater than the default
            // Expand the cols
            if (c != '\n' && !stop)
            { 
                env[row_index][col_index] = c;
                ++col_index;
                if(col_index == cols) {  
                    expandCols(rows, &cols, env);
                }
            }

            // Every time the character hits a new line it counts the rows
            if (c == '\n')
            {
                ++row_index;
                stop = true;
                col_index = 0;
                if(row_index == rows) {  
                    expandRows(&rows, cols, env);
                }
            }  
            
            // Count and set the rest of the characters in the environment 
            // without needing to expand the cols
            if ((c != '\n') && (stop == true))
            {
                env[row_index][col_index] = c;
                ++col_index;
                
            }          

        }

    }
    // Adds one more row because it doesn't count the last end of line character
    row_index += 1;

    // Resizes envrionment regardless if it is smaller or not
    // If so, the environment dimensions are changed anyway
    resizeEnv(env, row_index, col_index, &rows, &cols);
    *actualRows = rows;
    *actualCols = cols;    
}

// Copies elements of the old array to the new array
void copyEnv(Env oldEnv, Env newEnv, int rows, int cols)
{
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            newEnv[i][j] = oldEnv[i][j];
        }
    }
}

// Copies the elements from the new array back into the old array
void copyEnvBack(Env env, Env newEnv, int rows)
{
    for (int i = 0; i < rows; ++i) 
    {
        env[i] = newEnv[i];
    }
}

// Initiates the new empty space for expanded dimensions
void initialiseEnv(Env newEnv, int rows, int cols)
{
    for(int i = 0; i < rows; ++i){
        for(int j = 0; j < cols; ++j){
            newEnv[i][j] = ENV_INITIALISE_SYMBOL; 
        }
    }
}

// If the default environment is smaller is contracts it down to inputted environment dimensions
// If not, it does it anyway just to be sure
void resizeEnv(Env env, int rows_index, int cols_index, int* rows, int* cols)
{
    Env newEnv = new char*[rows_index];
    for (int i = 0; i < rows_index; ++i)
    {
        newEnv[i] = new char[cols_index];
    }

    copyEnv(env, newEnv, rows_index, cols_index);

    for (int i = 0; i < *rows; ++i) 
    {
        delete env[i];
    }

    copyEnvBack(env, newEnv, rows_index);

    delete[] newEnv;

    // Updating max rows and cols
    *rows = rows_index;
    *cols = cols_index;
    
}

// Expands the columns by the predefined increment
void expandCols(int rows, int* cols, Env oldEnv){
    int newCols = *cols + INCREMENT;

    Env newEnv = new char*[rows];
    for (int i = 0; i != rows; ++i) {
        newEnv[i] = new char[newCols];
    }

    initialiseEnv(newEnv, rows, newCols);
    copyEnv(oldEnv, newEnv, rows, *cols);
    *cols = newCols;
    copyEnvBack(oldEnv, newEnv, rows);

    delete[] newEnv;
}

// Expand the rows by the predefined increment
void expandRows(int* rows, int cols, Env oldEnv){
    int newRows = *rows + INCREMENT;
    
    Env newEnv = new char*[newRows];
    for (int i = 0; i != newRows; ++i) {
        newEnv[i] = new char[cols];
    }

    initialiseEnv(newEnv, newRows, cols);
    copyEnv(oldEnv, newEnv, *rows, cols);
    *rows = newRows;
    copyEnvBack(oldEnv, newEnv, *rows);

    delete[] newEnv;
}

// Prints out the environment after updating it with the solution
void printEnvStdout(Env env, NodeList* solution, int rows, int cols) {
    Node* currNode = nullptr;
    Node* nextNode = nullptr;
    
    for (int i = 1; i < solution->getLength()-1; ++i)
    {
        // Comparison checks for start and goal nodes then updates navigation path 
        currNode = new Node(solution->getNode(i)->getRow(), solution->getNode(i)->getCol(), solution->getNode(i)->getDistanceTraveled());
    
        // Compares direction of the current node to the previous node then updates navigation paths
        nextNode = new Node(solution->getNode(i+1)->getRow(), solution->getNode(i+1)->getCol(), solution->getNode(i-1)->getDistanceTraveled());
        if (currNode->getCol() > nextNode->getCol())
        {
            env[currNode->getRow()][currNode->getCol()] = MOVE_LEFT;
        }
        else if (currNode->getCol() < nextNode->getCol())
        {
            env[currNode->getRow()][currNode->getCol()] = MOVE_RIGHT;
        }
        else if (currNode->getRow() < nextNode->getRow())
        {
            env[currNode->getRow()][currNode->getCol()] = MOVE_DOWN;
        }
        else if (currNode->getRow() > nextNode->getRow())
        {
            env[currNode->getRow()][currNode->getCol()] = MOVE_UP;
        }

        delete nextNode;
        nextNode = nullptr;

        delete currNode;
        currNode = nullptr;
    }

    // Prints out the updated envrionment
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            std::cout<<env[i][j];
        }
        if (i < rows-1)
            std::cout<<std::endl;
    } 
}

void testNode() {
    std::cout << "TESTING Node" << std::endl;

    // Make a Node and print out the contents
    Node* node = new Node(1, 1, 2);
    std::cout << node->getRow() << ",";
    std::cout << node->getCol() << ",";
    std::cout << node->getDistanceTraveled() << std::endl;
    delete node;

    // Change Node and print again
    node = new Node(4, 2, 3);
    std::cout << node->getRow() << ",";
    std::cout << node->getCol() << ",";
    std::cout << node->getDistanceTraveled() << std::endl;
    delete node;
}

void testNodeList() {
    std::cout << "TESTING NodeList" << std::endl;

    // Make a simple NodeList, should be empty size
    NodeList* nodeList = new NodeList();
    int maxArraySize = 4*20*20;
    nodeList->setMaxArraySize(maxArraySize);
    std::cout << "NodeList size: " << nodeList->getLength() << std::endl;

    // Add a Node to the NodeList, print size
    Node* b1 = new Node(1, 1, 1);
    nodeList->addElement(b1);
    std::cout << "NodeList size: " << nodeList->getLength() << std::endl;

    // Add second Nodetest
    Node* b2 = new Node(0, 0, 1);
    nodeList->addElement(b2);
    std::cout << "NodeList size: " << nodeList->getLength() << std::endl;

    // Test Get-ith - should be 0,0,1
    Node* getB = nodeList->getNode(1);
    std::cout << getB->getRow() << ",";
    std::cout << getB->getCol() << ",";
    std::cout << getB->getDistanceTraveled() << std::endl;

    // Print out the NodeList
    std::cout << "PRINTING OUT A NODELIST IS AN EXERCISE FOR YOU TO DO" << std::endl;
    for (int i = 0; i < nodeList->getLength(); ++i)
    {
        std::cout << "( " << nodeList->getNode(i)->getRow() 
                          << ", " 
                          << nodeList->getNode(i)->getCol() 
                          << ", " 
                          << nodeList->getNode(i)->getDistanceTraveled() 
                          << ") ";
    }
}