// Method reverseList() referenced from
// "C++ Program to Reverse an Array", CodesCracker, C++ Programming Examples, retrieved from https://codescracker.com/cpp/program/cpp-program-reverse-array.htm

#include "NodeList.h"
#include <iostream>

NodeList::NodeList(){
    // Declaring variables
    this->length = 0;
    this->maxArraySize = 0;
  
}

// Making a method to set the array size to avoid
// passing a value through the constructor and changing the provided methods
// Only downside is that I'd have to be calling this method each time
// a nodelist is created
void NodeList::setMaxArraySize(int arraySize) {
    // Setting elements of node array to null pointer
    this->maxArraySize = arraySize;
    nodes = new Node*[maxArraySize];
    for (int i = 0; i < this->maxArraySize; ++i)
    {
        nodes[i] = nullptr;
    }
}

int NodeList::getMaxArraySize()
{
    return maxArraySize;
}
NodeList::~NodeList(){
    // Cleaning array NodeList
    for (int i = 0; i < maxArraySize; ++i)
    {
        if (nodes[i] != nullptr)
        {
            delete nodes[i];
            nodes[i] = nullptr;
        }
    }
    delete[] nodes;
}

NodeList::NodeList(NodeList& other){
    this->length = other.length;
    this->maxArraySize = other.maxArraySize;

    nodes = new Node*[this->maxArraySize];
    for (int i = 0; i < this->maxArraySize; ++i)
    {
        nodes[i] = nullptr;
    }

    for (int i = 0; i < other.length; ++i)
    {
        nodes[i] = new Node(other.getNode(i)->getRow(), other.getNode(i)->getCol(), other.getNode(i)->getDistanceTraveled());
    }
}

int NodeList::getLength(){
    return length;
}

void NodeList::addElement(Node* newPos){
    for (int i = 0; i < maxArraySize; ++i)
    {
        if (nodes[i] == nullptr)
        {
            nodes[i] = new Node(*newPos);
            ++length;
            i = maxArraySize+1;
        }
        
    }
}

Node* NodeList::getNode(int i){
    return nodes[i];
}

// Reverse the elements of the node list
void NodeList::reverseList()
{
    // Sets the index of j to final element in nodelist
    int j = length-1;
    for (int i = 0; i < j; ++i, --j)
    {
        // This reverses the list by swapping the first and last index, followed by the second and second last, and so on...
        Node* temp = nodes[i];
        nodes[i] = nodes[j];
        nodes[j] = temp;
    }
}