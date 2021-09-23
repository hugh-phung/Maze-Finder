#ifndef COSC_ASSIGN_ONE_TYPES
#define COSC_ASSIGN_ONE_TYPES

#include <math.h> 


#define ENV_DIM                     20
#define NODE_LIST_ARRAY_MAX_SIZE    4*(ENV_DIM * ENV_DIM)
#define INCREMENT                   5
#define ENV_INITIALISE_SYMBOL       '0'

// A 2D array to represent the environment or observations
// REMEMBER: in a environment, the location (x,y) is found by maze[y][x]!
typedef char** Env;

#define SYMBOL_WALL     '='
#define SYMBOL_EMPTY    '.'
#define SYMBOL_GOAL     'G'
#define SYMBOL_START    'S'

// Defining constants for printing out environment
#define MOVE_RIGHT '>'
#define MOVE_LEFT '<'
#define MOVE_UP '^'
#define MOVE_DOWN 'v'

#endif // COSC_ASSIGN_ONE_TYPES