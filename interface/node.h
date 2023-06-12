#ifndef __NODE_H
#define __NODE_H

//Nodes 0-5
//
#define NODE 1 //CHANGE THIS DEPEDING ON WHICH NODE IS THIS CODE GOING TO BE FLASHED FOR ACCORDING TO THE INSTRUCTIONS BELOW

#define CCU 0 //Central control unit has a node number of 0
#define MIDX 1 //the board that controls the X axis has a node number of 1
#define LEFTY 2 //the board that controls the left of the y axis has a node number of 2
#define RIGHTY 3 //the board that controls the right of the y axis has a node number of 3

//Predefine the mesh. In our case, star configuration, node 0 connected to everything and everything to node 0
#if NODE==0
#define CONNECTIONS() int connections[] = {1, 2, 3, 4 ,5}
#define SERVERS 5
#elif NODE==1
#define CONNECTIONS() int connections[] = {0}
#define SERVERS 1
#elif NODE==2
#define CONNECTIONS() int connections[] = {0}
#define SERVERS 1
#elif NODE==3
#define CONNECTIONS() int connections[] = {0}
#define SERVERS 1
#elif NODE==4
#define CONNECTIONS() int connections[] = {0}
#define SERVERS 1
#elif NODE==5
#define CONNECTIONS() int connections[] = {0}
#define SERVERS 1
#endif

#endif // __NODE_H
