#ifndef __NODE_H
#define __NODE_H

//Nodes 0-5
#define NODE 0

#if NODE==0
#define CONNECTIONS() int connections[] = {1, 2, 3}
#define SERVERS 3
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
