#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream> //for fileIO
#include <cstring>

const int MAX_CHARS_PER_LINE = 512;
const int MAX_TOKENS_PER_LINE = 10;
const char* const DELIMITER = " ";
const int INF = 9000;

using std::ifstream;

typedef struct Node{
	//int id; //Id is the index in the array
	int numNeighbors;
	double x,y; //xy position
	int* adj; //List of neighbors by ID
} Node;

//	@g - graph of nodes
//	@n - the number of vertices
//	@start - index of starting node
//
//	Overview: First set distance between all nodes to be inf.
//		Then 	
//

void dijkstra(Node** g, int n, int start,int end){
	int cost[n][n], dist[n], prev[n];
	int visited[n], count, mindist, nextnode = 0;

	//set all paths
	for(int i = 0; i < n; i++){
		dist[i] = INF;
		prev[i] = -1;
		visited[i] = 0;
	}

	//init starting node
	dist[start] = 0;
	//visited[start] = 1;
	count = 0;


	while(count < n-1){
		mindist = INF;
		for(int i = 0; i < n; i++){
			if(dist[i] < mindist && !visited[i]){
				mindist = dist[i];
				nextnode = i;
			}
		} //finished finding vertex with min v
		visited[nextnode] = 1;


		int numNeighbors = g[nextnode]->numNeighbors;


		for(int i = 0; i < numNeighbors; i++){
			int neighbor = g[nextnode]->adj[i];
			if(!visited[neighbor]){
				dist[neighbor] = mindist + 1;
				prev[neighbor] = nextnode;
			}
		} //end of updating dist using each neighbor
		count++;

		/*//for debug
		std::cout << "distances are: " << std::endl;
		for(int i = 0; i < n; i++){
			std::cout << dist[i] << " ";
		}
		
		std::cout << "\nprev are: " << std::endl;
		for(int i = 0; i < n; i++){
			std::cout << prev[i] << " ";
		}
		std::cout << std::endl;
		*/

	}  //end of finding shortest path using greedy

	/*
	//print all shortest paths
	for(int i = 0; i < n; i++){
		if(i!=start){
			std::cout << "\nDistance of " << i << " = " << dist[i] <<std::endl;
			std::cout << "Path = " << i;
			int j = i;
			while(j!= start){
				j = prev[j];
				std::cout << "<-" <<  j;
			}
		}
	}
	*/

	std::cout << "Path = " << end;
	int j = end;
	while(j!= start){
		j = prev[j];
		std::cout << "<-" <<  j;
	}

	std::cout << std::endl;
}


void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){
	//ROS_INFO("Seq: [%d]",msg->header.seq);
	//ROS_INFO("Position-> x: [%f], y:[%f], z: [%f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	//ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	//ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x, msg->twist.twist.angular.z);

	std::cout<<"X: "<< msg->pose.pose.position.x<<" Y: " << msg->pose.pose.position.y << std::endl;
}


int main(int argc, char* argv[]){

	//create list of points
	Node** nodes;

	//check for correct number of commands
	//	example
	//		usage: rosrun beginner_tuts odom_listener waypoints.txt 2
	//		explanation: parse waypoints.txt and start from starting waypoint 2
	if(argc != 2 && argc != 4){
		std::cout << "not enough inputs!\n\nUsage: rosrun beginner_tuts odom_listener <waypointlist> <starting point default=0> <end point default = 34" << std::endl;
		return -1;
	}

	//parse file
	ifstream fin;		//create file handler
	fin.open(argv[1]);	//open file
	if(!fin.good()) return 1; //exit if file not found

	//get number of lines/Nodes
	char lineBuf[MAX_CHARS_PER_LINE];
	fin.getline(lineBuf,MAX_CHARS_PER_LINE);
	int numLines = atoi(lineBuf);
	std::cout << "Number of Nodes is: " << numLines << std::endl;
	nodes = new Node*[numLines];
	int numNodes = 0;

	//read each line of file
	while(!fin.eof()){
		//read entire line
		char buf[MAX_CHARS_PER_LINE];
		fin.getline(buf,MAX_CHARS_PER_LINE);

		int n;
		const char* token[MAX_TOKENS_PER_LINE] = {};

		//read each line into usable format
		token[0] = strtok(buf,DELIMITER);
		if(token[0]){ //zero if line is blank
			for(n = 1; n < MAX_TOKENS_PER_LINE; n++){
				token[n] = strtok(0,DELIMITER);
				if(!token[n]) break; //no more tokens
			}
		}

		//parse each input nodes
		if(token[0] == NULL){
			std::cout << "EOF!" << std::endl;
			continue;
		} else {
			Node* node;
			node = new Node;
			node->numNeighbors = 0;
			node->x = atoi(token[0])*1.7;
			node->y = atoi(token[1])*1.7;
			node->adj = new int [10];

			//std::cout << "X: " << token[0] << " Y: " << token[1] << " with neighbors: ";
			for(int i = 2; i < n; i++){
				//std::cout << "Token[" << i << "] = " << token[i] << std::endl;
				//std::cout << token[i] << " ";
				node->adj[node->numNeighbors++] = atoi(token[i]);
				//node->numNeighbors++;
			}
			nodes[numNodes++] = node;
		}
		//std::cout << std::endl;

	} //end of parsing file

	std::cout << "got here" << std::endl;

	/*
	for(int i = 0; i < numLines; i++){
		std::cout << "Node "<<i<<" has neighbors: ";
		for(int j = 0; j < nodes[i]->numNeighbors; j++){
			std::cout << nodes[i]->adj[j]<<" ";
		}
		std::cout << std::endl;
	}
	*/

	dijkstra(nodes,numNodes,0,125);

	ros::init(argc,argv,"odom_listener");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/rtabmap/odom",1000,chatterCallback);

	ros::spin();

	return 0;
}
