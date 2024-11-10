#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <queue>
#include "a_star.h"

using namespace std;
struct Node
{
	int x,y;
	double g_cost,h_cost,f_cost;
	Node *parent;
	//using an initialiser list instead of this keyword
	Node(int x,int y, double g_cost=0;double h_cost=0;Node *parent=NULL): x(x), y(y), g_cost(g_cost), h(h_cost), f_cost(g_cost+h_cost), parent(parent) {}
        bool operator>(const Node& other) const
       	{ 
		return f_cost > other.f_cost;
       	}//overloading > to comapre between final cost of the nodes

       /* Node(int x, int y, double g_cost=0, double h_cost=0, Node* parent=NULL) {
            this->x = x;            
            this->y = y;           
            this->g_cost = g_cost;            
            this->h_cost = h_cost;            
            this->f_cost = g_cost + h_cost;        
            this->parent = parent;  
          }

        bool operator>(const Node& other) const {
            return f > other.f;
         }*/
};

struct comparenode {
    bool operator()(Node*a, Node*b)
    {
            return *a>*b;
    }
};

double heuristic(int x1, int y1, int x2, int y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
vector<Node>astar(Node start, Node goal)
{
   priority_map<Node*,vector<Node*>,comparenode>openlist;
   unordered_map<int, Node*>allNodes;
   start.h = heuristic(start.x, start.y, goal.x, goal.y);
   start.f_cost = start.g_cost + start.h_cost;
   openList.push(&start);

   while(!openList.empty())
   {
     //stack
     Node*current=openList.top();
     openList.pop();//read one by one
     if(current->x==goal.x && current->y==goal.y)
     {
	     vector <Node> path;
	     for(Node *n=current;n!=NULL;n=n->parent)
	     {
		     path.push_back(n);
	     }
	     return path;
     }
   }


}
int main(){

vector<Node>path = astar(grid,start,goal)
if(path.empty())
{
	cout<<"No path found"<<endl;
}
else
{
	cout<<"Path found"<<endl;
	for(const Node&node:path)
	{
		cout<<"("<<node.x<<","<<node.y<<")";
	}
	cout<<endl;
}
}
