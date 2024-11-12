#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <algorithm>

using namespace std;
struct Node
{
	int x,y;
	double g_cost,h_cost,f_cost;
	Node *parent;
	//using an initialiser list instead of this keyword
	Node(int x,int y, double g_cost=0,double h_cost=0,Node *parent=NULL): x(x), y(y), g_cost(g_cost), h_cost(h_cost), f_cost(g_cost+h_cost), parent(parent) {}
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
   priority_queue<Node*,vector<Node*>,comparenode>openlist;//priority list for 
   unordered_map<int, Node*>allNodes;//map for all the nodes
   start.h_cost = heuristic(start.x, start.y, goal.x, goal.y);//euclidean distance function h cost
   start.f_cost = start.g_cost + start.h_cost;
   openList.push(&start);

   while(!openlist.empty())
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
	     reverse(path.begin(), path.end()); //reverse path
	     return path;
     }
   int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};
   int dy[] = {0, 0, 1, -1, 1, -1, -1, 1}; //for the directions 
   int i; double newg_cost;
   for(i=0;i<8;i++)
   {
	   int newx=current->x+dx[i];
	   int newy=current->y+dy[i];

     if(newx>=0 && newx<grid.size() && grid[newx][newy]==0 && newy>=0 && newy<grid[0].size())//boundary condition checking
     {
	     if(i<4)//frist 4 in the array is for straight movements
	     {
		     newg_cost=current->g_cost+1.0;
	     }
	     else
	     {
                    newg_cost=current->g_cost+1.414;
	     }
	     Node* neighbour = new Node(newx, newy, newg_cost, heuristic(newx, newy, goal.x, goal.y), current);//create a new neighbour node
             //newx+newy shows collision
	     int key=newx*grid[0].size+newy;
	     
	     if(allNodes.find(key)==allNodes.end()||neighbour->g_cost<allNodes.find(key)->g_cost)//newx+newy is the unique key for map
	     {
               neighbour->f_cost=neighbour->g_cost+neighbour->h_cost;
	       openlist.push(neighbour);
	       //update all nodes
	       allNodes[newx+newy]=neighbour;
	     }

     } 
   }
}
return{}; //return empty if not found in while loop
}
int main(){
//SAMPLE GRID
vector<vector<int>> grid = {
  {0, 0, 0, 0, 0},
  {0, 1, 1, 1, 0},
  {0, 0, 0, 0, 0},
  {0, 1, 1, 1, 0},
  {0, 0, 0, 0, 0}
};

Node start(0, 0);
Node goal(4, 4);
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
