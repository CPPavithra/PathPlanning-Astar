#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <fstream>
#include <sstream>

using namespace std;

struct Node
{
	float x,y;
	double g_cost,h_cost,f_cost;
	Node *parent;
	//using an initialiser list instead of this keyword
	Node(float x,float y, double g_cost=0,double h_cost=0,Node *parent=NULL): x(x), y(y), g_cost(g_cost), h_cost(h_cost), f_cost(g_cost+h_cost), parent(parent) {}
        bool operator>(const Node& other) const
       	{ 
		return f_cost > other.f_cost;
       	}//overloading > to comapre between final cost of the nodes

       /* Node(float x, int y, double g_cost=0, double h_cost=0, Node* parent=NULL) {
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
vector<Node>astar(const vector<vector<int>>& grid, Node start, Node goal)
{
   priority_queue<Node*,vector<Node*>,comparenode>openlist;//priority list for 
   unordered_map<int, Node*>allNodes;//map for all the nodes
   start.h_cost = heuristic(start.x, start.y, goal.x, goal.y);//euclidean distance function h cost
   start.f_cost = start.g_cost + start.h_cost;
   openlist.push(&start);

   while(!openlist.empty())
   {
     //stack
     Node*current=openlist.top();
     openlist.pop();//read one by one
     if(current->x == goal.x && current->y == goal.y)
     {
	     vector <Node> path;
	     for(Node *n=current;n!=NULL;n=n->parent)
	     {
		     path.push_back(*n);//error
	     }
	     reverse(path.begin(), path.end()); //reverse path
	     return path;
     }
    //for variable square grid size
    int dx[] = {1,-1,0,0,1,-1,1,-1};
    int dy[] = {0,0,1,-1,1,-1,-1,1}; 

   int i; double newg_cost;
   for(i=0; i<8; i++)
   {
	   int newx = current->x+dx[i];
	   int newy  = current->y+dy[i];
     if(newx>=0 && newx<grid.size() && grid[newx][newy]==0 && newy>=0 && newy<grid[0].size())//boundary condition checking
     {
	     if(i<4)//frist 4 in the array is for straight movements
	     {
		     newg_cost=current->g_cost+1.0;
	     }
	     else
	     {
                    newg_cost=current->g_cost+1.414;//diagonal cost will be root2
	     }
	     Node* neighbour = new Node(newx, newy, newg_cost, heuristic(newx, newy, goal.x, goal.y), current);//create a new neighbour node
             //newx+newy shows collision
	     int key=newx*grid[0].size()+newy;
	     
	     if(allNodes.find(key)==allNodes.end() || neighbour -> g_cost<allNodes.find(key) -> second -> g_cost)//newx+newy is the unique key for map
	     {
               neighbour->f_cost=neighbour->g_cost+neighbour->h_cost;
	       openlist.push(neighbour);
	       //update all nodes
	       allNodes[key]=neighbour;
	     }

     } 
   }
}
return{}; //return empty if not found in while loop
}

vector<vector<int>> readGridFromCSV(const string& filename) {
    vector<vector<int>> grid;
    ifstream file(filename);
    if (!file.is_open()) {
        throw runtime_error("Could not open CSV file.");
    }
    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string value;
        vector<int> row;
        while (getline(ss, value, ',')) {
            row.push_back(stoi(value));
        }
        grid.push_back(row);
    }
    file.close();
    return grid;
}

int main(){
//SAMPLE GRID
/*vector<vector<int>> grid = {
 (0,0) {0, 0, 0, 0, 0},
  {0, 1, 1, 1, 1},
  {1, 0, 0, 0, 0},
  {0, 1, 1, 1, 0},
  {0, 0, 0, 0, 0}(4,4)
};*/

cout << "Reading grid map from CSV file" << endl;

// Read the grid map from a CSV file
string filename = "grid_map.csv"; // Replace with your CSV file name
vector<vector<int>> grid;
try {
    grid = readGridFromCSV(filename);
} catch (const exception& e) {
    cerr << "Error: " << e.what() << endl;
    return 1;
}
cout << "Grid map loaded" << endl;

    
double min_x = 0;
double max_x = grid[0].size();
double min_y = 0;
double max_y = grid.size();

cout<<"Setting boundaries"<<endl;

Node start(0,0);
Node goal(5,6);

cout<<"Start and goal node set astar starts"<<endl;

vector<Node>path = astar(grid,start,goal);
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
