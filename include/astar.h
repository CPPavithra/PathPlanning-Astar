#ifndef ASTAR_H
#define ASTAR_H
#include <vector>
#include <string>

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
};

double heuristic(int x1, int y1, int x2, int y2);

vector<Node>astar(const vector<vector<int>>& grid, Node start, Node goal);

#endif //ASTAR_H
