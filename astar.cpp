#include <iostream>
#include <vector>
#include <cmath>

struct Node
{
	int x,y;
	double g,h,f;
	Node *parent;
}
double heuristic(int x1, int y1, int x2, int y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int main(){

}
