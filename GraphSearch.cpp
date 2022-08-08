// GraphSearch.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <list>
#include <queue>
#include <unordered_map>
#include <string>

using namespace std;

// forward declaration
struct Node;

struct Edge
{
	int m_cost;
	Node* m_node;

	Edge(int cost, Node& node)
	: m_cost(cost)
	, m_node(&node)
	{
	}
};

struct Node
{
	string m_name;
	std::vector<Edge> m_neighbors;

	int getCost(Node* n) const
	{
		auto compare = [n](const Edge& e) -> bool
		{
			return e.m_node == n;
		};
		auto it = std::find_if(m_neighbors.begin(), m_neighbors.end(), compare);
		if (it != m_neighbors.end())
		{
			return it->m_cost;
		}
		return -1;
	}
	
	Node(string name)
		: m_name(name)
	{
	}
};

struct VisitedNode
{
	Node* m_node = nullptr;
	int m_cost_so_far = 0; // cost so far to this m_node

	VisitedNode() = default;
	VisitedNode(Node* node, int cost)
		: m_node(node)
		, m_cost_so_far(cost)
	{
	}

	friend bool operator<(VisitedNode const& n1, VisitedNode const& n2)
	{
		// lower score is higher priority
		return n1.m_cost_so_far > n2.m_cost_so_far;
	}
};

// Breadth First Search explores equally in all direction, doesnt not care about cost of edge
std::unordered_map<Node*, VisitedNode> BreadthFirstSearch(Node* start, Node* goal)
{
	std::queue<Node*> frontier;
	frontier.push(start);
	std::unordered_map<Node*, VisitedNode> came_from; // the node it came from
	came_from[start] = {nullptr, 0};

	while (!frontier.empty())
	{
		Node* current = frontier.front();
		frontier.pop();

		// early out
		if (current == goal)
		{
			break;
		}

		for (auto edge : current->m_neighbors)
		{
			Node* neighbor = edge.m_node;
			if (came_from.find(neighbor) == came_from.end())
			{
				frontier.push(neighbor);
				came_from[neighbor] = {current, 0};
			}
		}
	}
	return came_from;
}

// Dijkstra First Search prioritize which paths to explore 
std::unordered_map<Node*, VisitedNode> DijkstraSearch(Node* start, Node* goal)
{
	std::priority_queue<VisitedNode> frontier;
	frontier.push({start, 0});
	std::unordered_map<Node*, VisitedNode> came_from; // the node it came from
	came_from[start] = {nullptr, 0};

	while (!frontier.empty())
	{
		Node* current = frontier.top().m_node;
		frontier.pop();

		// early out
		if (current == goal)
		{
			break;
		}

		for (auto edge : current->m_neighbors)
		{
			Node* neighbor = edge.m_node;
			int new_cost = came_from[current].m_cost_so_far+ current->getCost(neighbor);
			if (came_from.find(neighbor) == came_from.end() || new_cost < came_from[neighbor].m_cost_so_far)
			{
				came_from[neighbor].m_cost_so_far = new_cost;
				came_from[neighbor].m_node = current;
				frontier.push({neighbor, new_cost});
			}
		}
	}
	return came_from;
}

std::vector<Node*> GetPath(const std::unordered_map<Node*, VisitedNode>& paths, Node* start, Node* goal)
{
	std::vector<Node*> path;
	path.push_back(goal);
	Node* current = goal;
	while (current != start)
	{
		auto it = paths.find(current);
		if (it != paths.end())
		{
			Node* to = it->first;
			Node* from = it->second.m_node;
			path.insert(path.begin(), from); // insert at front
			current = from;
		}
	}
	return path;
}

void Print(const std::vector<Node*>& path)
{
	int totalCost = 0;
	for (auto it = path.begin(); it < path.end()-1; ++it)
	{
		auto current = *it;
		auto next = *(it+1);
		auto cost = current->getCost(next);
		cout << current->m_name << "-(" << current->getCost(next) << ")->" << next->m_name << endl;
		totalCost += cost;
	}
	cout << "totalCost: " << totalCost << endl;
}

// https://www.redblobgames.com/pathfinding/a-star/introduction.html
// https://www.redblobgames.com/pathfinding/a-star/implementation.html#python-dijkstra
// https://brilliant.org/wiki/a-star-search/#the-a-algorithm
int main()
{
	// Create the Nodes
	std::shared_ptr<Node> A = std::make_shared<Node>("A");
	std::shared_ptr<Node> B = std::make_shared<Node>("B");
	std::shared_ptr<Node> C = std::make_shared<Node>("C");
	std::shared_ptr<Node> D = std::make_shared<Node>("D");

	// Make a Graph	
	A->m_neighbors.push_back({ 5,*B });
	A->m_neighbors.push_back({ 10,*D });
	B->m_neighbors.push_back({ 1,*C });
	C->m_neighbors.push_back({ 1,*D });

	// print out a possible paths from A (not shortest)
	cout << "Breath First Search" << endl;
	std::vector<Node*> path = GetPath(BreadthFirstSearch(A.get(), D.get()), A.get(), D.get());
	Print(path);

	cout << endl << "Dijkstra Search" << endl;
	// print out a possible paths from A (not shortest)
	path = GetPath(DijkstraSearch(A.get(), D.get()), A.get(), D.get());
	Print(path);
}
