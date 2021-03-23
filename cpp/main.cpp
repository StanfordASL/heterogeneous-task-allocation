#include <iostream>
#include <iostream>
#include <cstdio>
#include <ctime>
//#include <lemon/list_graph.h>
#include <lemon/smart_graph.h>
#include <lemon/network_simplex.h>
#include <lemon/cost_scaling.h>
#include <lemon/dijkstra.h>
#include "csv.h"

using namespace lemon;
using namespace std;

template <int numFields>
using CsvDialect = io::CSVReader<numFields>;

using Graph = SmartDigraph;
using Node = Graph::Node;
using Arc = Graph::Arc;
using NodeIt = Graph::NodeIt;
using ArcIt = Graph::ArcIt;

template<typename Type>
using NodeMap = Graph::NodeMap<Type>;

template<typename Type>
using ArcMap = Graph::ArcMap<Type>;

//using Weight = double;
using Weight = int;
using Capacity = int;

using NS = NetworkSimplex<Graph, Capacity, Weight>;

// returns solution for private task of a specific fleet
void solvePrivate(const Graph& workspace, const vector<vector<double>>& rewards, const vector<int>& init, vector<vector<int>>& x,  vector<vector<int>>& y)
{
	//cout << "computing homogeneous Private" << endl;
	
	int agents = init.size();
	
	Graph g; // flow graph
	ArcMap<int> capacity(g, agents); // capacity of edges
	//ArcMap<double> cost(g, 0); // cost of edges
	ArcMap<int> cost(g, 0); // cost of edges
	ArcMap<int*> correspondence(g, NULL); // correspondence between edges and value s of x,y
	
	int workspaceNodeNum = workspace.maxNodeId()+1;
	int Thor = rewards.size();
	
	// generate new nodes
	Node start = g.addNode();
	Node target = g.addNode();

	vector<vector<Node>> vNodes(Thor, vector<Node>(workspaceNodeNum));
	vector<vector<Node>> wNodes(Thor, vector<Node>(workspaceNodeNum));

	for (int t = 0; t < Thor; t++)
	{
		for (int v = 0; v <= workspace.maxNodeId(); v++)
		{
			vNodes[t][v] = g.addNode();
			wNodes[t][v] = g.addNode();
		}
	}
	
	// consolidate init vector
	vector<int> init_consol(workspaceNodeNum,0);
	for (vector<int>::const_iterator it = init.begin(); it !=  init.end(); it++)
		init_consol[*it]++;

	// generate E_s edges
	for (int v = 0; v < workspaceNodeNum; v++)
		if (init_consol[v] > 0)
		{
			Arc a = g.addArc(start,vNodes[0][v]);
			capacity[a] = init_consol[v];
			cost[a] = 0;
			correspondence[a]= NULL;
		}	

	// generate E_t edges
	for (int v = 0; v < workspaceNodeNum; v++)
	{
		Arc a = g.addArc(wNodes[Thor-1][v],target);
		capacity[a] = agents;
		cost[a] = 0;
		correspondence[a]= NULL;
	}
	
	// generate E_0 edges
	for (int t = 0; t < Thor; t++)
		for (int v = 0; v < workspaceNodeNum; v++)
		{
			Arc a = g.addArc(vNodes[t][v],wNodes[t][v]);
			capacity[a] = agents;
			cost[a] = 0;
			correspondence[a] = NULL;
		}

	//cout << "Thor " << Thor << endl;
	
	// generate E_R edges
	for (int t = 0; t < Thor; t++)
		for (int v = 0; v < workspaceNodeNum; v++)
		{
			// cout << "node v = " << v << ", t = " << t << endl;
			
			if (rewards[t][v] != 0)
			{
				Arc a = g.addArc(vNodes[t][v],wNodes[t][v]);
				capacity[a] = 1;
				//cost[a] = rewards[t][v];
				cost[a] = (int)rewards[t][v];
				correspondence[a] = &y[t][v];
			}
		}
	
	
	// generate E_E edges
	for (int t = 0; t < Thor-1; t++)
		for (ArcIt ai(workspace); ai != INVALID; ++ai)
		{		
			int o = workspace.id(workspace.source(ai));
			int d = workspace.id(workspace.target(ai));

			Arc a = g.addArc(wNodes[t][o],vNodes[t+1][d]);
			capacity[a] = agents;
			cost[a] = 0;
			correspondence[a] = &x[t][workspace.id(ai)];
		}

	////////////////
	// compute flow
	///////////////
	
	NS ns(g);

	// initialize mcf solver with source / target nodes
	// and total flow coming from source
	ns.costMap(cost).upperMap(capacity).stSupply(start, target, agents);

	// solution will be updated in the following vector
	ArcMap<Capacity> flows(g);
	
	NS::ProblemType status = ns.run(NS::PivotRule::ALTERING_LIST);
	switch (status) {
	case NS::INFEASIBLE:
		cerr << "insufficient flow" << endl;
		break;
	case NS::OPTIMAL:
		ns.flowMap(flows);

		//cerr << "flow correct" << endl;
		/*
		for (ArcIt a(g); a != INVALID; ++a)
			cout << "flow on " << g.id(a) << " = " << flows[a] << endl;
		
			cerr << "cost=" << ns.totalCost() << endl; */
		break;
	case NS::UNBOUNDED:
		cerr << "infinite flow" << endl;
		break;
	default:
		break;
	}

	// update solution in x,y
	for (ArcIt a(g); a != INVALID; ++a)
		if (correspondence[a] != NULL)
			*correspondence[a]=flows[a];
}

// returns solution for shared task of all agents
// rewards are for shared task only
void solveShared(const Graph& workspace, const vector<vector<double>>& rewards, const vector<vector<int>>& init,  vector<vector<vector<int>>>& x, vector<vector<vector<int>>>& z)
{
	//cout << "computing homogeneous Shared" << endl;
	// convert shared problem into a private problem
	int Thor = rewards.size();
	int nodesNum = workspace.maxNodeId()+1;
	int arcsNum = workspace.maxArcId()+1;
	int fleetsNum = init.size();
	

	// generate initial positions input
	vector<int> init_flat;
	for (int f = 0; f < fleetsNum; f++)
		for (unsigned int i = 0; i < init[f].size(); i++)
			init_flat.push_back(init[f][i]);

	
	// generate flat x for output
	vector<vector<int>> x_flat(Thor-1, vector<int>(arcsNum + 1, 0));

	vector<vector<int>> y_flat(Thor, vector<int>(nodesNum, 0));

	solvePrivate(workspace, rewards, init_flat, x_flat, y_flat);
	
	/*cout << "x_flat values (time, origin, destination, value)" << endl;
	for (int t=0; t < Thor-1; t++)
		for (int ai = 0; ai < arcsNum; ai++)
			if (x_flat[t][ai] != 0)
			{
				Arc a = workspace.arcFromId(ai);
				int o = workspace.id(workspace.source(a));
				int d = workspace.id(workspace.target(a));
				cout << t << " " << o << " " << d << " " << x_flat[t][ai] << endl;
			}

	cout << "y_flat values (time, vertex, value)" << endl;
	for (int t = 0; t < Thor; t++)
		for (int v = 0; v < nodesNum; v++)
			if (y_flat[t][v]!=0)
				cout << t << " " << v << " " << y_flat[t][v] << endl;
	*/


	for (int f = 0; f < fleetsNum; f++)
	{
		int agents = init[f].size();
		for (int a = 0; a < agents; a++)
		{
			int curVertex = init[f][a];

			for (int t = 0; t < Thor; t++)
			{
				if (y_flat[t][curVertex] != 0)
				{
					z[f][t][curVertex]++;
					y_flat[t][curVertex]--;
				}

				if (t == Thor - 1)
					break;

				// find the corresponding edge
				for (int arcId = 0; arcId < workspace.maxArcId() + 1; arcId++)
				{
					if (x_flat[t][arcId] == 0)
						continue;

					Arc a = workspace.arcFromId(arcId);
					int o = workspace.id(workspace.source(a));
					int d = workspace.id(workspace.target(a));

					if (o != curVertex)
						continue;

					x_flat[t][arcId]--;
					x[f][t][arcId]++;
					curVertex = d;
					
					break;
				}
			}
		}
	}
}

double totalReward(const vector<vector<vector<double>>>& rewards, const vector<vector<vector<int>>>& y, const vector<vector<vector<int>>>& z)
{
	double R = 0;

	int fleetsNum = rewards.size() - 1;
	int Thor = rewards[0].size();
	int nodesNum = rewards[0][0].size();

	for (int f = 0; f < fleetsNum; f++)
		for (int t = 0; t < Thor; t++)
			for (int v = 0; v < nodesNum; v++)
			{
				if (y[f][t][v] > 0)
					R += rewards[fleetsNum][t][v];
				if (z[f][t][v] > 0)
					R += rewards[f][t][v];
			}

	return R;
}

double solveSharedFirst(const Graph& workspace, const vector<vector<vector<double>>>& rewards, const vector<vector<int>>& init,  vector<vector<vector<int>>>& x, vector<vector<vector<int>>>& y, vector<vector<vector<int>>>& z)
{
	//cout << "computing SharedFirst" << endl;
	int Thor = rewards[0].size();
	int nodesNum = workspace.maxNodeId()+1;
	int arcsNum = workspace.maxArcId()+1;
	int fleetsNum = init.size();
	
	vector<vector<vector<int>>> x_bar(fleetsNum, vector<vector<int>>(Thor-1, vector<int>(arcsNum, 0)));
	vector<vector<vector<int>>> y_bar(fleetsNum, vector<vector<int>>(Thor, vector<int>(nodesNum, 0))); 
	
	solveShared(workspace, rewards[fleetsNum], init, x_bar, y_bar);

	vector<vector<vector<double>>> rewards_bar(fleetsNum, vector<vector<double>>(Thor, vector<double>(nodesNum, 0)));

	for (int f = 0; f < fleetsNum; f++)
		for (int t = 0; t < Thor; t++)
			for (int v = 0; v < nodesNum; v++)
			{
				rewards_bar[f][t][v] = rewards[f][t][v];
				
				if (y_bar[f][t][v] != 0)
					rewards_bar[f][t][v] += rewards[fleetsNum][t][v];
			}

	for (int f = 0; f < fleetsNum; f++)
	{
		solvePrivate(workspace, rewards_bar[f], init[f], x[f], z[f]);
		y[f] = z[f];
	}

	return totalReward(rewards, y, z);
}

double solvePrivateFirst(const Graph& workspace, const vector<vector<vector<double>>>& rewards, const vector<vector<int>>& init,  vector<vector<vector<int>>>& x, vector<vector<vector<int>>>& y, vector<vector<vector<int>>>& z)
{
	//cout << "computing PrivateFirst" << endl;
	int Thor = rewards[0].size();
	int nodesNum = workspace.maxNodeId()+1;
	int fleetsNum = init.size();
	
	vector<vector<vector<double>>> rewards_hat(fleetsNum, vector<vector<double>>(Thor, vector<double>(nodesNum, 0)));

	for (int f = 0; f < fleetsNum; f++)
		for (int t = 0; t < Thor; t++)
			for (int v = 0; v < nodesNum; v++)
				rewards_hat[f][t][v] = rewards[f][t][v] + rewards[fleetsNum][t][v] / (double)fleetsNum;
			
	for (int f = 0; f < fleetsNum; f++)
		solvePrivate(workspace, rewards_hat[f], init[f], x[f], z[f]);

	for (int t = 0; t < Thor; t++)
		for (int v = 0; v < nodesNum; v++)
			for (int f = 0; f < fleetsNum; f++)
				if (z[f][t][v] == 1)
				{
					y[f][t][v] = 1;
					break;
				}

	return totalReward(rewards, y, z);
}

double solvePFSF(const Graph& workspace, const vector<vector<vector<double>>>& rewards, const vector<vector<int>>& init,  vector<vector<vector<int>>>& x, vector<vector<vector<int>>>& y, vector<vector<vector<int>>>& z)
{
	//cout << "computing PFSF" << endl;
	
	int Thor = rewards[0].size();
	int nodesNum = workspace.maxNodeId()+1;
	int arcsNum = workspace.maxArcId()+1;
	int fleetsNum = init.size();
	
	vector<vector<vector<int>>> x1(fleetsNum, vector<vector<int>>(Thor-1, vector<int>(arcsNum, 0)));
	vector<vector<vector<int>>> x2 = x1;
	
	vector<vector<vector<int>>> y1(fleetsNum, vector<vector<int>>(Thor, vector<int>(nodesNum, 0)));
	vector<vector<vector<int>>> y2 = y1;

	vector<vector<vector<int>>> z1 = y1;
	vector<vector<vector<int>>> z2 = y1;

	double r1 = solvePrivateFirst(workspace, rewards, init, x1, y1, z1);
	double r2 = solveSharedFirst(workspace, rewards, init, x2, y2, z2);

	cout << "privateFirst reward = " << r1 << endl;
	cout << "sharedFirst reward = " << r2 << endl;

	if (r1 < r2)
	{
		x = x1;
		y = y1;
		z = z1;
		
		return r1;
	}

	x = x2;
	y = y2;
	z = z2;

	return r2;
}



// imports input from csv files generated by python
int importer(const string& path, Graph& workspace_graph, vector<vector<vector<double>>>& rewards, vector<vector<int>>& init)
{
	CsvDialect<2> edgesFile(path+"/edges.csv");
	CsvDialect<4> rewardsFile(path+"/rewards.csv");
	CsvDialect<2> initFile(path+"/init.csv");
	
	edgesFile.read_header(io::ignore_extra_column, "origin", "destination");

	int o,d;
	// read vertex number
	assert(edgesFile.read_row(o,d));
	for (int i = 0; i < d; i++)
		workspace_graph.addNode();

	while(edgesFile.read_row(o,d))
	{
		Node o_n = workspace_graph.nodeFromId(o);
		Node d_n = workspace_graph.nodeFromId(d);
		workspace_graph.addArc(o_n,d_n);
	}

	cout << "loaded:" << endl;
	cout << "- workspace graph with " << workspace_graph.maxNodeId()+1 << " vertices and "  <<  workspace_graph.maxArcId()+1 << " edges" <<  endl;

	rewardsFile.read_header(io::ignore_extra_column, "task_type","time","vertex","value");
	
	int types, vertices, times;
	double value;
	
	rewardsFile.read_row(types, times, vertices, value);
	rewards = vector<vector<vector<double>>>(types);

	for (int type = 0; type < types; type++)
	{
		rewards[type] = vector<vector<double>>(times);
		for (int t = 0; t < times; t++)
		{
			rewards[type][t] = vector<double>(vertices);
			for (int v = 0; v < vertices; v++)
			{
				int dummy;
				rewardsFile.read_row(dummy, dummy, dummy, value);
				rewards[type][t][v] = -value;
			}
		}
	}
	cout << "- rewards with " << types * vertices * times << " entries" << endl;

	initFile.read_header(io::ignore_extra_column, "fleet", "vertex");
	init = vector<vector<int>>(types-1,vector<int>());

	int f,v;
	while(initFile.read_row(f,v))
	{
		init[f].push_back(v);
	}

	cout << "- init with ";
	for (int f = 0; f < types -1; f++)
		cout << init[f].size() << " ";
	cout << "agents" << endl;

	return times;
}

int main(int argc, char** argv)
{
	string path = argv[1];
	
	Graph workspace;
	vector<vector<vector<double>>> rewards;
	vector<vector<int>> init;

	int timeHor = importer(path, workspace, rewards, init);

	int fleetsNum = init.size();

	vector<vector<vector<int>>> x(fleetsNum, vector<vector<int>>(timeHor-1, vector<int>(workspace.maxArcId() + 1, 0)));
	vector<vector<vector<int>>> y(fleetsNum, vector<vector<int>>(timeHor, vector<int>(workspace.maxNodeId() + 1, 0)));
	vector<vector<vector<int>>> z = y;

	clock_t start = clock();

	double R = - solvePFSF(workspace, rewards, init, x, y ,z);

	double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

	cout << "time:" <<  duration << endl;
	cout << "reward:" <<  R << endl;
}
