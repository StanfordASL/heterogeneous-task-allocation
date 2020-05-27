#include <iostream>
#include <iostream>
#include <cstdio>
#include <ctime>
//#include <lemon/list_graph.h>
#include <lemon/smart_graph.h>
#include <lemon/network_simplex.h>
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

using Weight = double;
using Capacity = int;

using NS = NetworkSimplex<Graph, Capacity, Weight>;

// returns solution for private task of a specific fleet
void solvePrivate(const Graph& workspace, const vector<vector<double>>& rewards, const vector<int>& init, vector<vector<int>>& x,  vector<vector<int>>& y)
{
	int agents = init.size();
	
	Graph g; // flow graph
	ArcMap<int> capacity(g, agents); // capacity of edges
	ArcMap<double> cost(g, 0); // cost of edges
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
				cost[a] = rewards[t][v];
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
	NS::ProblemType status = ns.run();
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
				if (y[f][t][v] == 1)
					R += rewards[fleetsNum][t][v];
				if (z[f][t][v] == 1)
					R += rewards[f][t][v];
			}

	return R;
}

double solveSharedFirst(const Graph& workspace, const vector<vector<vector<double>>>& rewards, const vector<vector<int>>& init,  vector<vector<vector<int>>>& x, vector<vector<vector<int>>>& y, vector<vector<vector<int>>>& z)
{
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
				rewards[type][t][v] = value;
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

int main()
{
	string path = "./../data";
	Graph workspace;
	vector<vector<vector<double>>> rewards;
	vector<vector<int>> init;

	int timeHor = importer(path, workspace, rewards, init);

	int fleetsNum = init.size();

	vector<vector<vector<int>>> x(fleetsNum, vector<vector<int>>(timeHor-1, vector<int>(workspace.maxArcId() + 1, 0)));
	vector<vector<vector<int>>> y(fleetsNum, vector<vector<int>>(timeHor, vector<int>(workspace.maxNodeId() + 1, 0)));
	vector<vector<vector<int>>> z = y;

	clock_t start = clock();

	double R = solvePFSF(workspace, rewards, init, x, y ,z);

	double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    cout << "running time in seconds: " << duration << endl;
	cout << "solution to PFSF with reward " << R << endl;

	/*
	cout << "x values (fleet, time, origin, destination, value)" << endl;
	for (unsigned int f = 0; f < rewards.size()-1; f++)
		for (auto t=0; t < timeHor-1; t++)
			for (auto ai = 0; ai < workspace.maxArcId()+1; ai++)
				if (x[f][t][ai] != 0)
				{
					Arc a = workspace.arcFromId(ai);
					int o = workspace.id(workspace.source(a));
					int d = workspace.id(workspace.target(a));
					cout << f << " " << t << " " << o << " " << d << " " << x[f][t][ai] << endl;
				}

	cout << "y values (fleet, time, vertex, value)" << endl;
	for (unsigned int f = 0; f < rewards.size()-1; f++)
		for (int t = 0; t < timeHor; t++)
			for (int v = 0; v < workspace.maxNodeId()+1; v++)
				if (y[f][t][v]!=0)
					cout << f << " " << t << " " << v << " " << y[f][t][v] << endl;
	
	cout << "z values (fleet, time, vertex, value)" << endl;
	for (unsigned int f = 0; f < rewards.size()-1; f++)
		for (int t = 0; t < timeHor; t++)
			for (int v = 0; v < workspace.maxNodeId()+1; v++)
				if (z[f][t][v]!=0)
					cout << f << " " << t << " " << v << " " << z[f][t][v] << endl;
	
	return 0; 
	*/

	/*
	vector<vector<int>> x(timeHor-1, vector<int>(workspace.maxArcId()+1, 0)); // movement vector (time / edge)
	vector<vector<int>> y(timeHor, vector<int>(workspace.maxNodeId()+1, 0)); // asignemnt vector (time / vertex)
						  
	solvePrivate(workspace, rewards[0], init[0], x, y);

	cout << "x values (time, origin, destination, value)" << endl;
	for (int t=0; t < timeHor-1; t++)
		for (int ai = 0; ai < workspace.maxArcId()+1; ai++)
			if (x[t][ai] != 0)
			{
				Arc a = workspace.arcFromId(ai);
				int o = workspace.id(workspace.source(a));
				int d = workspace.id(workspace.target(a));
				cout << t << " " << o << " " << d << " " << x[t][ai] << endl;
			}

	cout << "y values (time, vertex, value)" << endl;
	for (int t = 0; t < timeHor; t++)
		for (int v = 0; v < workspace.maxNodeId()+1; v++)
			if (y[t][v]!=0)
				cout << t << " " << v << " " << y[t][v] << endl;

	*/


	/*vector<vector<vector<int>>> x_shared(fleetsNum, vector<vector<int>>(timeHor-1, vector<int>(workspace.maxArcId() + 1, 0))); // movement vector (time / edge
	vector<vector<vector<int>>> z_shared(fleetsNum, vector<vector<int>>(timeHor, vector<int>(workspace.maxNodeId() + 1, 0))); // asignemnt vector (time / vertex)
	
	solveShared(workspace, rewards[fleetsNum], init, x_shared, z_shared);

	cout << "shared" << endl;

	cout << "x values (fleet, time, origin, destination, value)" << endl;
	for (unsigned int f = 0; f < rewards.size()-1; f++)
		for (auto t=0; t < timeHor-1; t++)
			for (auto ai = 0; ai < workspace.maxArcId()+1; ai++)
				if (x_shared[f][t][ai] != 0)
				{
					Arc a = workspace.arcFromId(ai);
					int o = workspace.id(workspace.source(a));
					int d = workspace.id(workspace.target(a));
					cout << f << " " << t << " " << o << " " << d << " " << x_shared[f][t][ai] << endl;
				}

	cout << "z values (fleet, time, vertex, value)" << endl;
	for (unsigned int f = 0; f < rewards.size()-1; f++)
		for (int t = 0; t < timeHor; t++)
			for (int v = 0; v < workspace.maxNodeId()+1; v++)
				if (z_shared[f][t][v]!=0)
					cout << f << " " << t << " " << v << " " << z_shared[f][t][v] << endl;
	
					return 0; */
}

int main_example()
{
	Graph g;

	// add nodes
	for (unsigned int i = 0; i < 4; i++)
		g.addNode();

	// add edges with node ids
	g.addArc(g.nodeFromId(0),g.nodeFromId(1));
	g.addArc(g.nodeFromId(0),g.nodeFromId(2));
	g.addArc(g.nodeFromId(1),g.nodeFromId(3));
	g.addArc(g.nodeFromId(1),g.nodeFromId(2));
	g.addArc(g.nodeFromId(3),g.nodeFromId(0));
	g.addArc(g.nodeFromId(2),g.nodeFromId(3));

	// using arc iterators 
	NodeMap<int> out_deg(g, 0);
	for (ArcIt a(g); a != INVALID; ++a)
		out_deg[g.source(a)]++;

	cout << "This is the outdegree of the graph." << endl;
	// using node iterators
	for (NodeIt i_n(g); i_n !=INVALID; ++i_n)
		cout << g.id(i_n) << " " << out_deg[i_n] << endl;

	// adding edge weights
	ArcMap<double> length(g, 1.0);
	for (ArcIt a(g); a != INVALID; ++a)
	{
		if (g.id(g.source(a)) == 0 && g.id(g.target(a))==2)
			length[a] = 3.0;
		if (g.id(g.source(a)) == 2 && g.id(g.target(a))==3)
			length[a] = 8.0;
		if (g.id(g.source(a)) == 1 && g.id(g.target(a))==3)
			length[a] = 5.0;
	}

	// shortest path between two nodes using dijkstra
	Dijkstra<Graph,ArcMap<double>> dij(g, length);
	// distance map to be updated by dij
	NodeMap<double> dist(g, 15000.0);

	// start and target nodes
	Node s = g.nodeFromId(0);
	Node t = g.nodeFromId(3);
	
	dij.distMap(dist);
	dij.run(s,t);
	
	cout << "distance from 0 to 3 = " << dij.dist(t) << endl;

	Node c_n = t;
	
	cout << "shortest path from 0 to 3" << endl;
	cout << "3" << endl;

	// extract shortest path (in reverse order)
	while (c_n != s)
	{
		Arc in_arc =  dij.predMap()[c_n];
		c_n = g.source(in_arc);
		
		cout << g.id(c_n) << endl;
	}


	// computing min-cost flow
	ArcMap<Capacity> capacities(g, 2);
	ArcMap<Weight> &weights = length;
	
	NS ns(g);
	// initialize mcf solver with source / target nodes
	// and total flow coming from source
	ns.costMap(weights).upperMap(capacities).stSupply(s, t, 4);

	// solution will be updated in the following vector
	ArcMap<Capacity> flows(g);
	NS::ProblemType status = ns.run();
	switch (status) {
	case NS::INFEASIBLE:
		cerr << "insufficient flow" << endl;
		break;
	case NS::OPTIMAL:
		ns.flowMap(flows);
		
		/*for (ArcIt a(g); a != INVALID; ++a)
			cout << "flow on " << g.id(a) << " = " << flows[a] << endl;
		
			cerr << "cost=" << ns.totalCost() << endl; */
		break;
	case NS::UNBOUNDED:
		cerr << "infinite flow" << endl;
		break;
	default:
		break;
	}

	return 0;
}

/*
  Example using static graph structure
 */

  /*
  std::vector<std::pair<int,int> > arcs;
  arcs.push_back(std::make_pair(0,1));
  arcs.push_back(std::make_pair(0,2));
  arcs.push_back(std::make_pair(1,3));
  arcs.push_back(std::make_pair(1,2));
  arcs.push_back(std::make_pair(3,0));
  arcs.push_back(std::make_pair(2,3));
  StaticDigraph sg;
  sg.build(4, arcs.begin(), arcs.end());*/

  /*StaticDigraph::NodeMap<int> out_deg(sg, 0);
  for (StaticDigraph::ArcIt a(sg); a != INVALID; ++a)
	  out_deg[sg.source(a)]++;

  StaticDigraph::ArcMap<double> length(sg, 1.0);
  for (StaticDigraph::ArcIt a(sg); a != INVALID; ++a)
  {
	  if (sg.id(sg.source(a)) == 0 && sg.id(sg.target(a))==2)
		  length[a] = 3.0;
	  if (sg.id(sg.source(a)) == 2 && sg.id(sg.target(a))==3)
		  length[a] = 2.0;
	  if (sg.id(sg.source(a)) == 1 && sg.id(sg.target(a))==3)
		  length[a] = 5.0;
  }
  
  cout << "Hello World! This is LEMON library here." << endl;
  cout << "We have a directed graph with " << countNodes(sg) << " nodes "
       << "and " << countArcs(sg) << " arc." << endl;

  cout << "This is the outdegree of the graph." << endl;
  for (StaticDigraph::NodeIt i_n(sg); i_n !=INVALID; ++i_n)
	  cout << sg.id(i_n) << " " << out_deg[i_n] << endl;

  Dijkstra<StaticDigraph,StaticDigraph::ArcMap<double>> dij(sg, length);
  StaticDigraph::NodeMap<double> dist(sg, 15000.0);

  StaticDigraph::Node s = sg.nodeFromId(0);
  StaticDigraph::Node t = sg.nodeFromId(3);
  
  dij.distMap(dist);
  dij.init();
  dij.addSource(s);
  dij.start(); */
  
  //dij.predMap(preds);
