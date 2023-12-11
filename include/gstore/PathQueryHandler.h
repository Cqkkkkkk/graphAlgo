// TODO: Convert all IDs to unsigned to incorporate more data?

#include "CSR.h"
#include <sstream>

#ifndef _QUERY_PATH_H
#define _QUERY_PATH_H

class PathQueryHandler
{
private:
	CSR *csr;
	std::unordered_map<int, std::set<int> > distinctInEdges;
	std::unordered_map<int, std::set<int> > distinctOutEdges;
	int cacheMaxSize;
	int n, m;	// #vertices, #edges
public:
	PathQueryHandler(CSR *_csr);
	
	~PathQueryHandler();

	void inputGraph(std::string filename);	// Read in a graph FOR TESTING. Graph file format:
						// First line: #vertices #labels
						// Each following line: u v label
	void printCSR();	// Feel free to modify this for testing
	// void generateQueries(int queryType, bool directed, int numQueries, vector<pair<int, int>, int>& queries);

	// Labeled graph interface: edge (u, v, pred)

	int getVertNum();	// Get total number of vertices
	int getEdgeNum();	// Get total number of edges
    int getSetEdgeNum(const std::vector<int> &pred_set);    // TODO: Get the number of edges labeled by preds in pred_set
	int getInIndexByID(int vid, int pred);	// Get vertice's index in csr[1] offset_list
	int getInSize(int vid, int pred);	// Get the number of in-neighbors of vert
						// linked by edges labeled by pred
	int getInVertID(int vid, int pred, int pos);	// Get the pos-th in-neighbor of vert
						// linked by an edge labeled by pred
	int getInVertID(int vid, int pos);	// Get in-neighbor regardless of pred;
						// corresponds to getTotalInSize with distinct
	int getSetInSize(int vid, const std::vector<int> &pred_set);	// Get number of in-neighbors by pred_set
	int getTotalInSize(int vid, bool distinct);	// Get the total number of in-neighbors of vert
						// distinct will eliminate repetitive occurrences of same in-neighbor with different label edges

	int getOutIndexByID(int vid, int pred);
	int getOutSize(int vid, int pred);
	int getOutVertID(int vid, int pred, int pos);
	int getOutVertID(int vid, int pos);
	int getSetOutSize(int vid, const std::vector<int> &pred_set);
	int getTotalOutSize(int vid, bool distinct);

	std::string getPathString(std::vector<int> &path_set);
    std::string getJSONArray(std::unordered_map<int, int> &map);
private:

};

#endif //_QUERY_PATH_H