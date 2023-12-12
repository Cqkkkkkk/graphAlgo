#include <iostream>
#include <unordered_set>
#include <vector>

#include "../include/gstore/PathQueryHandler.h"

using namespace std;

extern "C" string labelprop(std::vector<int> iri_set, bool directed, int k,
                            std::vector<int> pred_set,
                            PathQueryHandler* queryUtil) {}

extern "C" string cyclepath(std::vector<int> iri_set, bool directed,
                            std::vector<int> pred_set,
                            PathQueryHandler* queryUtil) {}

extern "C" string clusteringcoeff(std::vector<int> iri_set, bool directed,
                                  int k, std::vector<int> pred_set,
                                  PathQueryHandler* queryUtil) {}

unsigned int distFunc(int vid, std::unordered_map<int, int>& dist) {
    if (dist.find(vid) == dist.end()) {
        return -1;
    }
    return dist[vid];
}

extern "C" string sssp(std::vector<int> iri_set, bool directed,
                       std::vector<int> pred_set, PathQueryHandler* queryUtil) {
    // dijkstra
    std::unordered_map<int, int> dist;
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
                        std::greater<std::pair<int, int>>>
        que;
    dist[iri_set[0]] = 0;
    que.push(std::make_pair(0, iri_set[0]));
    while (!que.empty()) {
        std::pair<int, int> p = que.top();
        que.pop();
        int from_vid = p.second;
        int d = p.first;
        if (distFunc(from_vid, dist) < d) {
            continue;
        }
        for (int pred : pred_set) {
            for (int pos = 0; pos < queryUtil->getOutSize(from_vid, pred);
                 ++pos) {
                int to_vid = queryUtil->getOutVertID(from_vid, pred, pos);
                if (distFunc(to_vid, dist) > distFunc(from_vid, dist) + 1) {
                    dist[to_vid] = distFunc(from_vid, dist) + 1;
                    que.push(std::make_pair(dist[to_vid], to_vid));
                }
            }
            if (!directed) {
                for (int pos = 0; pos < queryUtil->getInSize(from_vid, pred);
                     ++pos) {
                    int to_vid = queryUtil->getInVertID(from_vid, pred, pos);
                    if (distFunc(to_vid, dist) > distFunc(from_vid, dist) + 1) {
                        dist[to_vid] = distFunc(from_vid, dist) + 1;
                        que.push(std::make_pair(dist[to_vid], to_vid));
                    }
                }
            }
        }
    }
    dist.erase(iri_set[0]);
    return queryUtil->getJSONArray(dist);
}

bool dfs(int node, int start, std::vector<std::pair<int, int>>& path,
         std::unordered_set<int>& visited, PathQueryHandler* queryUtil,
         const std::vector<int>& pred_set, bool directed,
         std::unordered_map<int, std::pair<int, int>>& parent) {

    if (visited.count(node)) {
        if (node == start && parent.find(node) != parent.end()) {
            auto parent_info = parent[node];
            // path.push_back({node, parent_info.second});  // Add the starting edge to complete the cycle
            return true;  // Cycle found
        }
        return false;  // Visited but not the start node
    }

    visited.insert(node);

    for (int pred : pred_set) {
        int outSize = queryUtil->getOutSize(node, pred);
        for (int pos = 0; pos < outSize; ++pos) {
            int neighbor = queryUtil->getOutVertID(node, pred, pos);
            if (neighbor != node &&
                (directed || parent.find(node) == parent.end() ||
                 parent[node].first != neighbor)) {
                path.push_back({node, pred});  // Add the edge to the path
                parent[neighbor] = {node, pred};  // Store both parent node and predicate
                if (dfs(neighbor, start, path, visited, queryUtil, pred_set,
                        directed, parent)) {
                    return true;  // Cycle found in recursion
                }
                parent.erase(neighbor);  // Remove the parent mark (backtracking)
                path.pop_back();  // Backtrack if no cycle found
            }
        }
    }

    return false;
}


std::string pathToString(const std::vector<std::pair<int, int>>& path) {
    std::stringstream ss;
    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0) ss << ",";
        ss << path[i].first << "," << path[i].second;
        if (i + 1 < path.size()) 
            ss << "," << path[i + 1].first;
        else
            ss << "," << path[0].first;
        // ss << "    @    ";
    }
    return ss.str();
}

extern "C" string cyclePath(std::vector<int> iri_set, bool directed,
                            std::vector<int> pred_set,
                            PathQueryHandler* queryUtil) {
    for (int start_node : iri_set) {
        std::vector<std::pair<int, int>> cycle_path;
        std::unordered_set<int> visited;
        std::unordered_map<int, std::pair<int, int>> parent;

        if (dfs(start_node, start_node, cycle_path, visited, queryUtil,
                pred_set, directed, parent)) {
            return pathToString(cycle_path);
        }
    }

    return "";  // No cycle found
}

PathQueryHandler getQueryUtil() {
    CSR* csr = new CSR[2];
    // OUT
    csr[0].init(3);
    csr[0].id2vid[0] = {0, 1, 3, 5};
    csr[0].id2vid[1] = {0, 1, 2, 3, 4};
    csr[0].id2vid[2] = {0, 3, 4, 5};
    std::map<unsigned, unsigned> vid2id0;
    std::map<unsigned, unsigned> vid2id1;
    std::map<unsigned, unsigned> vid2id2;
    vid2id0.insert(make_pair(0, 0));
    vid2id0.insert(make_pair(1, 1));
    vid2id0.insert(make_pair(2, 0));
    vid2id0.insert(make_pair(3, 2));
    vid2id0.insert(make_pair(4, 0));
    vid2id0.insert(make_pair(5, 3));
    vid2id1.insert(make_pair(0, 0));
    vid2id1.insert(make_pair(1, 1));
    vid2id1.insert(make_pair(2, 2));
    vid2id1.insert(make_pair(3, 3));
    vid2id1.insert(make_pair(4, 4));
    vid2id2.insert(make_pair(0, 0));
    vid2id2.insert(make_pair(1, 0));
    vid2id2.insert(make_pair(2, 0));
    vid2id2.insert(make_pair(3, 1));
    vid2id2.insert(make_pair(4, 2));
    vid2id2.insert(make_pair(5, 3));
    csr[0].vid2id[0] = vid2id0;
    csr[0].vid2id[1] = vid2id1;
    csr[0].vid2id[2] = vid2id2;
    csr[0].offset_list[0] = {0, 1, 2, 3};
    csr[0].offset_list[1] = {0, 1, 2, 3, 4};
    csr[0].offset_list[2] = {0, 1, 2, 4};
    csr[0].adjacency_list[0] = {1, 0, 1, 0, 2};
    csr[0].adjacency_list[1] = {1, 2, 3, 1, 3};
    csr[0].adjacency_list[2] = {2, 4, 2, 5, 4};

    // IN
    csr[1].init(3);
    csr[1].id2vid[0] = {0, 1, 2};
    csr[1].id2vid[1] = {1, 2, 3};
    csr[1].id2vid[2] = {2, 4, 5};
    std::map<unsigned, unsigned> vid2id3;
    std::map<unsigned, unsigned> vid2id4;
    std::map<unsigned, unsigned> vid2id5;
    vid2id3.insert(make_pair(0, 0));
    vid2id3.insert(make_pair(1, 1));
    vid2id3.insert(make_pair(2, 2));
    vid2id4.insert(make_pair(0, 0));
    vid2id4.insert(make_pair(1, 0));
    vid2id4.insert(make_pair(2, 1));
    vid2id4.insert(make_pair(3, 2));
    vid2id5.insert(make_pair(0, 0));
    vid2id5.insert(make_pair(1, 0));
    vid2id5.insert(make_pair(2, 0));
    vid2id5.insert(make_pair(3, 0));
    vid2id5.insert(make_pair(4, 1));
    vid2id5.insert(make_pair(5, 2));
    csr[1].vid2id[0] = vid2id3;
    csr[1].vid2id[1] = vid2id4;
    csr[1].vid2id[2] = vid2id5;
    csr[1].offset_list[0] = {0, 2, 4};
    csr[1].offset_list[1] = {0, 2, 3};
    csr[1].offset_list[2] = {0, 2, 4};
    csr[1].adjacency_list[0] = {1, 5, 0, 3, 5};
    csr[1].adjacency_list[1] = {0, 3, 1, 2, 4};
    csr[1].adjacency_list[2] = {0, 4, 3, 5, 4};
    return PathQueryHandler(csr);
}

int main(int argc, char* argv[]) {
    PathQueryHandler queryUtil = PathQueryHandler(nullptr);
    queryUtil.inputGraph("graph.txt");
    // queryUtil.printCSR();
    std::vector<int> iri_set = {0, 1, 2, 3};
    std::vector<int> pred_set = {0, 1};
    // string rt = sssp(iri_set, true, pred_set, &queryUtil);
    string rt = cyclePath(iri_set, true, pred_set, &queryUtil);

    std::cout << rt << std::endl;
    return 0;
}