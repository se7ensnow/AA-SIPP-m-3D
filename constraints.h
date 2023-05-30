#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <vector>
#include <unordered_map>
#include "gl_const.h"
#include "structs.h"
#include <algorithm>
#include <iostream>
#include <lineofsight.h>
#include "map.h"

class Constraints
{
public:
    Constraints(int width, int height, int length);
    ~Constraints(){}
    void updateCellSafeIntervals(cell cur_cell);
    std::vector<SafeInterval> getSafeIntervals(Node curNode, const std::unordered_multimap<int, Node> &close, int w, int h);
    std::vector<SafeInterval> getSafeIntervals(Node curNode);
    void addConstraints(const std::vector<Node> &sections, double size, double mspeed, const Map &map);
    std::vector<SafeInterval> findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, const Map &map);
    SafeInterval getSafeInterval(int i, int j, int k, int n) {return safe_intervals[i][j][k][n];}
    void resetSafeIntervals(int width, int height, int length);
    void addStartConstraint(int i, int j, int k, int size, std::vector<cell> cells, double agentsize = 0.5);
    void removeStartConstraint(std::vector<cell> cells, int start_i, int start_j, int start_k);
    void setSize(double size) {agentsize = size;}
    void setParams(double size, double mspeed, double rspeed, double tweight, double inflateintervals)
    { agentsize = size; this->mspeed = mspeed; this->rspeed = rspeed; this->tweight = tweight; this->inflateintervals = inflateintervals; }
    double minDist(Point A, Point C, Point D);


private:
    bool hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision);
    std::vector<std::vector<std::vector<std::vector<section>>>> constraints;
    std::vector<std::vector<std::vector<std::vector<SafeInterval>>>> safe_intervals;
    double rspeed;
    double mspeed;
    double agentsize;
    double tweight;
    double inflateintervals;

};


#endif // CONSTRAINTS_H
