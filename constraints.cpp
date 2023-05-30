#include "constraints.h"

Constraints::Constraints(int width, int height, int length)
{
    safe_intervals.resize(length);
    for(int i = 0; i < length; i++)
    {
        safe_intervals[i].resize(height);
        for(int j = 0; j < height; j++)
        {
            safe_intervals[i][j].resize(width);
            for(int k = 0; k < width; k++)
            {
                safe_intervals[i][j][k].resize(0);
                safe_intervals[i][j][k].push_back({0,CN_INFINITY});
            }
        }
    }
    constraints.resize(length);
    for(int i = 0; i < length; i++)
    {
        constraints[i].resize(height);
        for(int j = 0; j < height; j++)
        {
            constraints[i][j].resize(width);
            for(int k = 0; k < width; k++)
                constraints[i][j][k].resize(0);
        }
    }
}

bool sort_function(std::pair<double, double> a, std::pair<double, double> b)
{
    return a.first < b.first;
}

double Constraints::minDist(Point A, Point C, Point D)
{
    double AC_len = A.segmentLength(C);
    double AD_len = A.segmentLength(D);
    double CD_len = C.segmentLength(D);
    if(AC_len + AD_len == CD_len) {
        return 0;
    }
    if (C == D) {
        return AC_len;
    }
    Vector3D CA(A.i - C.i, A.j - C.j, A.k - C.k);
    Vector3D CD(D.i - C.i, D.j - C.j, D.k - C.k);
    double cosACD = CA.findCos(CD);
    if (cosACD <= 0) {
        return AC_len;
    }
    Vector3D DA(A.i - D.i, A.j - D.j, A.k - D.k);
    Vector3D DC(C.i - D.i, C.j - D.j, C.k - D.k);
    double cosADC = DA.findCos(DC);
    if (cosADC <= 0) {
        return AD_len;
    }
    double p = (AC_len + AD_len + CD_len) / 2;
    return 2 / CD_len * sqrt(p * (p - CD_len) * (p - AC_len) * (p - AD_len));
}

void Constraints::resetSafeIntervals(int width, int height, int length)
{
    safe_intervals.resize(length);
    for(int i = 0; i < length; i++)
    {
        safe_intervals[i].resize(height);
        for(int j = 0; j < height; j++)
        {
            safe_intervals[i][j].resize(width);
            for(int k = 0; k < width; k++)
            {
                safe_intervals[i][j][k].resize(0);
                safe_intervals[i][j][k].push_back({0,CN_INFINITY});
            }
        }
    }
}

void Constraints::updateCellSafeIntervals(cell cur_cell)
{
    if(safe_intervals[cur_cell.i][cur_cell.j][cur_cell.k].size() > 1)
        return;
    LineOfSight los(agentsize);
    std::vector<cell> cells = los.getCells(cur_cell.i, cur_cell.j, cur_cell.k);
    std::vector<section> secs;
    for(int k = 0; k < cells.size(); k++)
        for(int l = 0; l < constraints[cells[k].i][cells[k].j][cells[k].k].size(); l++)
            if(std::find(secs.begin(), secs.end(), constraints[cells[k].i][cells[k].j][cells[k].k][l]) == secs.end())
                secs.push_back(constraints[cells[k].i][cells[k].j][cells[k].k][l]);

    for(int k = 0; k < secs.size(); k++)
    {
        section sec = secs[k];
        double radius = agentsize + sec.size;
        int i0(secs[k].i1), j0(secs[k].j1), k0(secs[k].k1), i1(secs[k].i2), j1(secs[k].j2), k1(secs[k].k2), i2(cur_cell.i), j2(cur_cell.j), k2(cur_cell.k);
        SafeInterval interval;
        double dist, mindist;
        if(i0 == i1 && j0 == j1 && k0 == k1 && i0 == i2 && j0 == j2 && k0 == k2)
            mindist = 0;
        else
            mindist = minDist(Point(i2,j2,k2), Point(i0,j0,k0), Point(i1,j1,k1));
        if(mindist >= radius)
            continue;
        Point point(i2,j2,k2), p0(i0,j0,k0), p1(i1,j1,k1);
        double AC_len = point.segmentLength(p0);
        double AD_len = point.segmentLength(p1);
        double CD_len = p0.segmentLength(p1);
        double p = (AC_len + AD_len + CD_len) / 2;
        dist = 2 / CD_len * sqrt(p * (p - CD_len) * (p - AC_len) * (p - AD_len));
        int da = (i0 - i2)*(i0 - i2) + (j0 - j2)*(j0 - j2) + (k0 - k2)*(k0 - k2);
        int db = (i1 - i2)*(i1 - i2) + (j1 - j2)*(j1 - j2) + (k1 - k2)*(k1 - k2);
        double ha = sqrt(da - dist*dist);
        double size = sqrt(radius*radius - dist*dist);
        Vector3D CA(i2 - i0, j2 - j0, k2 - k0);
        Vector3D CD(i1 - i0, j1 - j0, k1 - k0);
        double cosACD = CA.findCos(CD);
        Vector3D DA(i2 - i1, j2 - j1, k2 - k1);
        Vector3D DC(i0 - i1, j0 - j1, k0 - k1);
        double cosADC = DA.findCos(DC);
        if (cosACD <= 0) {
            interval.begin = sec.g1;
            interval.end = sec.g1 + (size - ha)/sec.mspeed;
        }
        else if(cosADC <= 0)
        {
            interval.begin = sec.g2 - size/sec.mspeed + sqrt(db - dist*dist)/sec.mspeed;
            interval.end = sec.g2;
        }
        else if(da < radius*radius)
        {
            if(db < radius*radius)
            {
                interval.begin = sec.g1;
                interval.end = sec.g2;
            }
            else
            {
                double hb = sqrt(db - dist*dist);
                interval.begin = sec.g1;
                interval.end = sec.g2 - hb/sec.mspeed + size/sec.mspeed;
            }
        }
        else
        {
            if(db < radius*radius)
            {
                interval.begin = sec.g1 + ha/sec.mspeed - size/sec.mspeed;
                interval.end = sec.g2;
            }
            else
            {
                interval.begin = sec.g1 + ha/sec.mspeed - size/sec.mspeed;
                interval.end = sec.g1 + ha/sec.mspeed + size/sec.mspeed;
            }
        }
        for(unsigned int j = 0; j < safe_intervals[i2][j2][k2].size(); j++)
        {
            if(safe_intervals[i2][j2][k2][j].begin < interval.begin + CN_EPSILON && safe_intervals[i2][j2][k2][j].end + CN_EPSILON > interval.begin)
            {
                if(fabs(safe_intervals[i2][j2][k2][j].begin - interval.begin) < CN_EPSILON)
                {
                    safe_intervals[i2][j2][k2].insert(safe_intervals[i2][j2][k2].begin() + j, SafeInterval(safe_intervals[i2][j2][k2][j].begin,safe_intervals[i2][j2][k2][j].begin));
                    j++;
                    if(safe_intervals[i2][j2][k2][j].end < interval.end)
                        safe_intervals[i2][j2][k2].erase(safe_intervals[i2][j2][k2].begin() + j);
                    else
                        safe_intervals[i2][j2][k2][j].begin = interval.end;
                }
                else if(safe_intervals[i2][j2][k2][j].end < interval.end)
                    safe_intervals[i2][j2][k2][j].end = interval.begin;
                else
                {
                    std::pair<double,double> new1, new2;
                    new1.first = safe_intervals[i2][j2][k2][j].begin;
                    new1.second = interval.begin;
                    new2.first = interval.end;
                    new2.second = safe_intervals[i2][j2][k2][j].end;
                    safe_intervals[i2][j2][k2].erase(safe_intervals[i2][j2][k2].begin() + j);
                    if(new2.first < CN_INFINITY)
                        safe_intervals[i2][j2][k2].insert(safe_intervals[i2][j2][k2].begin() + j, SafeInterval(new2.first, new2.second));
                    safe_intervals[i2][j2][k2].insert(safe_intervals[i2][j2][k2].begin() + j, SafeInterval(new1.first, new1.second));
                }
            }
            else if(safe_intervals[i2][j2][k2][j].begin > interval.begin - CN_EPSILON && safe_intervals[i2][j2][k2][j].begin < interval.end)
            {
                if(fabs(safe_intervals[i2][j2][k2][j].begin - interval.begin) < CN_EPSILON)
                {
                    safe_intervals[i2][j2][k2].insert(safe_intervals[i2][j2][k2].begin() + j, SafeInterval(safe_intervals[i2][j2][k2][j].begin,safe_intervals[i2][j2][k2][j].begin));
                    j++;
                }
                if(safe_intervals[i2][j2][k2][j].end < interval.end)
                {
                    safe_intervals[i2][j2][k2].erase(safe_intervals[i2][j2][k2].begin() + j);
                }
                else
                {
                    safe_intervals[i2][j2][k2][j].begin = interval.end;
                }
            }
        }
        for(unsigned int j = 0; j < safe_intervals[i2][j2][k2].size(); j++)
            safe_intervals[i2][j2][k2][j].id = j;
    }
}

std::vector<SafeInterval> Constraints::getSafeIntervals(Node curNode, const std::unordered_multimap<int, Node> &close, int h, int w)
{
    std::vector<SafeInterval> intervals(0);
    auto range = close.equal_range(curNode.i*h*w + curNode.j*w + curNode.k);
    for(unsigned int i = 0; i < safe_intervals[curNode.i][curNode.j][curNode.k].size(); i++)
        if(safe_intervals[curNode.i][curNode.j][curNode.k][i].end >= curNode.g
                && safe_intervals[curNode.i][curNode.j][curNode.k][i].begin <= (curNode.Parent->interval.end + curNode.g - curNode.Parent->g))
        {
            bool has = false;
            for(auto it = range.first; it != range.second; it++)
                if(it->second.interval.begin == safe_intervals[curNode.i][curNode.j][curNode.k][i].begin)
                if((it->second.g + tweight*fabs(curNode.heading - it->second.heading)/(180*rspeed)) - curNode.g < CN_EPSILON)//take into account turning cost
                {
                    has = true;
                    break;
                }
            if(!has)
                intervals.push_back(safe_intervals[curNode.i][curNode.j][curNode.k][i]);
        }
    return intervals;
}

std::vector<SafeInterval> Constraints::getSafeIntervals(Node curNode)
{
    return safe_intervals[curNode.i][curNode.j][curNode.k];
}

void Constraints::addStartConstraint(int i, int j, int k, int size, std::vector<cell> cells, double agentsize) // sure
{
    section sec(i, j, k, i, j, k, 0, size);
    sec.size = agentsize;
    for(auto cell: cells)
        constraints[cell.i][cell.j][cell.k].insert(constraints[cell.i][cell.j][cell.k].begin(),sec);
    return;
}

void Constraints::removeStartConstraint(std::vector<cell> cells, int start_i, int start_j, int start_k)
{
    for(auto cell: cells)
        for(size_t k = 0; k < constraints[cell.i][cell.j][cell.k].size(); k++)
            if(constraints[cell.i][cell.j][cell.k][k].i1 == start_i && constraints[cell.i][cell.j][cell.k][k].j1 == start_j
            && constraints[cell.i][cell.j][cell.k][k].k1 == start_k && constraints[cell.i][cell.j][cell.k][k].g1 < CN_EPSILON)
            {
                constraints[cell.i][cell.j][cell.k].erase(constraints[cell.i][cell.j][cell.k].begin() + k);
                k--;
            }
    return;
}

void Constraints::addConstraints(const std::vector<Node> &sections, double size, double mspeed, const Map &map)
{
    std::vector<cell> cells;
    LineOfSight los(size);
    section sec(sections.back(), sections.back());
    sec.g2 = CN_INFINITY;
    sec.size = size;
    sec.mspeed = mspeed;
    cells = los.getCellsCrossedByLine(sec.i1, sec.j1, sec.k1, sec.i2, sec.j2, sec.k2, map);
    for(auto cell: cells)
        constraints[cell.i][cell.j][cell.k].push_back(sec);
    if(sec.g1 == 0)
        for(auto cell: cells)
            safe_intervals[cell.i][cell.j][cell.k].clear();
    for(unsigned int a = 1; a < sections.size(); a++)
    {
        cells = los.getCellsCrossedByLine(sections[a-1].i, sections[a-1].j, sections[a-1].k, sections[a].i, sections[a].j, sections[a].k, map);
        sec = section(sections[a-1], sections[a]);
        sec.size = size;
        sec.mspeed = mspeed;
        for(unsigned int i = 0; i < cells.size(); i++)
            constraints[cells[i].i][cells[i].j][cells[i].k].push_back(sec);
        /*if(a+1 == sections.size())
            updateSafeIntervals(cells,sec,true);
        else
            updateSafeIntervals(cells,sec,false);*/
    }
}

std::vector<SafeInterval> Constraints::findIntervals(Node curNode, std::vector<double> &EAT, const std::unordered_multimap<int, Node> &close, const Map &map)
{
    std::vector<SafeInterval> curNodeIntervals = getSafeIntervals(curNode, close, map.height, map.width);
    if(curNodeIntervals.empty())
        return curNodeIntervals;
    EAT.clear();
    LineOfSight los(agentsize);
    std::vector<cell> cells = los.getCellsCrossedByLine(curNode.i, curNode.j, curNode.k, curNode.Parent->i, curNode.Parent->j, curNode.Parent->k, map);
    std::vector<section> sections(0);
    section sec;
    for(unsigned int i = 0; i < cells.size(); i++)
        for(unsigned int j = 0; j < constraints[cells[i].i][cells[i].j][cells[i].k].size(); j++)
        {
            sec = constraints[cells[i].i][cells[i].j][cells[i].k][j];
            if(sec.g2 < curNode.Parent->g || sec.g1 > (curNode.Parent->interval.end + curNode.g - curNode.Parent->g))
                continue;
            if(std::find(sections.begin(), sections.end(), sec) == sections.end())
                sections.push_back(sec);
        }
    auto range = close.equal_range(curNode.i*map.width*map.height + curNode.j*map.width + curNode.k);

    for(unsigned int i=0; i<curNodeIntervals.size(); i++)
    {
        SafeInterval cur_interval(curNodeIntervals[i]);
        if(cur_interval.begin < curNode.g)
            cur_interval.begin = curNode.g;
        double startTimeA = curNode.Parent->g;
        if(cur_interval.begin > startTimeA + curNode.g - curNode.Parent->g)
            startTimeA = cur_interval.begin - curNode.g + curNode.Parent->g;
        unsigned int j = 0;
        bool goal_collision;
        while(j < sections.size())
        {
            goal_collision = false;

            if(hasCollision(curNode, startTimeA, sections[j], goal_collision))
            {
                double offset = 1.0;
                startTimeA += offset;
                cur_interval.begin += offset;
                j = 0;//start to check all constraints again, because time has changed
                if(goal_collision || cur_interval.begin > cur_interval.end || startTimeA > curNode.Parent->interval.end)
                {
                    curNodeIntervals.erase(curNodeIntervals.begin() + i);
                    i--;
                    break;
                }
            }
            else
                j++;
        }
        if(j == sections.size())
        {
            bool has = false;
            for(auto rit = range.first; rit != range.second; rit++)
                if(rit->second.interval.begin == curNodeIntervals[i].begin)
                if((rit->second.g + tweight*fabs(curNode.heading - rit->second.heading)/(180*rspeed) - cur_interval.begin) < CN_EPSILON)//take into account turning cost
                {
                    has = true;
                    curNodeIntervals.erase(curNodeIntervals.begin()+i);
                    i--;
                    break;
                }
            if(!has)
                EAT.push_back(cur_interval.begin);
        }
    }
    return curNodeIntervals;
}

bool Constraints::hasCollision(const Node &curNode, double startTimeA, const section &constraint, bool &goal_collision)
{
    double endTimeA(startTimeA + curNode.g - curNode.Parent->g), startTimeB(constraint.g1), endTimeB(constraint.g2);
    if(startTimeA > endTimeB || startTimeB > endTimeA)
        return false;
    Vector3D A(curNode.Parent->i,curNode.Parent->j, curNode.Parent->k);
    Vector3D VA((curNode.i - curNode.Parent->i)/(curNode.g - curNode.Parent->g), (curNode.j - curNode.Parent->j)/(curNode.g - curNode.Parent->g), (curNode.k - curNode.Parent->k)/(curNode.g - curNode.Parent->g));
    Vector3D B(constraint.i1, constraint.j1, constraint.k1);
    Vector3D VB((constraint.i2 - constraint.i1)/(constraint.g2 - constraint.g1), (constraint.j2 - constraint.j1)/(constraint.g2 - constraint.g1), (constraint.k2 - constraint.k1)/(constraint.g2 - constraint.g1));
    if(startTimeB > startTimeA)
    {
      // Move A to the same time instant as B
      A += VA*(startTimeB-startTimeA);
      startTimeA=startTimeB;
    }
    else if(startTimeB < startTimeA)
    {
      B += VB*(startTimeA - startTimeB);
      startTimeB = startTimeA;
    }
    double r(constraint.size + agentsize + inflateintervals); //combined radius
    Vector3D w(B - A);
    double c(w*w - r*r);
    if(c < 0)
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    } // Agents are currently colliding

    // Use the quadratic formula to detect nearest collision (if any)
    Vector3D v(VA - VB);
    double a(v*v);
    double b(w*v);

    double dscr(b*b - a*c);
    if(dscr <= 0)
        return false;

    double ctime = (b - sqrt(dscr))/a;
    if(ctime > -CN_EPSILON && ctime < std::min(endTimeB,endTimeA) - startTimeA + CN_EPSILON)
    {
        if(constraint.g2 == CN_INFINITY)
            goal_collision = true;
        return true;
    }
    else
        return false;
}
