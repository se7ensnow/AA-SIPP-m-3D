/* (c) 2017. Andreychuk A.
 * This class implements line-of-sight function for a variable size of agent.
 * It also has a method for checking cell's traversability.
 * For its work is needed the size of agent and a map container that has 'cellIsObstacle' and 'cellOnGrid' methods.
 * If it is not possible to give the permission to access the grid, the one can use 'getCellsCrossedByLine' method.
 * It doesn't use grid and returns a set of all cells(as pairs of coordinates) that are crossed by an agent moving along a line.
 */

#ifndef LINEOFSIGHT_H
#define LINEOFSIGHT_H
#include "gl_const.h"

#define CN_OBSTACLE 1

#include <vector>
#include <math.h>
#include <algorithm>
#include <unordered_map>

class LineOfSight
{
public:
    LineOfSight(double agentSize = 0.5)
    {
        this->agentSize = agentSize;
        int add_x, add_y, add_z, num = agentSize + 0.5 - CN_EPSILON;
        for(int x = -num; x <= +num; x++)
            for(int y = -num; y <= +num; y++)
                for(int z = -num; z <= +num; z++)
                {
                    add_x = x != 0 ? 1 : 0;
                    add_y = y != 0 ? 1 : 0;
                    add_z = z != 0 ? 1 : 0;
                    if((pow(2*abs(x) - add_x, 2) + pow(2*abs(y) - add_y, 2) + pow(2*abs(z) - add_z, 2)) < pow(2*agentSize, 2))
                        cells.push_back({x, y, z});
                }
        if(cells.empty())
            cells.push_back({0,0,0});
    }

    void setSize(double agentSize)
    {
        this->agentSize = agentSize;
        int add_x, add_y, add_z, num = agentSize + 0.5 - CN_EPSILON;
        cells.clear();
        for(int x = -num; x <= +num; x++)
            for(int y = -num; y <= +num; y++)
                for(int z = -num; z <= +num; z++)
                {
                    add_x = x != 0 ? 1 : 0;
                    add_y = y != 0 ? 1 : 0;
                    add_z = z != 0 ? 1 : 0;
                    if((pow(2*abs(x) - add_x, 2) + pow(2*abs(y) - add_y, 2) + pow(2*abs(z) - add_z, 2)) < pow(2*agentSize, 2))
                        cells.push_back({x, y, z});
                }
        if(cells.empty())
            cells.push_back({0,0,0});
    }

    template <class T>
    std::unordered_map<int, std::vector<int>> getCellsCrossedByLineFlat(int x1, int y1, int x2, int y2, const T &map)
    {
        std::unordered_map<int, std::vector<int>> lineCells(0);
        if(x1 == x2 && y1 == y2)
        {
            return lineCells;
        }
        int delta_x = std::abs(x1 - x2);
        int delta_y = std::abs(y1 - y2);
        if((delta_x >= delta_y && x1 > x2) || (delta_y > delta_x && y1 > y2))
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        int step_x = (x1 < x2 ? 1 : -1);
        int step_y = (y1 < y2 ? 1 : -1);
        int error = 0, x = x1, y = y1;
        int k, num;
        std::pair<int, int> add;
        int gap = agentSize*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPSILON;
        int extraCheck = agentSize*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;

        for(int n = 1; n <= extraCheck; n++)
        {
            error += delta_y;
            num = (gap - error)/delta_x;
            for(k = 1; k <= num; k++)
                lineCells[x1 - n*step_x].push_back(y1 + k*step_y);
            for(k = 1; k <= num; k++)
                lineCells[x2 + n*step_x].push_back(y2 - k*step_y);
        }
        error = 0;
        for(x = x1; x != x2 + step_x; x+=step_x)
        {
            lineCells[x].push_back(y);
            if(x < x2 - extraCheck)
            {
                num = (gap + error)/delta_x;
                for(k = 1; k <= num; k++)
                    lineCells[x].push_back(y + k*step_y);
            }
            if(x > x1 + extraCheck)
            {
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    lineCells[x].push_back(y - k*step_y);
            }
            error += delta_y;
            if((error<<1) > delta_x)
            {
                y += step_y;
                error -= delta_x;
            }
        }

        // for(auto it = lineCells.begin(); it != lineCells.end(); it++)
        //     if(!map.CellOnGrid(it->first, it->second))
        //     {
        //         lineCells.erase(it);
        //         it = lineCells.begin();
        //     }
        return lineCells;
    }
    //returns all cells that are affected by agent during moving along a line


    template <class T>
    std::vector<cell> getCellsCrossedByLine(int x1, int y1, int z1, int x2, int y2, int z2, const T &map)
    {
        std::vector<cell> lineCells(0);
        if(x1 == x2 && y1 == y2 && z1 == z2)
        {
            for(auto cell:cells)
                lineCells.push_back({x1+cell.i, y1+cell.j, z1+cell.k});
            return lineCells;
        }
        int delta_x = std::abs(x1 - x2);
        int delta_y = std::abs(y1 - y2);
        int delta_z = std::abs(z1 - z2);
        std::unordered_map<int, std::vector<int>> a_proection, b_proection;
        if (delta_x == std::max(std::max(delta_x, delta_y), delta_z)) {
            a_proection = getCellsCrossedByLineFlat(x1, y1, x2, y2, map);
            b_proection = getCellsCrossedByLineFlat(x1, z1, x2, z2, map);
            for (int i = x1; i <= x2; ++i) {
                for (auto j : a_proection[i]) {
                    for (auto k : b_proection[i]) {
                        lineCells.push_back({i, j, k});
                    }
                }
            }
        } else if (delta_y == std::max(std::max(delta_x, delta_y), delta_z)) {
            a_proection = getCellsCrossedByLineFlat(y1, x1, y2, x2, map);
            b_proection = getCellsCrossedByLineFlat(y1, z1, y2, z2, map);
            for (int j = y1; j <= y2; ++j) {
                for (auto i : a_proection[j]) {
                    for (auto k : b_proection[j]) {
                        lineCells.push_back({i, j, k});
                    }
                }
            }
        } else {
            a_proection = getCellsCrossedByLineFlat(z1, x1, z2, x2, map);
            b_proection = getCellsCrossedByLineFlat(z1, y1, z2, y2, map);
            for (int k = z1; k <= z2; ++k) {
                for (auto i : a_proection[k]) {
                    for (auto j : b_proection[k]) {
                        lineCells.push_back({i, j, k});
                    }
                }
            }
        }
        for(auto cur_cell:cells) {
            cell cur_start = {x1 + cur_cell.i, y1 + cur_cell.j, z1 + cur_cell.k};
            cell cur_end = {x2 + cur_cell.i, y2 + cur_cell.j, z2 + cur_cell.k};
            if (std::find(lineCells.begin(), lineCells.end(), cur_start) == lineCells.end()) {
                lineCells.push_back(cur_start);
            }
            if (std::find(lineCells.begin(), lineCells.end(), cur_end) == lineCells.end()) {
                lineCells.push_back(cur_end);
            }
        }

        for(auto it = lineCells.begin(); it != lineCells.end(); it++)
                 if(!map.CellOnGrid(it->i, it->j, it->k))
                 {
                     lineCells.erase(it);
                     it = lineCells.begin();
                 }

        return lineCells;
    }



    template <class T>
    bool checkTraversability(int x, int y, int z, const T &map)
    {
        for(auto & cell : cells)
            if(!map.CellOnGrid(x + cell.i, y + cell.j, z + cell.k) ||
            map.CellIsObstacle(x + cell.i, y + cell.j, z + cell.k))
                return false;
        return true;
    }
    //checks traversability of all cells affected by agent's body

    template <class T>
    bool checkLine(int x1, int y1, int z1, int x2, int y2, int z2, const T &map)
    {
        if(!checkTraversability(x1, y1, z1, map) || !checkTraversability(x2, y2, z2, map)) //additional check of start and goal traversability,
            return false;                                                                  //it can be removed if they are already checked

        if(x1 == x2 && y1 == y2 && z1 == z2)
        {
            for(auto cell:cells)
                if (!checkTraversability(x1+cell.i, y1+cell.j, z1+cell.k, map)) {
                    return false;
                }
            return true;
        }
        int delta_x = std::abs(x1 - x2);
        int delta_y = std::abs(y1 - y2);
        int delta_z = std::abs(z1 - z2);
        std::unordered_map<int, std::vector<int>> a_proection, b_proection;
        if (delta_x >= delta_y && delta_x >= delta_z) {
            a_proection = getCellsCrossedByLineFlat(x1, y1, x2, y2, map);
            b_proection = getCellsCrossedByLineFlat(x1, z1, x2, z2, map);
            if (x1 > x2) {
                std::swap(x1, x2);
            }
            for (int i = x1; i <= x2; ++i) {
                for (auto j : a_proection[i]) {
                    for (auto k : b_proection[i]) {
                        if (!checkTraversability(i, j, k, map)) {
                            return false;
                        }
                    }
                }
            }
        } else if (delta_y >= delta_x && delta_y >= delta_z) {
            a_proection = getCellsCrossedByLineFlat(y1, x1, y2, x2, map);
            b_proection = getCellsCrossedByLineFlat(y1, z1, y2, z2, map);
            if (y1 > y2) {
                std::swap(y1, y2);
            }
            for (int j = y1; j <= y2; ++j) {
                for (auto i : a_proection[j]) {
                    for (auto k : b_proection[j]) {
                        if (!checkTraversability(i, j, k, map)) {
                            return false;
                        }
                    }
                }
            }
        } else {
            a_proection = getCellsCrossedByLineFlat(z1, x1, z2, x2, map);
            b_proection = getCellsCrossedByLineFlat(z1, y1, z2, y2, map);
            if (z1 > z2) {
                std::swap(z1, z2);
            }
            for (int k = z1; k <= z2; ++k) {
                for (auto i : a_proection[k]) {
                    for (auto j : b_proection[k]) {
                        if (!checkTraversability(i, j, k, map)) {
                            return false;
                        }
                    }
                }
            }
        }
        for(auto cell:cells) {
            if (!checkTraversability(x1 + cell.i, y1 + cell.j, z1 + cell.k, map)) {
                return false;
            }
            if (!checkTraversability(x2 + cell.i, y2 + cell.j, z2 + cell.k, map)) {
                return false;
            }
        }
        return true;
    }
    //checks line-of-sight between a line

    std::vector<cell> getCells(int i, int j, int k)
    {
        std::vector<cell> cells_res;
        for(auto& cell : this->cells)
            cells_res.push_back({i+cell.i,j+cell.j, k+cell.k});
        return cells_res;
    }

private:
    double agentSize;
    std::vector<cell> cells; //cells that are affected by agent's body
};

#endif // LINEOFSIGHT_H
