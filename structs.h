#ifndef STRUCTS_H
#define STRUCTS_H
#include "gl_const.h"
#include <utility>
#include <vector>
#include <string>
#include <cmath>

struct conflict
{
    int agent1;
    int agent2;
    int sec1;
    int sec2;
    double i;
    double j;
    double k;
    double g;
};


struct Agent
{
    std::string id;
    int start_i;
    int start_j;
    int start_k;
    double start_heading;
    int goal_i;
    int goal_j;
    int goal_k;
    double goal_heading;
    double size;
    double rspeed;
    double mspeed;
    Agent(){ start_i = -1; start_j = -1; start_k = -1; goal_i = -1; goal_j = -1; goal_k = -1;
             size = CN_DEFAULT_SIZE; mspeed = CN_DEFAULT_MSPEED; rspeed = CN_DEFAULT_RSPEED;
             start_heading = CN_DEFAULT_SHEADING; goal_heading = CN_DEFAULT_GHEADING; }
};

struct constraint
{
    double i;
    double j;
    double g;
    bool goal;
};

struct movement
{
    double g;
    int p_dir;
    int s_dir;
};

struct SafeInterval
{
    double begin;
    double end;
    int id;
    SafeInterval(double begin_=0, double end_=CN_INFINITY, int id_=0):begin(begin_), end(end_), id(id_) {}
};

struct Node
{
    Node(int _i=-1, int _j=-1, int _k=-1, double _g=-1, double _F=-1):i(_i),j(_j),k(_k),g(_g),F(_F),Parent(nullptr){}
    ~Node(){ Parent = nullptr; }
    int     i, j, k;
    double  size;
    double  g;
    double  F;
    double  heading;
    Node*   Parent;
    SafeInterval interval;
};

struct obstacle
{
    std::string id;
    double size;
    double mspeed;
    std::vector<Node> sections;
    obstacle(){ id = -1; size = CN_DEFAULT_SIZE; mspeed = CN_DEFAULT_MSPEED; }
};

struct section
{
    section(int _i1=-1, int _j1=-1, int _k1=-1, int _i2=-1, int _j2=-1, int _k2=-1, double _g1=-1, double _g2=-1)
        :i1(_i1), j1(_j1), k1(_k1), i2(_i2), j2(_j2), k2(_k2), g1(_g1), g2(_g2){}
    section(const Node &a, const Node &b):i1(a.i), j1(a.j), k1(a.k), i2(b.i), j2(b.j), k2(b.k), g1(a.g), g2(b.g){}
    int i1;
    int j1;
    int k1;
    int i2;
    int j2;
    int k2;
    double size;
    double g1;
    double g2;//is needed for goal and wait actions
    double mspeed;
    bool operator == (const section &comp) const {return (i1 == comp.i1 && j1 == comp.j1 && k1 == comp.k1 && g1 == comp.g1);}

};

struct cell {
    cell(int _i=-1, int _j=-1, int _k=-1):i (_i), j (_j), k(_k){}
    int i;
    int j;
    int k;
    bool operator == (const cell &comp) const {return (i == comp.i && j == comp.j && k == comp.k);}
};

class Vector3D {
  public:
    Vector3D(double _i = 0.0, double _j = 0.0, double _k = 0.0):i(_i),j(_j),k(_k){}
    double i, j, k;

    inline Vector3D operator +(const Vector3D &vec) const { return Vector3D(i + vec.i, j + vec.j, k + vec.k); }
    inline Vector3D operator -(const Vector3D &vec) const { return Vector3D(i - vec.i, j - vec.j, k - vec.k); }
    inline Vector3D operator -() const { return Vector3D(-i,-j,-k); }
    inline Vector3D operator /(const double &num) const { return Vector3D(i/num, j/num, k/num); }
    inline Vector3D operator *(const double &num) const { return Vector3D(i*num, j*num, k*num); }
    inline double operator *(const Vector3D &vec) const { return i*vec.i + j*vec.j + k*vec.k; }
    inline void operator +=(const Vector3D &vec) { i += vec.i; j += vec.j; k += vec.k; }
    inline void operator -=(const Vector3D &vec) { i -= vec.i; j -= vec.j; k -= vec.k; }
    double vectorLength() const { return sqrt(i*i + j*j + k*k); }
    double findCos(const Vector3D &vec) const { return (*this*vec) / (vectorLength() + vec.vectorLength()); }
};

class Point {
public:
    double i;
    double j;
    double k;

    Point(double _i = 0.0, double _j = 0.0, double _k = 0.0):i (_i), j (_j), k(_k){}
    Point operator-(Point &p){return Point(i - p.i, j - p.j, k - p.k);}
    int operator== (Point &p){return (i == p.i) && (j == p.j) && (k == p.k);}
    double segmentLength(Point &pO)
    {
        Point p1 = *this;
        Point a = p1 - pO;
        return sqrt(a.i*a.i + a.j*a.j + a.k*a.k);
    }
};
#endif
