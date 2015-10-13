#pragma once
#include "Point3d.h"


class Capsule{
public:
    //a capsule is defined by the location of the two end points, as well as the radius of the capsule
    Point3d p1, p2;
    double radius;

public:
    Capsule(void);
    Capsule(const Point3d& p1, const Point3d& p2, double r);
    ~Capsule(void);
};

