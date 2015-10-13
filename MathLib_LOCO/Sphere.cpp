#include "stdafx.h"

#include "Sphere.h"

Sphere::Sphere(void){
    this->pos = Point3d();
    this->radius = 0;
}

Sphere::Sphere(const Point3d& p, double r){
    this->pos = p;
    this->radius = r;
}

Sphere::~Sphere(void){
}

