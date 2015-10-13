#include "stdafx.h"

#include "Capsule.h"

Capsule::Capsule(void){
    radius = 0;
}

Capsule::Capsule(const Point3d& p1, const Point3d& p2, double r){
    this->p1 = p1;
    this->p2 = p2;
    this->radius = r;
}

Capsule::~Capsule(void){
}
