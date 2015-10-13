#include "stdafx.h"

#include "Point3d.h"
//#include <Include/glHeaders.h>
#include "Vector3d.h"
/**
	draws this point.
*/
void Point3d::drawObject(){
//	glPointSize(5);
//	glBegin(GL_POINTS);
//		glVertex3d(this->x, this->y, this->z);
//	glEnd();
//	glPointSize(1);
}

/**
	addition of two vectors - results in a new vector.
*/
Point3d Point3d::operator + (const Vector3d &v) const{
	return Point3d(this->x + v.x, this->y + v.y,this->z + v.z);
}


/**
	add this vector to the current point
*/
Point3d& Point3d::operator += (const Vector3d &v){
	this->x += v.x;
	this->y += v.y;
	this->z += v.z;
	return *this;
}


/**
	difference between two points - results in a new vector.
*/
Vector3d Point3d::operator - (const Point3d &p) const{
	return Vector3d(this->x - p.x, this->y - p.y,this->z - p.z);
}

	//*this = p + v * s
void Point3d::setToOffsetFromPoint(const Point3d &p, const Vector3d& v, double s){
	this->x = p.x + v.x*s;
	this->y = p.y + v.y*s;
	this->z = p.z + v.z*s;
}

