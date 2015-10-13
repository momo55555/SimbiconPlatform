#pragma once

#include "ThreeTuple.h"


class Vector3d;
/*=====================================================================================================================================================*
 | This class implements a Point in 3d. It will be stored in homogenous coordinates (x, y, z, w). Every time a w-component is set, the x, y, z will be |
 | rescaled so that w results in being 1 - in other words, the w component is always 1. The x, y and z components are inherited from the Three Tuple   |
 | class that this class extends.                                                                                                                      |
 *=====================================================================================================================================================*/


class Point3d : public ThreeTuple{
public:
	/**
		some useful constructors.
	*/
	Point3d() : ThreeTuple(){
	}
	Point3d(double x, double y, double z) : ThreeTuple(x, y, z){
	}

	Point3d(double x, double y, double z, double w) : ThreeTuple(x, y, z){
		setW(w);
	}

	Point3d(double x, double y) : ThreeTuple(x, y){
	}

	Point3d(ThreeTuple &p) : ThreeTuple(p){
	}

	Point3d(const Point3d& other){
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
	}

	Point3d& operator = (const Point3d& other){
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
		return *this;
	}

	/**
		default destructor.
	*/
	~Point3d(){}
	
	/**
		this method is used to set the w component.
	*/
	inline void setW(double w){
		if (w == 0)
        {
            //throwError("Cannot set w-component of a point to 0.");
            return;
        }

		this->x/=w;
		this->y/=w;
		this->z/=w;
	}

	//*this = p + v * s
	void setToOffsetFromPoint(const Point3d &p, const Vector3d& v, double s);

	/**
		addition of a point and a vector - results in a point
	*/
	Point3d operator + (const Vector3d &v) const;

	/**
		add this vector to the current point
	*/
	Point3d& operator += (const Vector3d &v);

	inline Point3d& operator /= (double val){
		double v = 1/val;
		x *= v;
		y *= v;
		z *= v;
		return *this;
	}

	/**
		difference betewwn two points - results in a vector
	*/
	Vector3d operator - (const Point3d &p) const;

	/**
		Returns a vector that has all its components multiplied by -1.
	*/
	inline Point3d operator - (){
		return Point3d(-this->x, -this->y, -this->z);
	}

	/**
		multiplication by a constant - results in a new vector.
	*/
	inline Point3d operator * (double n) const{
		return Point3d(n*this->x, n*this->y, n*this->z);
	}

	void drawObject();

//	virtual Point3d* createCopy();
};


