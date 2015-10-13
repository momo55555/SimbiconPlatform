#pragma once

class RigidBody;
/**
	This class is mainly a container for a Contact Point. It holds information such as the world coordinates of the contact point, 
	the normal at the contact, the rigid bodies that generated it, etc.
*/
class ContactPoint
{
public:
	//this is the world coordinate of the origin of the contact force...
	Point3d cp;
	//this is the normal at the contact point
	Vector3d n;
	//and this is the penetration depth
	double d;
	//this is the first rigid body that participated in the contact
	RigidBody* rb1;
	//and this is the second...
	RigidBody* rb2;
	//and this is the force applied (with f being applied to rb1, and -f to rb2)
	Vector3d f;

	//provide a copy operator
	ContactPoint& operator = (const ContactPoint& other){
		this->cp = other.cp;
		this->f = other.f;
		this->n = other.n;
		this->d = other.d;
		this->rb1 = other.rb1;
		this->rb2 = other.rb2;
		return *this;
	}

	//provide a copy operator
	ContactPoint(const ContactPoint& other){
		this->cp = other.cp;
		this->f = other.f;
		this->n = other.n;
		this->d = other.d;
		this->rb1 = other.rb1;
		this->rb2 = other.rb2;
	}
	//and a default constructor
	ContactPoint(){
		rb1 = rb2 = NULL;
	}

};