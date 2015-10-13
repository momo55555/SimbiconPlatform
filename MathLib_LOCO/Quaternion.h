#pragma once

#include "TransformationMatrix.h"
#include "Vector3d.h"


/*================================================================================================================================================================*
 *	This class provides an implementation for quaternions. Internally, each quaternion it will be stored in the form: q = s + v, where s is a scalar and v is a   |
 *	vector.                                                                                                                                                       |
 *================================================================================================================================================================*/

class Quaternion 
{

public:
	//the scalar part of the quaternion
	double s;
	//the vector part of the quaternion
	Vector3d v;
public:
	/**
		A constructor. Intuitive...
	*/
	Quaternion(double s, Vector3d v){
		this->s = s;
		this->v = v;
	}

	/**
		A copy constructor
	*/
	Quaternion(const Quaternion& other){
		this->s = other.s;
		this->v = other.v;
	}

	/**
		Default constructor
	*/
	Quaternion(){
		this->s = 1;
		this->v = Vector3d(0,0,0);
	}
	/**
		Another constructor.
	*/
	Quaternion(double w, double x, double y, double z){
		this->s = w;
		this->v = Vector3d(x,y,z);
	}
	
	// Only works if v1 and v2 are unit length
	Quaternion(const Vector3d& v1, const Vector3d& v2) {
		v = v1.crossProductWith(v2);
		s = 1 + v1.dotProductWith(v2);
		toUnit();
	}

	/**
		A copy operator
	*/
	Quaternion& operator = (const Quaternion &rhs){
		this->s = rhs.s;
		this->v = rhs.v;
		return *this;
	}

	/**
		this method is used to set the current quaternion to the product of the two quaternions passed in as parameters.
		the bool parameters invA and invB indicate wether or not, the quaternion a or b should be inverted (well, complex conjugate really)
		for the multiplication
	*/
	void setToProductOf(const Quaternion& a, const Quaternion& b, bool invA = false, bool invB = false);
	
	/**
		this method is used to return the rotation angle represented by this quaternion - in the range -pi to pi.
		Because you can always consider a rotation to be of x degrees around axis v, or by -x degrees around axis -v,
		we need to know the base rotation axis.
	*/
	inline double getRotationAngle(const Vector3d& positiveRotAxis){
		int sinSign = SGN(positiveRotAxis.dotProductWith(v));
		double result = 2 * safeACOS(s);
		if (sinSign < 0)
			result = -result;
		if (result > PI) result -= 2*PI;
		if (result < -PI) result += 2*PI;
		return result;
	}

	/**
		Default destructor.
	*/
	~Quaternion(void){
	}

	const double getAngle() const { return 2 * safeACOS(s); }

	Vector3d getAxis() const { return v.unit(); }

	/**
		Returns the complex conjugate of the current quaternion.
	*/
	Quaternion getComplexConjugate() const;

	/**
		Returns the inverse of the current quaternion: q * q^-1 = identity quaternion: s = 1, v = (0,0,0)
	*/
	Quaternion getInverse() const;

	/**
		Returns the length of a quaternion.
	*/
	double getLength() const;

	/**
		Computes the dot product between the current quaternion and the one given as parameter.
	*/
	double dotProductWith(const Quaternion &other) const;

	/**
		This method returns a quaternion that is the result of linearly interpolating between the current quaternion and the one provided as a parameter.
		The value of the parameter t indicates the progress: if t = 0, the result will be *this. If it is 1, it will be other. If it is inbetween, then
		the result will be a combination of the two initial quaternions.
		Both quaternions that are used for the interpolation are assumed to have unit length!!!
	*/
    Quaternion linearlyInterpolateWith(const Quaternion &other, double t) const;

	/**
		This method returns a quaternion that is the result of spherically interpolating between the current quaternion and the one provided as a parameter.
		The value of the parameter t indicates the progress: if t = 0, the result will be *this. If it is 1, it will be other. If it is inbetween, then
		the result will be a combination of the two initial quaternions.
		Both quaternions that are used for the interpolation are assumed to have unit length!!!
	*/
	Quaternion sphericallyInterpolateWith(const Quaternion &other, double t)const;

	/**
		This method will return a quaternion that represents a rotation of angle radians around the axis provided as a parameter.
		IT IS ASSUMED THAT THE VECTOR PASSED IN IS A UNIT VECTOR!!!
	*/
	static Quaternion getRotationQuaternion(double angle, const Vector3d &axis);

	/**
		This method is used to rotate the vector that is passed in as a parameter by the current quaternion (which is assumed to be a 
		unit quaternion).
	*/
	Vector3d rotate(const Vector3d& u) const;


	/**
		This method is used to rotate the vector that is passed in as a parameter by the current quaternion (which is assumed to be a 
		unit quaternion).
	*/
	Vector3d inverseRotate(const Vector3d& u) const;

	/**
		This method populates the transformation matrix that is passed in as a parameter with the 
		4x4 matrix that represents an equivalent rotation as the current quaternion.
	*/
	void getRotationMatrix(TransformationMatrix* m) const;

	/**
		This method fills the presumably 3x3 matrix so that it represents an equivalent rotation as the given quaternion.
	*/
	void getRotationMatrix(Matrix* m) const;

	/**
		Returns the result of multiplying the current quaternion by rhs. NOTE: the product of two quaternions represents a rotation as well: q1*q2 represents
		a rotation by q2 followed by a rotation by q1!!!!
	*/
	Quaternion operator * (const Quaternion &rhs) const;

	/**
		This operator multiplies the current quaternion by the rhs one. Keep in mind the note RE quaternion multiplication.
	*/
	Quaternion& operator *= (const Quaternion &rhs);

	/**
		This method multiplies the current quaternion by a scalar.
	*/
	Quaternion& operator *= (double scalar);

	/**
		This method returns a copy of the current quaternion multiplied by a scalar.
	*/
	Quaternion operator * (double scalar) const;

	/**
		This method returns a quaternion that was the result of adding the quaternion rhs to the current quaternion.
	*/
	Quaternion operator + (const Quaternion &rhs) const;

	/**
		This method adds the rhs quaternion to the current one.
	*/
	Quaternion& operator += (const Quaternion &rhs);

	/**
		This method transforms the current quaternion to a unit quaternion.
	*/
	Quaternion& toUnit();

	/**
		This method returns the scalar part of the current quaternion
	*/
	inline double getS() const {return this->s;}

	/**
		This method returns the vector part of the current quaternion
	*/
	inline Vector3d getV() const {return this->v;}

	/**
		Rotates the vector v by the quaternion and the result is placed in 'result'
	*/
	inline void fastRotate(const Vector3d& u, Vector3d* result) const{
		//uRot = q * (0, u) * q' = (s, v) * (0, u) * (s, -v)
		//working it out manually, we get:
	
//		Vector3d t = u * s + v.crossProductWith(u);
//		*result = v*u.dotProductWith(v) + t * s + v.crossProductWith(t);

		result->setToCrossProduct(v, u);
		result->addScaledVector(u, s);
		Vector3d tmp;
		tmp.setToCrossProduct(v, *result);
		result->multiplyBy(s);
		result->addVector(tmp);
		result->addScaledVector(v, u.dotProductWith(v));

	}

	/**
		sets the current quaternion to represent a rotation of theta radians about the (assumed) unit axis
	*/
	inline void setToRotationQuaternion(double angle, const Vector3d& axis){
		s = cos(angle/2);
		v.setValues(axis.x, axis.y, axis.z);
		v.multiplyBy(sin(angle/2));
	}

	/**
		Assume that the current quaternion represents the relative orientation between two coordinate frames A and B.
		This method decomposes the current relative rotation into a twist of frame B around the axis v passed in as a
		parameter, and another more arbitrary rotation.

		AqB = AqT * TqB, where T is a frame that is obtained by rotating frame B around the axis v by the angle
		that is returned by this function.

		In the T coordinate frame, v is the same as in B, and AqT is a rotation that aligns v from A to that
		from T.

		It is assumed that vB is a unit vector!! rotAngle is the rotation angle around rotAxis (this gives the
		AqT transformation. To go from frame B to A, we then twist around the axis v by the amount returned
		by this function, and we then rotate around rotAxis by rotAngle.
	*/
//	double decomposeRotation(const Vector3d vB, double *rotAngle, Vector3d* rotAxis) const;

	/**
		Assume that the current quaternion represents the relative orientation between two coordinate frames P and C (i.e. q
		rotates vectors from the child/local frame C into the parent/global frame P).

		With v specified in frame C's coordinates, this method decomposes the current relative rotation, such that:

		PqC = qA * qB, where qB represents a rotation about axis v.
		
		This can be thought of us as a twist about axis v - qB - and a more general rotation, and swing
		- qA - decomposition. Note that qB can be thought of as a rotation from the C frame into a tmp trame T,
		and qA a rotation from T into P.

		In the T coordinate frame, v is the same as in C, and qA is a rotation that aligns v from P to that
		from T.
	*/
	void Quaternion::decomposeRotation(Quaternion* qA, Quaternion* qB, const Vector3d& vC) const;

	/**
		Assume that the current quaternion represents the relative orientation between two coordinate frames A and B.
		This method decomposes the current relative rotation into a twist of frame B around the axis v passed in as a
		parameter, and another more arbitrary rotation.

		AqB = AqT * TqB, where T is a frame that is obtained by rotating frame B around the axis v by the angle
		that is returned by this function.

		In the T coordinate frame, v is the same as in B, and AqT is a rotation that aligns v from A to that
		from T.

		It is assumed that vB is a unit vector!! This method returns TqB, which represents a twist about
		the axis vB.
	*/
	Quaternion decomposeRotation(const Vector3d vB) const;

};



/**
	This method is used to return a number that measures the distance between two rotation quaternion. The number returned
	is equal to |theta|, where theta is the min angle of rotations that can align the two coordinate frames
	represented by the two orientations.
*/
inline double distanceBetweenOrientations(const Quaternion &a, const Quaternion &b){
	double temp = (a.getComplexConjugate() * b).v.length();
	if (temp>1)
		temp = 1;
	return 2*asin(temp);
}

