#pragma once

#include "Matrix.h"
#include "Point3d.h"
#include "Vector3d.h"

/*================================================================================================================================================================*
 |	This is a 4x4 matrix class with methods that apply to transformation matrices mainly.                                                                         |
 *================================================================================================================================================================*/
class TransformationMatrix : public Matrix 
{
public:
	/**
		default constructor
	*/
	TransformationMatrix();

	/**
		destructor
	*/
	~TransformationMatrix(void);

	/**
		copy constructor from another transformation matrix
	*/
	TransformationMatrix(const TransformationMatrix& other);

	/**
		and a copy constructor from a normal matrix - must make sure that it has the right dimensions though
	*/
	TransformationMatrix(const Matrix& other);


	/**
		copy operator from a transformation matrix
	*/
	TransformationMatrix& operator = (const TransformationMatrix& other);

	/**
		and a copy operator from a generic matrix - again, dimensions must match
	*/
	TransformationMatrix& operator = (const Matrix& other);


	/**
		This method populates the data in the matrix with the values that are obtained from openGL (stored in column major order)
	*/
	void setOGLValues(const double* oglValues);

	/**
		This method copies the data stored in the current matrix in the array of doubles provided as input.
	*/
	void getValues(double* values) const;

	/**
		This method copies the data stored in the current matrix in the array of doubles provided as input, in column major order.
	*/
	void getOGLValues(double* values)const;

	/**
		This method sets the current matrix to be equal to one of the products: a * b, a'*b, a*b' or a'*b'.
		The values of transA and transB indicate which of the matrices are tranposed and which ones are not.
		If the desired product does not result in a 4x4 matrix an error is thrown.
	*/
	void setToProductOf(const Matrix& a, const Matrix& b, bool transA = false, bool transB = false);

	/**
		This method sets the current matrix to be equal to one of the products: a * b, a'*b, a*b' or a'*b'.
		The values of transA and transB indicate which of the matrices are tranposed and which ones are not.
		If the desired product does not result in a 4x4 matrix an error is thrown.
	*/
	void setToProductOf(const TransformationMatrix& a, const Matrix& b, bool transA = false, bool transB = false);

	/**
		This method sets the current matrix to be equal to one of the products: a * b, a'*b, a*b' or a'*b'.
		The values of transA and transB indicate which of the matrices are tranposed and which ones are not.
		If the desired product does not result in a 4x4 matrix an error is thrown.
	*/
	void setToProductOf(const Matrix& a, const TransformationMatrix& b, bool transA = false, bool transB = false);

	/**
		This method sets the current matrix to be equal to one of the products: a * b, a'*b, a*b' or a'*b'.
		The values of transA and transB indicate which of the matrices are tranposed and which ones are not.
	*/
	void setToProductOf(const TransformationMatrix& a, const TransformationMatrix& b, bool transA = false, bool transB = false);


	/**
		This method sets the current matrix to a 4x4 transformation matrix that corresponds to a translation
	*/
	void setToTranslationMatrix(const ThreeTuple& coords);

	/**
		This method sets the current matrix to a 4x4 transformation matrix that corresponds to a rotation of angle radians around the given axis.
		IT IS ASSUMED THAT THE VECTOR PASSED IN IS A UNIT VECTOR!!!
	*/
	void setToRotationMatrix(double angle, Vector3d& axis);

	/**
		This method returns a point that was transformed by the current matrix. Assume the point's w coordinate is 1!
	*/
	Point3d operator*(const Point3d &p) const;

	/**
		This method returns a vector that was transformed by the current matrix. Note that since the w-coordinate of the vector is 0, it will be unnafected
		by the translation part of the transformation matrix.
	*/
	Vector3d operator*(const Vector3d &v) const;


	/**
		This method returns the vector that corresponds to the translation associated with the current matrix.
		This vector goes from the origin of the global frame to the origin of the local frame, where this matrix
		corresponds to the transfomation matrix from the local frame into the golbal frame. The vector returned
		is specified in global coordinates.
	*/
	Vector3d getTranslation() const;

	/**
		This method returns the vector from the origin of the global frame to the origin of the local frame, where this matrix
		corresponds to the transfomation matrix from the local frame into the golbal frame. The vector returned is specified
		in local coordinates. This is equivalent to returning a vector that is equal to: R^T * r. The difference between the
		vector returned here, and the one obtained by looking at the translation of the inverse transformation matrix is that
		this one points from the origin of the global frame to the local frame, whereas the other one points the opposite way.
	*/
	Vector3d getLocalCoordTranslation() const;

	/**
		This method sets the translation part of the transformation matrix to the one that is passed in as a parameter.
	*/
	void setTranslation(const Vector3d& t);

	/**
		This method sets the translation part of the transformation matrix to the one that is passed in as a parameter.
	*/
	void setTranslation(const Point3d& t);

	/**
		This method clears the translation portion of the matrix.
	*/
	void clearTranslation();

	/**
		This method sets the current matrix to be equal to its transpose.
	*/
	void setToTranspose();

	/**
		This method returns a matrix that can be used to change coordinate frames, from a local frame to a global one. The three vectors that are
		passed in as a parameter represent the specifications of the x, y and z axes of the local frame, expressed (in global coordinates)as a function of the 
		X, Y and Z axes	of the global frame. The origin represents the origin of the local frame in the global one, expressed in global frame coordinates.
		It is assumed that the vectors form an orthonormal basis!!!
	*/
	void setToCoordFrameTransformation(const Vector3d &x, const Vector3d &y, const Vector3d &z, const Point3d &origin);


	/**
		This method returns the inverse of the current matrix, assuming that it (the current matrix) represents a matrix that is used to change coordinate
		systems (i.e. go from a local frame to a global one by multiplying by this matrix. The 3x3 matrix in the left, top corner should be orthonormal!!!
	*/
	void setToInverseCoordFrameTransformation();

	/**
		This method returns the inverse of the matrix that is passed in as a parameter, assuming that it (the current matrix) represents a matrix that is 
		used to change coordinate systems (i.e. go from a local frame to a global one by multiplying by this matrix. The 3x3 matrix in the left, top corner 
		should be orthonormal!!!
	*/
	void setToInverseCoordFrameTransformationOf(const TransformationMatrix& other);


};
