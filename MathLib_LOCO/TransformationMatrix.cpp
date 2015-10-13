#include "stdafx.h"
#include <iostream>

#include "transformationmatrix.h"
#include <gsl/blas/gsl_blas.h>


#pragma once

#include "Matrix.h"
#include "Point3d.h"
#include "Vector3d.h"


/**
	default constructor
*/
TransformationMatrix::TransformationMatrix() : Matrix(4,4){
}

/**
	destructor
*/
TransformationMatrix::~TransformationMatrix(void){
	//nothing to do here...
}

/**
	copy constructor from another transformation matrix
*/
TransformationMatrix::TransformationMatrix(const TransformationMatrix& other) : Matrix(other){
}

/**
	and a copy constructor from a normal matrix - must make sure that it has the right dimensions though
*/
TransformationMatrix::TransformationMatrix(const Matrix& other) : Matrix(other){
	if (this->matrix->size1!=4 || this->matrix->size2!=4){
		this->resizeTo(4,4);
		//throwError( "CONSTRUCTOR: Matrix passed in did not have the correct dimensions!");
	}
}


/**
	copy operator from a transformation matrix
*/
TransformationMatrix& TransformationMatrix::operator = (const TransformationMatrix& other){
	//we already know they have the same dimensions, so we won't check for that to save time
	gsl_matrix_memcpy(this->matrix, other.matrix);
	return *this;
}

/**
	and a copy operator from a generic matrix - again, dimensions must match
*/
TransformationMatrix& TransformationMatrix::operator = (const Matrix& other){
	//before doing anything, we must make sure that the matrix that is passed in has the correct dimensions.


	//if it has the right dimension, we'll do the copy...
	gsl_matrix_memcpy(this->matrix, other.getMatrixPointer());
	return *this;
}


/**
	This method populates the data in the matrix with the values that are obtained from openGL (stored in column major order)
*/
void TransformationMatrix::setOGLValues(const double* oglValues){
	for (int i=0;i<4;i++)
		for (int j=0;j<4;j++)
			this->set(i,j,oglValues[j*4+i]);
}

/**
	This method copies the data stored in the current matrix in the array of doubles provided as input.
*/
void TransformationMatrix::getValues(double* values) const{
	//copy the values from this matrix into the array... try to do it in one shot first
	if (this->matrix->tda == 4)
		std::memcpy(values, this->matrix->data, 16*sizeof(double));
	else{
		//we'll have to do it element by element
		for (int i=0;i<4;i++)
			for (int j=0;j<4;j++)
				values[i*4+j] = this->get(i,j);
	}
}

/**
	This method copies the data stored in the current matrix in the array of doubles provided as input, in column major order.
*/
void TransformationMatrix::getOGLValues(double* values)const{
		for (int i=0;i<4;i++)
			for (int j=0;j<4;j++)
				values[j*4+i] = this->get(i,j);
}

/**
	This method sets the current matrix to be equal to one of the products: a * b, a'*b, a*b' or a'*b'.
	The values of transA and transB indicate which of the matrices are tranposed and which ones are not.
	If the desired product does not result in a 4x4 matrix an error is thrown.
*/
void TransformationMatrix::setToProductOf(const Matrix& a, const Matrix& b, bool transA, bool transB){
	Matrix::setToProductOf(a, b, transA, transB);
	//check and make sure the dimensions were correct
	if (this->matrix->size1!=4 || this->matrix->size2!=4){
		this->resizeTo(4,4);
        return;
		//throwError("Matrix Product: Matrices passed in did not have the correct dimensions!");
	}
}


/**
	This method sets the current matrix to be equal to one of the products: a * b, a'*b, a*b' or a'*b'.
	The values of transA and transB indicate which of the matrices are tranposed and which ones are not.
	If the desired product does not result in a 4x4 matrix an error is thrown.
*/
void TransformationMatrix::setToProductOf(const TransformationMatrix& a, const Matrix& b, bool transA, bool transB){
	//check and make sure the dimensions were correct
	if (b.getMatrixPointer()->size1!=4 || b.getMatrixPointer()->size2!=4){
		//throwError("Matrix Product: Matrices passed in did not have the correct dimensions!");
        return;
	}
	Matrix::setToProductOf(a, b, transA, transB);
}

/**
	This method sets the current matrix to be equal to one of the products: a * b, a'*b, a*b' or a'*b'.
	The values of transA and transB indicate which of the matrices are tranposed and which ones are not.
	If the desired product does not result in a 4x4 matrix an error is thrown.
*/
void TransformationMatrix::setToProductOf(const Matrix& a, const TransformationMatrix& b, bool transA, bool transB){
	//check and make sure the dimensions were correct
	if (a.getMatrixPointer()->size1!=4 || a.getMatrixPointer()->size2!=4){
		//throwError("Matrix Product: Matrices passed in did not have the correct dimensions!");
        return;
	}
	Matrix::setToProductOf(a, b, transA, transB);
}

/**
	This method sets the current matrix to be equal to one of the products: a * b, a'*b, a*b' or a'*b'.
	The values of transA and transB indicate which of the matrices are tranposed and which ones are not.
	If the desired product does not result in a 4x4 matrix an error is thrown.
*/
void TransformationMatrix::setToProductOf(const TransformationMatrix& a, const TransformationMatrix& b, bool transA, bool transB){
	CBLAS_TRANSPOSE_t TransA = (transA) ? (CblasTrans):(CblasNoTrans);
	CBLAS_TRANSPOSE_t TransB = (transB) ? (CblasTrans):(CblasNoTrans);

	//since we know all the matrices have the correct dimensions, we will skip the extra steps, but we still need to make sure the current matrix
	//is not equal to a or b...
	if (this->matrix != a.matrix && this->matrix != b.matrix && a.matrix->owner == 1 && b.matrix->owner==1 && this->matrix->owner==1)
		gsl_blas_dgemm(TransA, TransB, 1.0, a.matrix, b.matrix, 0.0, this->matrix);
	else{
		TransformationMatrix *c = new TransformationMatrix();
		//otherwise it means that either a or b is the current matrix, so we'll allocate a new one...
		gsl_blas_dgemm(TransA, TransB, 1.0, a.matrix, b.matrix, 0.0, c->matrix);

		//now copy over the current matrix the result of the multiplication - deep copy
		deepCopy(*c);
		//and now deallocate the matrix c
		delete c;
	}
}


/**
	This method sets the current matrix to a 4x4 transformation matrix that corresponds to a translation
*/
void TransformationMatrix::setToTranslationMatrix(const ThreeTuple& coords){
	this->loadIdentity();
	this->setTranslation(Vector3d(coords.x, coords.y, coords.z));
}

/**
	This method sets the current matrix to a 4x4 transformation matrix that corresponds to a rotation of angle radians around the given axis.
	IT IS ASSUMED THAT THE VECTOR PASSED IN IS A UNIT VECTOR!!!
*/
void TransformationMatrix::setToRotationMatrix(double angle, Vector3d& axis){
	double Ax = axis.x, Ay = axis.y, Az = axis.z;
	double c = cos(angle);
	double s = sin(angle);
	double rotData[16] = {c + (1-c)*Ax*Ax,		(1-c)*Ax*Ay - s*Az,		(1-c)*Ax*Az + s*Ay,		0,
						(1-c)*Ax*Ay+s*Az,		c + (1-c)*Ay*Ay,		(1-c)*Ay*Az - s*Ax,		0,
						(1-c)*Ax*Az-s*Ay,		(1-c)*Ay*Az+s*Ax,		c+(1-c)*Az*Az,			0,
						0,						0,						0,						1};
	setValues(rotData);
}

/**
	This method returns a point that was transformed by the current matrix. Assume the point's w coordinate is 1!
*/
Point3d TransformationMatrix::operator*(const Point3d &p) const{
	gsl_matrix *m = this->matrix;

	double w = (MATRIX_AT(m, 3, 0)*p.x+MATRIX_AT(m, 3, 1)*p.getY()+MATRIX_AT(m, 3, 2)*p.z+MATRIX_AT(m, 3, 3));
	Point3d result(MATRIX_AT(m, 0, 0)*p.x+MATRIX_AT(m, 0, 1)*p.y+MATRIX_AT(m, 0, 2)*p.z+MATRIX_AT(m, 0, 3)
					,(MATRIX_AT(m, 1, 0)*p.x+MATRIX_AT(m, 1, 1)*p.y+MATRIX_AT(m, 1, 2)*p.z+MATRIX_AT(m, 1, 3))
					,(MATRIX_AT(m, 2, 0)*p.x+MATRIX_AT(m, 2, 1)*p.y+MATRIX_AT(m, 2, 2)*p.z+MATRIX_AT(m, 2, 3)));
	result.setW(w);
	return result;
}

/**
	This method returns a vector that was transformed by the current matrix. Note that since the w-coordinate of the vector is 0, it will be unnafected
	by the translation part of the transformation matrix.
*/
Vector3d TransformationMatrix::operator*(const Vector3d &v) const{
	gsl_matrix *m = this->matrix;
	return Vector3d(MATRIX_AT(m, 0, 0)*v.x+MATRIX_AT(m, 0, 1)*v.y+MATRIX_AT(m, 0, 2)*v.z
					,(MATRIX_AT(m, 1, 0)*v.x+MATRIX_AT(m, 1, 1)*v.y+MATRIX_AT(m, 1, 2)*v.z)
					,(MATRIX_AT(m, 2, 0)*v.x+MATRIX_AT(m, 2, 1)*v.y+MATRIX_AT(m, 2, 2)*v.z));
}


/**
	This method returns the vector that corresponds to the translation associated with the current matrix.
	This vector goes from the origin of the global frame to the origin of the local frame, where this matrix
	corresponds to the transfomation matrix from the local frame into the golbal frame. The vector returned
	is specified in global coordinates.
*/
Vector3d TransformationMatrix::getTranslation() const{
	Vector3d result;
	result.x = MATRIX_AT(this->matrix, 0, 3);
	result.y = MATRIX_AT(this->matrix, 1, 3);
	result.z = MATRIX_AT(this->matrix, 2, 3);
	return result;
}

/**
	This method returns the vector from the origin of the global frame to the origin of the local frame, where this matrix
	corresponds to the transfomation matrix from the local frame into the golbal frame. The vector returned is specified
	in local coordinates. This is equivalent to returning a vector that is equal to: R^T * r. The difference between the
	vector returned here, and the one obtained by looking at the translation of the inverse transformation matrix is that
	this one points from the origin of the global frame to the local frame, whereas the other one points the opposite way.
*/
Vector3d TransformationMatrix::getLocalCoordTranslation() const{
	//multiply the translation vector by the transpose of the rotation part: R^T * r
	Vector3d result;

	gsl_matrix *m = this->matrix;

	result.x = MATRIX_AT(m, 0, 0)*MATRIX_AT(m, 0, 3) + MATRIX_AT(m, 1, 0)*MATRIX_AT(m, 1, 3) + MATRIX_AT(m, 2, 0)*MATRIX_AT(m, 2, 3);
	result.y = MATRIX_AT(m, 0, 1)*MATRIX_AT(m, 0, 3) + MATRIX_AT(m, 1, 1)*MATRIX_AT(m, 1, 3) + MATRIX_AT(m, 2, 1)*MATRIX_AT(m, 2, 3);
	result.z = MATRIX_AT(m, 0, 2)*MATRIX_AT(m, 0, 3) + MATRIX_AT(m, 1, 2)*MATRIX_AT(m, 1, 3) + MATRIX_AT(m, 2, 2)*MATRIX_AT(m, 2, 3);
	return result;
}

/**
	This method sets the translation part of the transformation matrix to the one that is passed in as a parameter. The translation
	vector is assumed to be represented in global coordinates.
*/
void TransformationMatrix::setTranslation(const Vector3d& t){
	MATRIX_AT(this->matrix, 0, 3) = t.x;
	MATRIX_AT(this->matrix, 1, 3) = t.y;
	MATRIX_AT(this->matrix, 2, 3) = t.z;
}

/**
	This method sets the translation part of the transformation matrix to the one that is passed in as a parameter.
*/
void TransformationMatrix::setTranslation(const Point3d& t){
	MATRIX_AT(this->matrix, 0, 3) = t.x;
	MATRIX_AT(this->matrix, 1, 3) = t.y;
	MATRIX_AT(this->matrix, 2, 3) = t.z;
}


/**
	This method clears the translation portion of the matrix.
*/
void TransformationMatrix::clearTranslation(){
	MATRIX_AT(this->matrix, 0, 3) = 0;
	MATRIX_AT(this->matrix, 1, 3) = 0;
	MATRIX_AT(this->matrix, 2, 3) = 0;
}


/**
	This method sets the current matrix to be equal to its transpose.
*/
void TransformationMatrix::setToTranspose(){
	for (int i=0;i<4;i++)
		for (int j=i+1;j<4;j++){
			double temp = MATRIX_AT(this->matrix, i, j);
			MATRIX_AT(this->matrix, i, j) = MATRIX_AT(this->matrix, j, i);
			MATRIX_AT(this->matrix, j, i) = temp;
		}
}

/**
	This method returns a matrix that can be used to change coordinate frames, from a local frame to a global one. The three vectors that are
	passed in as a parameter represent the specifications of the x, y and z axes of the local frame, expressed (in global coordinates)as a function of the 
	X, Y and Z axes	of the global frame. The origin represents the origin of the local frame in the global one, expressed in global frame coordinates.
	It is assumed that the vectors form an orthonormal basis!!!
*/
void TransformationMatrix::setToCoordFrameTransformation(const Vector3d &x, const Vector3d &y, const Vector3d &z, const Point3d &origin){
	//NOTE: The transformation below is equivalent to Ttr * Trot, where Trot transforms the axes of the child (local) frame into the parent (global) frame
	//and Ttr is the transformation that changes the location of the origin from the child to the parent frame (coordinates expressed in parent frame!!).
	//The inverse transformation is Trot^-1 * Ttr^-1. TRot^-1 is the transpose of TRot (because it is an orthonormal matrix), and Ttr^-1 is just moving
	//to -coordinate of origin - the inverse goes from world space to local space (or parent to child) - it is implemented in the method 
	//getInverseCoordFrameTransformation
	double data[16]  = {x.x, y.x, z.x, origin.x,
						x.y, y.y, z.y, origin.y,
						x.z, y.z, z.z, origin.z,
						0  , 0  , 0	 ,	    1};
	setValues(data);
}


/**
	This method returns the inverse of the current matrix, assuming that it (the current matrix) represents a matrix that is used to change coordinate
	systems (i.e. go from a local frame to a global one by multiplying by this matrix). The 3x3 matrix in the left, top corner should be orthonormal!!!
*/
void TransformationMatrix::setToInverseCoordFrameTransformation(){
	//this is the vector from the origin of the global frame to the orign of the local one.
	Vector3d r = this->getLocalCoordTranslation();		//this is the negative of the displacement between the global frame and the local one, expressed in local
														//coords, which is what the translation part in the inverse matrix should be anyway
	this->clearTranslation();
	this->setToTranspose();
	this->setTranslation(-r);
}

/*
	This method returns the inverse of the matrix that is passed in as a parameter, assuming that it (the current matrix) represents a matrix that is 
	used to change coordinate systems (i.e. go from a local frame to a global one by multiplying by this matrix. The 3x3 matrix in the left, top corner 
	should be orthonormal!!!
*/
void TransformationMatrix::setToInverseCoordFrameTransformationOf(const TransformationMatrix& other){
	*this = other;
	setToInverseCoordFrameTransformation();
}



void testTransformationMatrixClass(){


	TransformationMatrix rot, trans;
	rot.setToRotationMatrix(0.52, Vector3d(1,1,0).toUnit());
	trans.setToTranslationMatrix(Vector3d(3,-5,2));
	//tprintf("rot:\n------\n");
	rot.printMatrix();
	//tprintf("translation:\n------\n");
	trans.printMatrix();
	TransformationMatrix tr;

	tr.setToProductOf(trans,rot);
	//tprintf("trans * rot:\n------\n");
	tr.printMatrix();
	tr.setToProductOf(tr,trans);
	//tprintf("composite transformation:\n------\n");
	tr.printMatrix();

	TransformationMatrix invTr1 = tr;
	invTr1.setToInverseCoordFrameTransformation();
	Matrix invTr2(4,4);
	invTr2.setToInverseOf(tr);
	TransformationMatrix invTr3 = invTr1;
	invTr3.setToInverseCoordFrameTransformation();
	//tprintf("inverse 1:\n------\n");
	invTr1.printMatrix();
	//tprintf("inverse 2:\n------\n");
	invTr2.printMatrix();
	//tprintf("inverse 3:\n------\n");
	invTr3.printMatrix();
	//tprintf("composite transformation:\n------\n");
	tr.printMatrix();

	TransformationMatrix ide1;
	ide1.setToProductOf(invTr1 , tr);
    TransformationMatrix ide2;
	ide2.setToProductOf(invTr2, tr);

	//tprintf("identity 1: \n------\n");
	ide1.printMatrix();
	//tprintf("identity 2: \n------\n");
	ide2.printMatrix();

}

