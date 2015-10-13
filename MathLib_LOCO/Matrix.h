#pragma once

#include <stdlib.h>
#include <stdio.h>

#include "MathLib.h"
#include "ThreeTuple.h"

#include <gsl/matrix/gsl_matrix.h>

#define MATRIX_AT(m, i, j) (*((m->data + ((i) * m->tda + (j)))))

class Vector3d;

/*====================================================================================================================================================================*
 | This class will be used to represent matrices of arbitrary sizes (m rows by n columns) that have elements of type double. The underlying data strucutre used by    |
 | this class is gsl's (Gnu Scientific Library) matrix class. This class also makes use of the ATLAS implementation of BLAS for some operations such as matrix-matrix |
 | multiplication. This class is meant to improve performance, not necessarily ease of use.                                                                           |
 *====================================================================================================================================================================*/

class Matrix
{
friend class Vector;
friend class ParticleEngine;
protected:

	//this data structure holds all the matrix data
	gsl_matrix *matrix;


public:
	/**
		constructor - creates an m rows by n columns matrix that is not initialized to any particular values
	*/
	Matrix(int m, int n);

	/**
		default constructor
	*/
	Matrix();

	/**
		copy constructor - performs a deep copy of the matrix passed in as a parameter.
	*/
	Matrix(const Matrix& other);

	/**
		destructor.
	*/
	~Matrix();

	/**
		loads the matrix with all zero values.
	*/
	void loadZero();

	/**
		loads the matrix with 1's on the diagonal, 0's everywhere else - note: the matrix doesn't have to be square.
	*/
	void loadIdentity();

	/**
		this method sets the current matrix to a 3x3 matrix that is equal to the outer product of the vectors a and b
	*/
	void setToOuterproduct(const Vector3d& a, const Vector3d& b);

	/**
		this method resizes the current matrix to have m rows and n cols. If not enough space is allocated for it, then a new matrix of
		correct dimensions is allocated. There is no guarantee with regards to the data that is contained in the matrix after a resize operation.
	*/
	void resizeTo(int m, int n);

	/**
		copy operator - performs a deep copy of the matrix passed in as a parameter.
	*/
	Matrix& operator=(const Matrix &other);

	/**
		this method performs a shallow copy of the matrix that is passed in as a parameter.
	*/
	void shallowCopy(const Matrix& other, int startRow = 0, int startCol = 0, int endRow = -1, int endCol = -1);

	/**
		this method performs a deep copy of the matrix that is passed in as a paramerer.
	*/
	void deepCopy(const Matrix& other);

	/**
		Returns the number of columns
	*/
	int getColumnCount() const;

	/**
		Returns the number of rows
	*/
	int getRowCount()const;

	/**
		Multiplies each element in the current matrix by a constant
	*/
	void multiplyBy(const double val);

	/**
		This method sets the current matrix to be equal to one of the products: a * b, a'*b, a*b' or a'*b'.
		The values of transA and transB indicate which of the matrices are tranposed and which ones are not.
	*/
	void setToProductOf(const Matrix& a, const Matrix& b, bool transA = false, bool transB = false);

	/**
		This method computes the inverse of the matrix a and writes it over the current matrix. The implementation
		for the inverse of a matrix was obtained from Graphite. The parameter t is used as a threshold value for
		determinants, etc so that we still get a result for the inverse of our matrix even if it is very poorly conditioned.
	*/
	void setToInverseOf(const Matrix &a, double t = 0);

	/**
		This method prints the contents of the matrix - testing purpose only.
	*/
	void printMatrix() const;

	/**
		This method sets the current matrix to be a sub-matrix (starting at (i,j) and ending at (i+rows, j+cols)
		of the one that is passed in as a parameter - shallow copy only.
	*/
	void setToSubmatrix(const Matrix &a, int i, int j, int rows, int cols);

	/**
		This method returns a copy of the value of the matrix at (i,j)
	*/
	double get(int i, int j) const;

	/**
		This method sets the value of the matrix at (i,j) to newVal.
	*/
	void set(int i, int j, double newVal);


	/**
		This method is used to set the values in the matrix to the ones that are passed in the array of doubles.
		It is assumed that the array contains the right number of elements and that there is no space between consecutive rows (tda == nrCols).
	*/
	void setValues(double* vals);

	/**
		this method returns a pointer to its internal matrix data structure
	*/
	gsl_matrix* getMatrixPointer() const;


	/**
		Implement this operator to have a quick way of multiplying 3x3 matrices by vectors - used for dynamics for instance
	*/
	Vector3d operator * (const Vector3d &other);

	/**
		Implement a potentially faster function for 3x3 matrix - vector3d multiplication. Result and v need to be different!!
	*/
	void postMultiplyVector(const Vector3d& v, Vector3d& result);

	/**
		Still need to do (if need be): get row/col vector. Initialize a matrix from a vector so that we can create a 1xn matrix easily.
		make sure dgemv works properly, implement vector class, etc, swap, add, scale matrices, etc
	*/

	/**
		this method adds the matrix that is passed in as a parameter to the current matrix. In addition, it scales, both matrices by the two
		numbers that are passed in as parameters:
		*this = a * *this + b * other.
	*/
	void add(const Matrix& other, double scaleA = 1.0, double scaleB = 1.0);

	/**
		this method subtracts the matrix that is passed in as a parameter from the current matrix. In addition, it scales, both matrices by the two
		numbers that are passed in as parameters:
		*this = a * *this - b * other.
	*/
	void sub(const Matrix& other, double scaleA = 1.0, double scaleB = 1.0);

};
