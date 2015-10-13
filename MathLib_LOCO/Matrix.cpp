#include "stdafx.h"

#include "matrix.h"
#include "gsl/blas/gsl_blas.h"
#include "Vector3d.h"

/**
	constructor	- creates an m rows	by n columns matrix	that is	not initialized to a particular values
*/
Matrix::Matrix(int m, int n){
	this->matrix = gsl_matrix_alloc(m, n);
}

/**
	default	constructor
*/
Matrix::Matrix(){
	//create 4x4 matrices by default - no good reason for it, but I don't want to keep checking for nulls
	this->matrix = gsl_matrix_alloc(4, 4);
}

/**
	copy constructor - performs	a deep copy	of the matrix passed in	as a parameter.
*/
Matrix::Matrix(const Matrix& other){
	this->matrix = gsl_matrix_alloc(other.matrix->size1, other.matrix->size2);
	gsl_matrix_memcpy(this->matrix, other.matrix);
}

/**
	destructor.
*/
Matrix::~Matrix(){
	gsl_matrix_free(this->matrix);
}

/**
	loads the matrix with all zero values.
*/
void Matrix::loadZero(){
	gsl_matrix_set_zero(this->matrix);
}

/**
	loads the matrix with 1's on the diagonal -	note: the matrix doesn't have to be	square.
*/
void Matrix::loadIdentity(){
	gsl_matrix_set_identity(this->matrix);
}

/**
	this method sets the current matrix to a 3x3 matrix that is equal to the outer product of the vectors a and b
*/
void Matrix::setToOuterproduct(const Vector3d& a, const Vector3d& b)
{
	resizeTo(3,3);
	MATRIX_AT(this->matrix, 0, 0) = a.x * b.x;
	MATRIX_AT(this->matrix, 0, 1) = a.x * b.y;
	MATRIX_AT(this->matrix, 0, 2) = a.x * b.z;

	MATRIX_AT(this->matrix, 1, 0) = a.y * b.x;
	MATRIX_AT(this->matrix, 1, 1) = a.y * b.y;
	MATRIX_AT(this->matrix, 1, 2) = a.y * b.z;

	MATRIX_AT(this->matrix, 2, 0) = a.z * b.x;
	MATRIX_AT(this->matrix, 2, 1) = a.z * b.y;
	MATRIX_AT(this->matrix, 2, 2) = a.z * b.z;
}


/**
	this method returns true if the current matrix can be safely overwritten by a matrix with m rows by n columns, or false if 
	we need to allocate a new matrix to do the job.
*/
void Matrix::resizeTo(int m, int n)
{
	//if the current matrix already has the correct dimensions, our work is easy
	if (this->matrix->size1 == m && this->matrix->size2 == n)
		return;

	//if the matrix already has a block of memory large enough, then our work is again easy
	if (this->matrix->owner == 1 && this->matrix->block != NULL && this->matrix->block->size >= (size_t)m*n){
		this->matrix->size1 = m;
		this->matrix->size2 = n;
		this->matrix->tda = n;
		return;
	}

	//otherwise the matrix doesn't own the memory or it doesn't have enough of it, so we'll create a new one of correct dimensions
	gsl_matrix_free(this->matrix);
	this->matrix = gsl_matrix_alloc(m, n);
}

/**
	copy operator -	performs a deep	copy of	the	matrix passed in as	a parameter.
*/
Matrix&	Matrix::operator=(const	Matrix &other){
	//make sure that the current matrix is large enough and has the right dimensions
	resizeTo((int)other.matrix->size1, (int)other.matrix->size2);

	//and now copy the data
	gsl_matrix_memcpy(this->matrix, other.matrix);
	return *this;
}

/**
	this method	performs a shallow copy	of the matrix that is passed in	as a parameter.
*/
void Matrix::shallowCopy(const Matrix& other, int startRow, int startCol, int endRow, int endCol){
	//we have to first delete the current matrix
	gsl_matrix_free(this->matrix);
	//and create the shallow copy
	if (endRow < 0) endRow = other.matrix->size1;
	if (endCol < 0) endRow = other.matrix->size2;
	this->matrix = gsl_matrix_alloc_from_matrix(other.matrix, startRow, startCol, endRow, endCol);
}

/**
	this method performs a deep copy of the matrix that is passed in as a paramerer.
*/
void Matrix::deepCopy(const Matrix& other){
	//make sure that the current matrix is large enough and has the right dimensions
	resizeTo((int)other.matrix->size1, (int)other.matrix->size2);

	//and now copy the data
	gsl_matrix_memcpy(this->matrix, other.matrix);
}

/**
	Returns	the	number of columns
*/
int	Matrix::getColumnCount() const{
	return (int)this->matrix->size2;
}

/**
	Returns	the	number of rows
*/
int	Matrix::getRowCount()const{
	return (int)this->matrix->size1;
}

/**
	Multiplies each	element	in the current matrix by a constant
*/
void Matrix::multiplyBy(const double val){
	gsl_matrix_scale(this->matrix, val);
}

/**
	Implement this operator to have a quick way of multiplying 3x3 matrices by vectors - used for dynamics for instance
*/
Vector3d Matrix::operator * (const Vector3d &other){
	Vector3d result;
	if (this->matrix->size1 != 3 || this->matrix->size2 != 3)
    {
		//throwError("Can only use the * operator between a 3x3 matrix and a vector3d object!");
    }
    result.x = MATRIX_AT(this->matrix, 0, 0) * other.x + MATRIX_AT(this->matrix, 0, 1) * other.y + MATRIX_AT(this->matrix, 0, 2) * other.z;
	result.y = MATRIX_AT(this->matrix, 1, 0) * other.x + MATRIX_AT(this->matrix, 1, 1) * other.y + MATRIX_AT(this->matrix, 1, 2) * other.z;
	result.z = MATRIX_AT(this->matrix, 2, 0) * other.x + MATRIX_AT(this->matrix, 2, 1) * other.y + MATRIX_AT(this->matrix, 2, 2) * other.z;

	return result;
}

/**
	Implement a potentially faster function for 3x3 matrix - vector3d multiplication
*/
void Matrix::postMultiplyVector(const Vector3d& v, Vector3d& result){
	if (this->matrix->size1 != 3 || this->matrix->size2 != 3)
    {
		return;
        //throwError("Can only use the * operator between a 3x3 matrix and a vector3d object!");
    }
    result.x = MATRIX_AT(this->matrix, 0, 0) * v.x + MATRIX_AT(this->matrix, 0, 1) * v.y + MATRIX_AT(this->matrix, 0, 2) * v.z;
	result.y = MATRIX_AT(this->matrix, 1, 0) * v.x + MATRIX_AT(this->matrix, 1, 1) * v.y + MATRIX_AT(this->matrix, 1, 2) * v.z;
	result.z = MATRIX_AT(this->matrix, 2, 0) * v.x + MATRIX_AT(this->matrix, 2, 1) * v.y + MATRIX_AT(this->matrix, 2, 2) * v.z;
}

/**
	This method	sets the current matrix	to be equal	to one of the products:	a *	b, a'*b, a*b' or a'*b'.
	The	values of transA and transB	indicate which of the matrices are tranposed and which ones	are	not.

	NOTE: the matrices a and b should have no elements in common to the current matrix, otherwise the
	result is unpredictable.

*/
void Matrix::setToProductOf(const Matrix& a, const Matrix& b, bool transA, bool	transB)
{
	//we'll use the blas subroutine for this
	CBLAS_TRANSPOSE_t TransA = (transA) ? (CblasTrans):(CblasNoTrans);
	CBLAS_TRANSPOSE_t TransB = (transB) ? (CblasTrans):(CblasNoTrans);

	const size_t M = this->matrix->size1;
	const size_t N = this->matrix->size2;
	const size_t MA = (!transA) ? a.matrix->size1 : a.matrix->size2;
	const size_t NA = (!transA) ? a.matrix->size2 : a.matrix->size1;
	const size_t MB = (!transB) ? b.matrix->size1 : b.matrix->size2;
	const size_t NB = (!transB) ? b.matrix->size2 : b.matrix->size1;

	//we might be able to perform the multiplication inplace if: 
	//	- the current matrix has the correct dimensions
	//	- the current matrix is different than both a and b
	//	- NOTE: if either a or b is a shallow copy of a then the results are unpredictable
	if (this->matrix != a.matrix && this->matrix != b.matrix){
		//if the current matrix already has the correct dimension, proceed right away
		if (M == MA && N == NB && NA == MB){   /* [MxN] = [MAxNA][MBxNB] */
			gsl_blas_dgemm(TransA, TransB, 1.0, a.matrix, b.matrix, 0.0, this->matrix);
			return;
		}
		//we'll resize the current matrix and then proceede
		resizeTo((int)MA, (int)NB);
		gsl_blas_dgemm(TransA, TransB, 1.0, a.matrix, b.matrix, 0.0, this->matrix);
		return;
	}
	
	Matrix *c = new Matrix((int)MA, (int)NB);
	//otherwise it means that either a or b is the current matrix, so we'll allocate a new one...
	gsl_blas_dgemm(TransA, TransB, 1.0, a.matrix, b.matrix, 0.0, c->matrix);

	//now copy over the current matrix the result of the multiplication - deep copy
	deepCopy(*c);
	//and now deallocate the matrix c
	delete c;
}

/**
	This method	computes the inverse of	the	matrix a and writes	it over	the	current	matrix.
*/
void Matrix::setToInverseOf(const Matrix &a, double t){
		if (a.matrix->size1 != a.matrix->size2)
        {
			//throwError("Cannot invert a matrix that is not square");
            return;
        }

		int DIM = (int)a.matrix->size1;
		this->resizeTo(DIM, DIM);

//we'll write some messy code and try to get efficient inverses by means of determinants for 1x1, 2x2 and 3x3 matrices. We'll also safeguard 
//against very small determinants if desired...
		if (DIM == 1){
			double a00 = MATRIX_AT(a.matrix, 0, 0);
			if (fabs(a00)<t)
				a00 = t * fabs(a00)/a00;
			MATRIX_AT(this->matrix, 0, 0) = 1/a00;
			return;
		}

		if (DIM == 2){
			double a11 = MATRIX_AT(a.matrix, 0, 0);
			double a12 = MATRIX_AT(a.matrix, 0, 1);
			double a21 = MATRIX_AT(a.matrix, 1, 0);
			double a22 = MATRIX_AT(a.matrix, 1, 1);
			
			double det = a11*a22-a12*a21;
			if (fabs(det)<t)
				det = t * fabs(det)/det;

			MATRIX_AT(this->matrix, 0, 0) = a22 / det;
			MATRIX_AT(this->matrix, 0, 1) = -a12 / det;
			MATRIX_AT(this->matrix, 1, 0) = -a21 / det;
			MATRIX_AT(this->matrix, 1, 1) = a11 / det;
			return;
		}

		if (DIM == 3){
			double a11 = MATRIX_AT(a.matrix, 0, 0);
			double a12 = MATRIX_AT(a.matrix, 0, 1);
			double a13 = MATRIX_AT(a.matrix, 0, 2);

			double a21 = MATRIX_AT(a.matrix, 1, 0);
			double a22 = MATRIX_AT(a.matrix, 1, 1);
			double a23 = MATRIX_AT(a.matrix, 1, 2);

			double a31 = MATRIX_AT(a.matrix, 2, 0);
			double a32 = MATRIX_AT(a.matrix, 2, 1);
			double a33 = MATRIX_AT(a.matrix, 2, 2);
			
			double det = a11*(a33*a22-a32*a23)-a21*(a33*a12-a32*a13)+a31*(a23*a12-a22*a13);

			if (fabs(det)<t)
				det = t * fabs(det)/det;

			MATRIX_AT(this->matrix, 0, 0) = (a33*a22-a32*a23)/det;
			MATRIX_AT(this->matrix, 0, 1) = -(a33*a12-a32*a13)/det;
			MATRIX_AT(this->matrix, 0, 2) = (a23*a12-a22*a13)/det;

			MATRIX_AT(this->matrix, 1, 0) = -(a33*a21-a31*a23)/det;
			MATRIX_AT(this->matrix, 1, 1) = (a33*a11-a31*a13)/det;
			MATRIX_AT(this->matrix, 1, 2) = -(a23*a11-a21*a13)/det;


			MATRIX_AT(this->matrix, 2, 0) = (a32*a21-a31*a22)/det;
			MATRIX_AT(this->matrix, 2, 1) = -(a32*a11-a31*a12)/det;
			MATRIX_AT(this->matrix, 2, 2) = (a22*a11-a21*a12)/det;

			return;
		}

//ok, it's already messy, so if the dimmensions are even bigger than 3, we'll do it by row reduction


		double val, val2;
		int i, j, k, ind;
		
		//make a copy of the current matrix
		Matrix tmp = a;

		this->loadIdentity();
    
		for (i = 0; i != DIM; i++) {
			
			val = MATRIX_AT(tmp.matrix, i, i);			/* find pivot */
			ind = i;
			for (j = i + 1; j != DIM; j++) {
				if (fabs(MATRIX_AT(tmp.matrix, j, i)) > fabs(val)) {
					ind = j;
					val = MATRIX_AT(tmp.matrix, j, i);
				}
			}
            
			if (ind != i) {			
				for (j = 0; j != DIM; j++) {
					val2 = MATRIX_AT(this->matrix, i, j);
					MATRIX_AT(this->matrix,i,j) = MATRIX_AT(this->matrix, ind, j);
					MATRIX_AT(this->matrix,ind,j) = val2;           /* swap columns */
					val2 = MATRIX_AT(tmp.matrix, i, j);
					MATRIX_AT(tmp.matrix,i,j) = MATRIX_AT(tmp.matrix, ind, j);
					MATRIX_AT(tmp.matrix, ind,j) = val2;
				}
			}

			//safeguard against zero's if need be...
			if (fabs(val)<t)
				val = t * fabs(val)/val;

			if (IS_ZERO(val))
            {
				//throwError("Matrix is singular.");
                return;
            }
			for (j = 0; j != DIM; j++) {
				MATRIX_AT(tmp.matrix, i, j) /= val;
				MATRIX_AT(this->matrix, i, j) /= val;
			}
        
			for (j = 0; j != DIM; j++) {		
				if (j == i)
					continue;                       /* eliminate column */
				val = MATRIX_AT(tmp.matrix, j, i);
				for (k = 0; k != DIM; k++) {
					MATRIX_AT(tmp.matrix, j,k) = MATRIX_AT(tmp.matrix, j, k) - MATRIX_AT(tmp.matrix, i, k)  * val;
					MATRIX_AT(this->matrix, j,k) = MATRIX_AT(this->matrix, j, k) - MATRIX_AT(this->matrix, i, k)  * val;
				}
			}
		}

		//and done
}

/**
	This method	prints the contents	of the matrix -	testing	purpose	only.
*/
void Matrix::printMatrix() const{
	for (unsigned int i=0;i<this->matrix->size1;i++){
		for (unsigned int j=0;j<this->matrix->size2;j++)
        {
			//tprintf("%2.6lf\t", this->get(i,j));
		//tprintf("\n");
        }
	}	
}

/**
	This method	sets the current matrix	to be a	sub-matrix (starting at	(i,j) and ending at	(i+rows, j+cols)
	of the one that	is passed in as	a parameter	- shallow copy only.
*/
void Matrix::setToSubmatrix(const Matrix &a, int i,	int	j, int rows, int cols){
	//we have to first delete the current matrix
//	gsl_matrix_free(this->matrix);
	//and create the shallow copy
//	this->matrix = gsl_matrix_alloc_from_matrix(a.matrix, i, j, rows, cols);
//we'll do it manually to make it faster, and we won't check for the correctness of the input - up to the user to provide good parameters

  if (this->matrix->owner){
      gsl_block_free(this->matrix->block);
  }

  this->matrix->data = a.matrix->data + i * a.matrix->tda + j ;
  this->matrix->size1 = rows;
  this->matrix->size2 = cols;
  this->matrix->tda = a.matrix->tda;
  this->matrix->block = a.matrix->block;
  this->matrix->owner = 0;
}

/**
	This method	returns	a copy of the value	of the matrix at (i,j)
*/
double Matrix::get(int i, int j) const{
	return gsl_matrix_get(this->matrix, i, j);
}

/**
	This method	sets the value of the matrix at	(i,j) to newVal.
*/
void Matrix::set(int i,	int	j, double newVal){
	gsl_matrix_set(this->matrix, i, j, newVal);
}

/**
	This method is used to set the values in the matrix to the ones that are passed in the array of doubles.
	It is assumed that the array contains the right number of elements and that there is no space between consecutive rows (tda == nrCols).
*/
void Matrix::setValues(double* vals){
	//we'll create a fake matrix first
	gsl_matrix fake;
	fake.size1 = this->matrix->size1;
	fake.size2 = this->matrix->size2;
	fake.tda = this->matrix->size2;
	fake.data = vals;
	fake.block = NULL;
	fake.owner = 0;

	//and now we'll copy the elements from the fake matrix over to the current matrix
	gsl_matrix_memcpy(this->matrix, &fake);

	//and that's it...
}



/**
	this method returns a pointer to its internal matrix data structure
*/
gsl_matrix* Matrix::getMatrixPointer() const{
	return this->matrix;
}


/**
	this method adds the matrix that is passed in as a parameter to the current matrix. In addition, it scales, both matrices by the two
	numbers that are passed in as parameters:
	*this = a * *this + b * other.
*/
void Matrix::add(const Matrix& other, double scaleA, double scaleB){
	if (scaleA == 1.0 && scaleB == 1)
		gsl_matrix_add(this->matrix, other.matrix);
	else
		gsl_matrix_add_scaled(this->matrix, other.matrix, scaleA, scaleB);
}

/**
	this method subtracts the matrix that is passed in as a parameter from the current matrix. In addition, it scales, both matrices by the two
	numbers that are passed in as parameters:
	*this = a * *this - b * other.
*/
void Matrix::sub(const Matrix& other, double scaleA, double scaleB){
	if (scaleA == 1.0 && scaleB == 1)
		gsl_matrix_sub(this->matrix, other.matrix);
	else
		gsl_matrix_add_scaled(this->matrix, other.matrix, scaleA, -scaleB);	
}



void testMatrixClass(){

	Matrix AA(5,5);

	AA.loadIdentity();

	AA.set(0,0,1);
	AA.set(0,1,3);
	AA.set(0,2,4);


	AA.set(1,0,4);
	AA.set(1,1,6);
	AA.set(1,2,2.3);


	AA.set(2,0,-1);
	AA.set(2,1,3);
	AA.set(2,2,4.4);

	AA.set(4, 3, -3.14156);
	AA.set(4, 1, 2.345);


	Matrix BB(5, 5);
	BB.setToInverseOf(AA);
	BB.printMatrix();
	//tprintf("------\n");

	Matrix CC(5, 5);
	CC.setToProductOf(AA, BB);
	CC.printMatrix();
	//tprintf("------\n");


	Matrix A(4,3);
	A.loadIdentity();
	A.set(3,2,5);
	A.printMatrix();
	
	//tprintf("------\n");

	Matrix B(3,5);
	B.loadIdentity();
	B.set(0,3,4);B.set(0,4,6);
	B.set(1,0,3);B.set(1,2,2);B.set(1,3,2);
	B.set(2,0,2);B.set(2,1,-1);B.set(2,3,5);B.set(2,4,4);

	B.printMatrix();
	//tprintf("------\n");

	Matrix C(6, 6);
	C.setToProductOf(A,B);
	C.printMatrix();
	//tprintf("------\n");

	C.setToProductOf(B,B, true, false);
	C.printMatrix();
	//tprintf("------\n");

	Matrix E;
	E.deepCopy(C);

	Matrix D(3, 3);
	D.setToSubmatrix(C, 0, 1, 4, 2);
	D.printMatrix();
	//tprintf("------\n");

	C.setToProductOf(A,D,true,false);
	C.printMatrix();
	//tprintf("------\n");

	D.printMatrix();
	//tprintf("------\n");

	C.setToProductOf(B,A,true,true);
	C.printMatrix();
	//tprintf("------\n");

	D.setToSubmatrix(C, 0, 0, 3, 3);
	D.printMatrix();
	//tprintf("------\n");

	B.deepCopy(D);
	B.printMatrix();
	//tprintf("------\n");


	B.setToInverseOf(B);
	B.printMatrix();
	//tprintf("------\n");


	C.setToProductOf(B,D);
	C.printMatrix();
	//tprintf("------\n");

	E.printMatrix();	
	//tprintf("------\n");

	D.setToSubmatrix(E, 1, 1, 3, 3);
	D.deepCopy(C);

	E.printMatrix();	
	//tprintf("------\n");


	D.setToSubmatrix(E, 0, 0, 2, 5);
	double vals[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
	D.setValues(vals);
	E.printMatrix();
	//tprintf("------\n");
}
