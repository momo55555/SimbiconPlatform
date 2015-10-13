/* blas/blas.c
 * 
 * Copyright (C) 1996, 1997, 1998, 1999, 2000, 2001 Gerard Jungman & Brian 
 * Gough
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

/* GSL implementation of BLAS operations for vectors and dense
 * matrices.  Note that GSL native storage is row-major.  */

#include <gsl/gsl_math.h>
#include <gsl/gsl_errno.h>
#include <gsl/blas/gsl_blas_types.h>
#include <gsl/blas/gsl_blas.h>
#include <gsl/blas/cblas.h>

/* ========================================================================
 * Level 1
 * ========================================================================
 */

/* CBLAS defines vector sizes in terms of int. GSL defines sizes in
   terms of size_t, so we need to convert these into integers.  There
   is the possibility of overflow here. FIXME: Maybe this could be
   caught */

#define INT(X) ((int)(X))



int gsl_blas_ddot (const gsl_vector * X, const gsl_vector * Y, double *result){
  if (X->size == Y->size){
	  *result = ddot (&INT (X->size), X->data, &INT (X->stride), Y->data, &INT (Y->stride));
      return GSL_SUCCESS;
    }
  else {
      GSL_ERROR ("invalid length", GSL_EBADLEN);
    }
}

/**
*  DNRM2 returns the euclidean norm of a vector via the function
*  name, so that
*
*     DNRM2 := sqrt( x'*x )
*/

double gsl_blas_dnrm2 (const gsl_vector * X){
  return dnrm2 (&INT (X->size), X->data, &INT (X->stride));
}

/**
*     interchanges two vectors.	
*/
int gsl_blas_dswap (gsl_vector * X, gsl_vector * Y){
  if (X->size == Y->size){
      dswap (&INT (X->size), X->data, &INT (X->stride), Y->data, &INT (Y->stride));
      return GSL_SUCCESS;
    }
  else{
      GSL_ERROR ("invalid length", GSL_EBADLEN);
    };
}



/**
*     copies a vector, x, to a vector, y.
*/
int gsl_blas_dcopy (const gsl_vector * X, gsl_vector * Y){
  if (X->size == Y->size){
      dcopy (&INT (X->size), X->data, &INT (X->stride), Y->data, &INT (Y->stride));
      return GSL_SUCCESS;
    }
  else {
      GSL_ERROR ("invalid length", GSL_EBADLEN);
    }
}


/* ===========================================================================
 * Level 2
 * ===========================================================================
 */

int
gsl_blas_dgemv (CBLAS_TRANSPOSE_t TransA, double alpha, const gsl_matrix * A,
                const gsl_vector * X, double beta, gsl_vector * Y)
{
	//here the matrix is by default transposed, so to get the right behaviour we will transpose it when the user didn't want it tranposed, and leave it the way it is when it should be transposed

  //note that M and N are changed to show that the matrix is transposed by default.
  const size_t M = A->size1;
  const size_t N = A->size2;

  char aTransp = (TransA == CblasNoTrans) ? 'T' : 'N';

  if ((TransA == CblasNoTrans && N == X->size && M == Y->size)
      || (TransA == CblasTrans && M == X->size && N == Y->size))
    {
      dgemv (&aTransp, &INT (N), &INT (M), &alpha, A->data,
                   &INT (A->tda), X->data, &INT (X->stride), &beta, Y->data,
                   &INT (Y->stride));
      return GSL_SUCCESS;
    }
  else
    {
      GSL_ERROR ("invalid length", GSL_EBADLEN);
    }
}


/*
 * ===========================================================================
 * Prototypes for level 3 BLAS
 * ===========================================================================
 */


int gsl_blas_dgemm (CBLAS_TRANSPOSE_t TransA, CBLAS_TRANSPOSE_t TransB,
                double alpha, const gsl_matrix * A, const gsl_matrix * B,
                double beta, gsl_matrix * C)
{
/**
	Since BLAS uses column major matrices, when asked to perform C = A*B we will
	actually compute C = B*A, with the number of rows and cols switched up.
	(C' = B' * A' if C = A * B, and we're working with transposes by default)

	That's why we need to switch the order of the matrices as well as the order of M and N.

*/

  const size_t M = C->size1;
  const size_t N = C->size2;
  const size_t MA = (TransA == CblasNoTrans) ? A->size1 : A->size2;
  const size_t NA = (TransA == CblasNoTrans) ? A->size2 : A->size1;
  const size_t MB = (TransB == CblasNoTrans) ? B->size1 : B->size2;
  const size_t NB = (TransB == CblasNoTrans) ? B->size2 : B->size1;

  char aTransp = (TransA == CblasNoTrans) ? 'N' : 'T';
  char bTransp = (TransB == CblasNoTrans) ? 'N' : 'T';

  if (M == MA && N == NB && NA == MB)   /* [MxN] = [MAxNA][MBxNB] */
    {
	  dgemm (&bTransp, &aTransp, &INT (N), &INT (M), &INT (NA),
                   &alpha, B->data, &INT (B->tda), A->data, &INT (A->tda), &beta,
                   C->data, &INT (C->tda));
      return GSL_SUCCESS;
    }
  else
    {
      GSL_ERROR ("invalid length", GSL_EBADLEN);
    }
}

