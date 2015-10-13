/* blas/gsl_blas.h
 * 
 * Copyright (C) 1996, 1997, 1998, 1999, 2000 Gerard Jungman
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

/*
 * Author:  G. Jungman
 */
#ifndef __GSL_BLAS_H__
#define __GSL_BLAS_H__

#include <gsl/vector/gsl_vector.h>
#include <gsl/matrix/gsl_matrix.h>

#include <gsl/blas/gsl_blas_types.h>


#undef __BEGIN_DECLS
#undef __END_DECLS
#ifdef __cplusplus
# define __BEGIN_DECLS extern "C" {
# define __END_DECLS }
#else
# define __BEGIN_DECLS /* empty */
# define __END_DECLS /* empty */
#endif

__BEGIN_DECLS

/**
	MODIFIED BY STELIAN:
		- only keep a subset of the methods that I think are important
		- link to an external blas library
*/

/* ========================================================================
 * Level 1
 * ========================================================================
 */

/**
*     forms the dot product of two vectors.
*/
GSL_FUNC int gsl_blas_ddot (const gsl_vector * X, const gsl_vector * Y, double * result);

/**
*  DNRM2 returns the euclidean norm of a vector via the function
*  name, so that
*
*     DNRM2 := sqrt( x'*x )
*/
GSL_FUNC double gsl_blas_dnrm2  (const gsl_vector * X);

/**
*     interchanges two vectors.	
*/
GSL_FUNC int  gsl_blas_dswap (gsl_vector * X, gsl_vector * Y);

/**
*     copies a vector, x, to a vector, y.
*/
GSL_FUNC int  gsl_blas_dcopy (const gsl_vector * X, gsl_vector * Y);

/* ===========================================================================
 * Level 2
 * ===========================================================================
 */

/**
*  DGEMV  performs one of the matrix-vector operations
*
*     y := alpha*A*x + beta*y,   or   y := alpha*A'*x + beta*y,
*
*  where alpha and beta are scalars, x and y are vectors and A is an
*  m by n matrix.
*/
GSL_FUNC int  gsl_blas_dgemv (CBLAS_TRANSPOSE_t TransA, double alpha, const gsl_matrix * A, const gsl_vector * X, double beta, gsl_vector * Y);

/*
 * ===========================================================================
 * Prototypes for level 3 BLAS
 * ===========================================================================
 */

/**
*  DGEMM  performs one of the matrix-matrix operations
*
*     C := alpha*op( A )*op( B ) + beta*C,
*
*  where  op( X ) is one of
*
*     op( X ) = X   or   op( X ) = X',
*
*  alpha and beta are scalars, and A, B and C are matrices, with op( A )
*  an m by k matrix,  op( B )  a  k by n matrix and  C an m by n matrix.
*/
GSL_FUNC int  gsl_blas_dgemm (CBLAS_TRANSPOSE_t TransA, CBLAS_TRANSPOSE_t TransB, double alpha, const gsl_matrix * A, const gsl_matrix * B, double beta, gsl_matrix * C);

__END_DECLS

#endif /* __GSL_BLAS_H__ */
