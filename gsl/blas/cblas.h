#pragma once

#include <stdio.h>

/**
	This header file describes a few BLAS methods that I will be using regurarly.
*/
#include "f2c.h"

doublereal ddot(integer *n, doublereal *dx, integer *incx, doublereal *dy, integer *incy);
doublereal dnrm2(integer *n, doublereal *x, integer *incx);
int dswap(integer *n, doublereal *dx, integer *incx, doublereal *dy, integer *incy);
int dcopy(integer *n, doublereal *dx, integer *incx, doublereal *dy, integer *incy);
int dgemv(char *trans, integer *m, integer *n, doublereal *	alpha, doublereal *a, integer *lda, doublereal *x, integer *incx, doublereal *beta, doublereal *y, integer *incy);
int dgemm(char *transa, char *transb, integer *m, integer *	n, integer *k, doublereal *alpha, doublereal *a, integer *lda, doublereal *b, integer *ldb, doublereal *beta, doublereal *c, integer *ldc);
logical lsame_(char *ca, char *cb);


