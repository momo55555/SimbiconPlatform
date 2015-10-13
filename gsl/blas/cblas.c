#include <gsl/blas/cblas.h>

/**
	All the methods below are obtained directly from the ATLAS implementation of BLAS, available at: http://www.netlib.org/clapack/cblas/
*/

#include "f2c.h"

doublereal ddot(integer *n, doublereal *dx, integer *incx, doublereal *dy, integer *incy){

    /* System generated locals */
    integer i__1;
    doublereal ret_val;

    /* Local variables */
    static integer i, m;
    static doublereal dtemp;
    static integer ix, iy, mp1;


/*     forms the dot product of two vectors.   
       uses unrolled loops for increments equal to one.   
       jack dongarra, linpack, 3/11/78.   
       modified 12/3/93, array(1) declarations changed to array(*)   

   Parameter adjustments   
       Function Body */
#define DY(I) dy[(I)-1]
#define DX(I) dx[(I)-1]


    ret_val = 0.;
    dtemp = 0.;
    if (*n <= 0) {
	return ret_val;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*        code for unequal increments or equal increments   
            not equal to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i = 1; i <= *n; ++i) {
	dtemp += DX(ix) * DY(iy);
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    ret_val = dtemp;
    return ret_val;

/*        code for both increments equal to 1   


          clean-up loop */

L20:
    m = *n % 5;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i = 1; i <= m; ++i) {
	dtemp += DX(i) * DY(i);
/* L30: */
    }
    if (*n < 5) {
	goto L60;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i = mp1; i <= *n; i += 5) {
	dtemp = dtemp + DX(i) * DY(i) + DX(i + 1) * DY(i + 1) + DX(i + 2) * 
		DY(i + 2) + DX(i + 3) * DY(i + 3) + DX(i + 4) * DY(i + 4);
/* L50: */
    }
L60:
    ret_val = dtemp;
    return ret_val;
} /* ddot_ */



doublereal dnrm2(integer *n, doublereal *x, integer *incx)
{
    /* System generated locals */
    integer i__1, i__2;
    doublereal ret_val, d__1;

    /* Builtin functions */
    double sqrt(doublereal);

    /* Local variables */
    static doublereal norm, scale, absxi;
    static integer ix;
    static doublereal ssq;


/*  DNRM2 returns the euclidean norm of a vector via the function   
    name, so that   

       DNRM2 := sqrt( x'*x )   



    -- This version written on 25-October-1982.   
       Modified on 14-October-1993 to inline the call to DLASSQ.   
       Sven Hammarling, Nag Ltd.   


    
   Parameter adjustments   
       Function Body */
#define X(I) x[(I)-1]


    if (*n < 1 || *incx < 1) {
	norm = 0.;
    } else if (*n == 1) {
	norm = abs(X(1));
    } else {
	scale = 0.;
	ssq = 1.;
/*        The following loop is equivalent to this call to the LAPACK 
  
          auxiliary routine:   
          CALL DLASSQ( N, X, INCX, SCALE, SSQ ) */

	i__1 = (*n - 1) * *incx + 1;
	i__2 = *incx;
	for (ix = 1; *incx < 0 ? ix >= (*n-1)**incx+1 : ix <= (*n-1)**incx+1; ix += *incx) {
	    if (X(ix) != 0.) {
		absxi = (d__1 = X(ix), abs(d__1));
		if (scale < absxi) {
/* Computing 2nd power */
		    d__1 = scale / absxi;
		    ssq = ssq * (d__1 * d__1) + 1.;
		    scale = absxi;
		} else {
/* Computing 2nd power */
		    d__1 = absxi / scale;
		    ssq += d__1 * d__1;
		}
	    }
/* L10: */
	}
	norm = scale * sqrt(ssq);
    }

    ret_val = norm;
    return ret_val;

/*     End of DNRM2. */

} /* dnrm2_ */

int dswap(integer *n, doublereal *dx, integer *incx, doublereal *dy, integer *incy)
{


    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i, m;
    static doublereal dtemp;
    static integer ix, iy, mp1;


/*     interchanges two vectors.   
       uses unrolled loops for increments equal one.   
       jack dongarra, linpack, 3/11/78.   
       modified 12/3/93, array(1) declarations changed to array(*)   


    
   Parameter adjustments   
       Function Body */
#define DY(I) dy[(I)-1]
#define DX(I) dx[(I)-1]


    if (*n <= 0) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*       code for unequal increments or equal increments not equal   
           to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i = 1; i <= *n; ++i) {
	dtemp = DX(ix);
	DX(ix) = DY(iy);
	DY(iy) = dtemp;
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*       code for both increments equal to 1   


         clean-up loop */

L20:
    m = *n % 3;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i = 1; i <= m; ++i) {
	dtemp = DX(i);
	DX(i) = DY(i);
	DY(i) = dtemp;
/* L30: */
    }
    if (*n < 3) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i = mp1; i <= *n; i += 3) {
	dtemp = DX(i);
	DX(i) = DY(i);
	DY(i) = dtemp;
	dtemp = DX(i + 1);
	DX(i + 1) = DY(i + 1);
	DY(i + 1) = dtemp;
	dtemp = DX(i + 2);
	DX(i + 2) = DY(i + 2);
	DY(i + 2) = dtemp;
/* L50: */
    }
    return 0;
} /* dswap_ */





int dcopy(integer *n, doublereal *dx, integer *incx, doublereal *dy, integer *incy)
{


    /* System generated locals */
    integer i__1;

    /* Local variables */
    static integer i, m, ix, iy, mp1;


/*     copies a vector, x, to a vector, y.   
       uses unrolled loops for increments equal to one.   
       jack dongarra, linpack, 3/11/78.   
       modified 12/3/93, array(1) declarations changed to array(*)   


    
   Parameter adjustments   
       Function Body */
#define DY(I) dy[(I)-1]
#define DX(I) dx[(I)-1]


    if (*n <= 0) {
	return 0;
    }
    if (*incx == 1 && *incy == 1) {
	goto L20;
    }

/*        code for unequal increments or equal increments   
            not equal to 1 */

    ix = 1;
    iy = 1;
    if (*incx < 0) {
	ix = (-(*n) + 1) * *incx + 1;
    }
    if (*incy < 0) {
	iy = (-(*n) + 1) * *incy + 1;
    }
    i__1 = *n;
    for (i = 1; i <= *n; ++i) {
	DY(iy) = DX(ix);
	ix += *incx;
	iy += *incy;
/* L10: */
    }
    return 0;

/*        code for both increments equal to 1   


          clean-up loop */

L20:
    m = *n % 7;
    if (m == 0) {
	goto L40;
    }
    i__1 = m;
    for (i = 1; i <= m; ++i) {
	DY(i) = DX(i);
/* L30: */
    }
    if (*n < 7) {
	return 0;
    }
L40:
    mp1 = m + 1;
    i__1 = *n;
    for (i = mp1; i <= *n; i += 7) {
	DY(i) = DX(i);
	DY(i + 1) = DX(i + 1);
	DY(i + 2) = DX(i + 2);
	DY(i + 3) = DX(i + 3);
	DY(i + 4) = DX(i + 4);
	DY(i + 5) = DX(i + 5);
	DY(i + 6) = DX(i + 6);
/* L50: */
    }
    return 0;
} /* dcopy_ */


int dgemm(char *transa, char *transb, integer *m, integer *	n, integer *k, doublereal *alpha, doublereal *a, integer *lda, doublereal *b, integer *ldb, doublereal *beta, doublereal *c, integer *ldc)
{

    /* System generated locals */
    integer a_dim1, a_offset, b_dim1, b_offset, c_dim1, c_offset, i__1, i__2, 
	    i__3;

    /* Local variables */
    static integer info;
    static logical nota, notb;
    static doublereal temp;
    static integer i, j, l, ncola;
    extern logical lsame_(char *, char *);
    static integer nrowa, nrowb;
    extern /* Subroutine */ int xerbla_(char *, integer *);


/*  Purpose   
    =======   

    DGEMM  performs one of the matrix-matrix operations   

       C := alpha*op( A )*op( B ) + beta*C,   

    where  op( X ) is one of   

       op( X ) = X   or   op( X ) = X',   

    alpha and beta are scalars, and A, B and C are matrices, with op( A ) 
  
    an m by k matrix,  op( B )  a  k by n matrix and  C an m by n matrix. 
  

    Parameters   
    ==========   

    TRANSA - CHARACTER*1.   
             On entry, TRANSA specifies the form of op( A ) to be used in 
  
             the matrix multiplication as follows:   

                TRANSA = 'N' or 'n',  op( A ) = A.   

                TRANSA = 'T' or 't',  op( A ) = A'.   

                TRANSA = 'C' or 'c',  op( A ) = A'.   

             Unchanged on exit.   

    TRANSB - CHARACTER*1.   
             On entry, TRANSB specifies the form of op( B ) to be used in 
  
             the matrix multiplication as follows:   

                TRANSB = 'N' or 'n',  op( B ) = B.   

                TRANSB = 'T' or 't',  op( B ) = B'.   

                TRANSB = 'C' or 'c',  op( B ) = B'.   

             Unchanged on exit.   

    M      - INTEGER.   
             On entry,  M  specifies  the number  of rows  of the  matrix 
  
             op( A )  and of the  matrix  C.  M  must  be at least  zero. 
  
             Unchanged on exit.   

    N      - INTEGER.   
             On entry,  N  specifies the number  of columns of the matrix 
  
             op( B ) and the number of columns of the matrix C. N must be 
  
             at least zero.   
             Unchanged on exit.   

    K      - INTEGER.   
             On entry,  K  specifies  the number of columns of the matrix 
  
             op( A ) and the number of rows of the matrix op( B ). K must 
  
             be at least  zero.   
             Unchanged on exit.   

    ALPHA  - DOUBLE PRECISION.   
             On entry, ALPHA specifies the scalar alpha.   
             Unchanged on exit.   

    A      - DOUBLE PRECISION array of DIMENSION ( LDA, ka ), where ka is 
  
             k  when  TRANSA = 'N' or 'n',  and is  m  otherwise.   
             Before entry with  TRANSA = 'N' or 'n',  the leading  m by k 
  
             part of the array  A  must contain the matrix  A,  otherwise 
  
             the leading  k by m  part of the array  A  must contain  the 
  
             matrix A.   
             Unchanged on exit.   

    LDA    - INTEGER.   
             On entry, LDA specifies the first dimension of A as declared 
  
             in the calling (sub) program. When  TRANSA = 'N' or 'n' then 
  
             LDA must be at least  max( 1, m ), otherwise  LDA must be at 
  
             least  max( 1, k ).   
             Unchanged on exit.   

    B      - DOUBLE PRECISION array of DIMENSION ( LDB, kb ), where kb is 
  
             n  when  TRANSB = 'N' or 'n',  and is  k  otherwise.   
             Before entry with  TRANSB = 'N' or 'n',  the leading  k by n 
  
             part of the array  B  must contain the matrix  B,  otherwise 
  
             the leading  n by k  part of the array  B  must contain  the 
  
             matrix B.   
             Unchanged on exit.   

    LDB    - INTEGER.   
             On entry, LDB specifies the first dimension of B as declared 
  
             in the calling (sub) program. When  TRANSB = 'N' or 'n' then 
  
             LDB must be at least  max( 1, k ), otherwise  LDB must be at 
  
             least  max( 1, n ).   
             Unchanged on exit.   

    BETA   - DOUBLE PRECISION.   
             On entry,  BETA  specifies the scalar  beta.  When  BETA  is 
  
             supplied as zero then C need not be set on input.   
             Unchanged on exit.   

    C      - DOUBLE PRECISION array of DIMENSION ( LDC, n ).   
             Before entry, the leading  m by n  part of the array  C must 
  
             contain the matrix  C,  except when  beta  is zero, in which 
  
             case C need not be set on entry.   
             On exit, the array  C  is overwritten by the  m by n  matrix 
  
             ( alpha*op( A )*op( B ) + beta*C ).   

    LDC    - INTEGER.   
             On entry, LDC specifies the first dimension of C as declared 
  
             in  the  calling  (sub)  program.   LDC  must  be  at  least 
  
             max( 1, m ).   
             Unchanged on exit.   


    Level 3 Blas routine.   

    -- Written on 8-February-1989.   
       Jack Dongarra, Argonne National Laboratory.   
       Iain Duff, AERE Harwell.   
       Jeremy Du Croz, Numerical Algorithms Group Ltd.   
       Sven Hammarling, Numerical Algorithms Group Ltd.   



       Set  NOTA  and  NOTB  as  true if  A  and  B  respectively are not 
  
       transposed and set  NROWA, NCOLA and  NROWB  as the number of rows 
  
       and  columns of  A  and the  number of  rows  of  B  respectively. 
  

    
   Parameter adjustments   
       Function Body */

#define A(I,J) a[(I)-1 + ((J)-1)* ( *lda)]
#define B(I,J) b[(I)-1 + ((J)-1)* ( *ldb)]
#define C(I,J) c[(I)-1 + ((J)-1)* ( *ldc)]

    nota = lsame_(transa, "N");
    notb = lsame_(transb, "N");
    if (nota) {
	nrowa = *m;
	ncola = *k;
    } else {
	nrowa = *k;
	ncola = *m;
    }
    if (notb) {
	nrowb = *k;
    } else {
	nrowb = *n;
    }

/*     Test the input parameters. */

    info = 0;
    if (! nota && ! lsame_(transa, "C") && ! lsame_(transa, "T")) {
	info = 1;
    } else if (! notb && ! lsame_(transb, "C") && ! lsame_(transb, 
	    "T")) {
	info = 2;
    } else if (*m < 0) {
	info = 3;
    } else if (*n < 0) {
	info = 4;
    } else if (*k < 0) {
	info = 5;
    } else if (*lda < max(1,nrowa)) {
	info = 8;
    } else if (*ldb < max(1,nrowb)) {
	info = 10;
    } else if (*ldc < max(1,*m)) {
	info = 13;
    }
    if (info != 0) {
	xerbla_("DGEMM ", &info);
	return 0;
    }

/*     Quick return if possible. */

    if (*m == 0 || *n == 0 || (*alpha == 0. || *k == 0) && *beta == 1.) {
	return 0;
    }

/*     And if  alpha.eq.zero. */

    if (*alpha == 0.) {
	if (*beta == 0.) {
	    i__1 = *n;
	    for (j = 1; j <= *n; ++j) {
		i__2 = *m;
		for (i = 1; i <= *m; ++i) {
		    C(i,j) = 0.;
/* L10: */
		}
/* L20: */
	    }
	} else {
	    i__1 = *n;
	    for (j = 1; j <= *n; ++j) {
		i__2 = *m;
		for (i = 1; i <= *m; ++i) {
		    C(i,j) = *beta * C(i,j);
/* L30: */
		}
/* L40: */
	    }
	}
	return 0;
    }

/*     Start the operations. */

    if (notb) {
	if (nota) {

/*           Form  C := alpha*A*B + beta*C. */

	    i__1 = *n;
	    for (j = 1; j <= *n; ++j) {
		if (*beta == 0.) {
		    i__2 = *m;
		    for (i = 1; i <= *m; ++i) {
			C(i,j) = 0.;
/* L50: */
		    }
		} else if (*beta != 1.) {
		    i__2 = *m;
		    for (i = 1; i <= *m; ++i) {
			C(i,j) = *beta * C(i,j);
/* L60: */
		    }
		}
		i__2 = *k;
		for (l = 1; l <= *k; ++l) {
		    if (B(l,j) != 0.) {
			temp = *alpha * B(l,j);
			i__3 = *m;
			for (i = 1; i <= *m; ++i) {
			    C(i,j) += temp * A(i,l);
/* L70: */
			}
		    }
/* L80: */
		}
/* L90: */
	    }
	} else {

/*           Form  C := alpha*A'*B + beta*C */

	    i__1 = *n;
	    for (j = 1; j <= *n; ++j) {
		i__2 = *m;
		for (i = 1; i <= *m; ++i) {
		    temp = 0.;
		    i__3 = *k;
		    for (l = 1; l <= *k; ++l) {
			temp += A(l,i) * B(l,j);
/* L100: */
		    }
		    if (*beta == 0.) {
			C(i,j) = *alpha * temp;
		    } else {
			C(i,j) = *alpha * temp + *beta * C(i,j);
		    }
/* L110: */
		}
/* L120: */
	    }
	}
    } else {
	if (nota) {

/*           Form  C := alpha*A*B' + beta*C */

	    i__1 = *n;
	    for (j = 1; j <= *n; ++j) {
		if (*beta == 0.) {
		    i__2 = *m;
		    for (i = 1; i <= *m; ++i) {
			C(i,j) = 0.;
/* L130: */
		    }
		} else if (*beta != 1.) {
		    i__2 = *m;
		    for (i = 1; i <= *m; ++i) {
			C(i,j) = *beta * C(i,j);
/* L140: */
		    }
		}
		i__2 = *k;
		for (l = 1; l <= *k; ++l) {
		    if (B(j,l) != 0.) {
			temp = *alpha * B(j,l);
			i__3 = *m;
			for (i = 1; i <= *m; ++i) {
			    C(i,j) += temp * A(i,l);
/* L150: */
			}
		    }
/* L160: */
		}
/* L170: */
	    }
	} else {

/*           Form  C := alpha*A'*B' + beta*C */

	    i__1 = *n;
	    for (j = 1; j <= *n; ++j) {
		i__2 = *m;
		for (i = 1; i <= *m; ++i) {
		    temp = 0.;
		    i__3 = *k;
		    for (l = 1; l <= *k; ++l) {
			temp += A(l,i) * B(j,l);
/* L180: */
		    }
		    if (*beta == 0.) {
			C(i,j) = *alpha * temp;
		    } else {
			C(i,j) = *alpha * temp + *beta * C(i,j);
		    }
/* L190: */
		}
/* L200: */
	    }
	}
    }

    return 0;

/*     End of DGEMM . */

} /* dgemm_ */






int dgemv(char *trans, integer *m, integer *n, doublereal *	alpha, doublereal *a, integer *lda, doublereal *x, integer *incx, doublereal *beta, doublereal *y, integer *incy)
{


    /* System generated locals */
    integer a_dim1, a_offset, i__1, i__2;

    /* Local variables */
    static integer info;
    static doublereal temp;
    static integer lenx, leny, i, j;
    extern logical lsame_(char *, char *);
    static integer ix, iy, jx, jy, kx, ky;
    extern /* Subroutine */ int xerbla_(char *, integer *);


/*  Purpose   
    =======   

    DGEMV  performs one of the matrix-vector operations   

       y := alpha*A*x + beta*y,   or   y := alpha*A'*x + beta*y,   

    where alpha and beta are scalars, x and y are vectors and A is an   
    m by n matrix.   

    Parameters   
    ==========   

    TRANS  - CHARACTER*1.   
             On entry, TRANS specifies the operation to be performed as   
             follows:   

                TRANS = 'N' or 'n'   y := alpha*A*x + beta*y.   

                TRANS = 'T' or 't'   y := alpha*A'*x + beta*y.   

                TRANS = 'C' or 'c'   y := alpha*A'*x + beta*y.   

             Unchanged on exit.   

    M      - INTEGER.   
             On entry, M specifies the number of rows of the matrix A.   
             M must be at least zero.   
             Unchanged on exit.   

    N      - INTEGER.   
             On entry, N specifies the number of columns of the matrix A. 
  
             N must be at least zero.   
             Unchanged on exit.   

    ALPHA  - DOUBLE PRECISION.   
             On entry, ALPHA specifies the scalar alpha.   
             Unchanged on exit.   

    A      - DOUBLE PRECISION array of DIMENSION ( LDA, n ).   
             Before entry, the leading m by n part of the array A must   
             contain the matrix of coefficients.   
             Unchanged on exit.   

    LDA    - INTEGER.   
             On entry, LDA specifies the first dimension of A as declared 
  
             in the calling (sub) program. LDA must be at least   
             max( 1, m ).   
             Unchanged on exit.   

    X      - DOUBLE PRECISION array of DIMENSION at least   
             ( 1 + ( n - 1 )*abs( INCX ) ) when TRANS = 'N' or 'n'   
             and at least   
             ( 1 + ( m - 1 )*abs( INCX ) ) otherwise.   
             Before entry, the incremented array X must contain the   
             vector x.   
             Unchanged on exit.   

    INCX   - INTEGER.   
             On entry, INCX specifies the increment for the elements of   
             X. INCX must not be zero.   
             Unchanged on exit.   

    BETA   - DOUBLE PRECISION.   
             On entry, BETA specifies the scalar beta. When BETA is   
             supplied as zero then Y need not be set on input.   
             Unchanged on exit.   

    Y      - DOUBLE PRECISION array of DIMENSION at least   
             ( 1 + ( m - 1 )*abs( INCY ) ) when TRANS = 'N' or 'n'   
             and at least   
             ( 1 + ( n - 1 )*abs( INCY ) ) otherwise.   
             Before entry with BETA non-zero, the incremented array Y   
             must contain the vector y. On exit, Y is overwritten by the 
  
             updated vector y.   

    INCY   - INTEGER.   
             On entry, INCY specifies the increment for the elements of   
             Y. INCY must not be zero.   
             Unchanged on exit.   


    Level 2 Blas routine.   

    -- Written on 22-October-1986.   
       Jack Dongarra, Argonne National Lab.   
       Jeremy Du Croz, Nag Central Office.   
       Sven Hammarling, Nag Central Office.   
       Richard Hanson, Sandia National Labs.   



       Test the input parameters.   

    
   Parameter adjustments   
       Function Body */
#define X(I) x[(I)-1]
#define Y(I) y[(I)-1]

#define A(I,J) a[(I)-1 + ((J)-1)* ( *lda)]

    info = 0;
    if (! lsame_(trans, "N") && ! lsame_(trans, "T") && ! 
	    lsame_(trans, "C")) {
	info = 1;
    } else if (*m < 0) {
	info = 2;
    } else if (*n < 0) {
	info = 3;
    } else if (*lda < max(1,*m)) {
	info = 6;
    } else if (*incx == 0) {
	info = 8;
    } else if (*incy == 0) {
	info = 11;
    }
    if (info != 0) {
	xerbla_("DGEMV ", &info);
	return 0;
    }

/*     Quick return if possible. */

    if (*m == 0 || *n == 0 || *alpha == 0. && *beta == 1.) {
	return 0;
    }

/*     Set  LENX  and  LENY, the lengths of the vectors x and y, and set 
  
       up the start points in  X  and  Y. */

    if (lsame_(trans, "N")) {
	lenx = *n;
	leny = *m;
    } else {
	lenx = *m;
	leny = *n;
    }
    if (*incx > 0) {
	kx = 1;
    } else {
	kx = 1 - (lenx - 1) * *incx;
    }
    if (*incy > 0) {
	ky = 1;
    } else {
	ky = 1 - (leny - 1) * *incy;
    }

/*     Start the operations. In this version the elements of A are   
       accessed sequentially with one pass through A.   

       First form  y := beta*y. */

    if (*beta != 1.) {
	if (*incy == 1) {
	    if (*beta == 0.) {
		i__1 = leny;
		for (i = 1; i <= leny; ++i) {
		    Y(i) = 0.;
/* L10: */
		}
	    } else {
		i__1 = leny;
		for (i = 1; i <= leny; ++i) {
		    Y(i) = *beta * Y(i);
/* L20: */
		}
	    }
	} else {
	    iy = ky;
	    if (*beta == 0.) {
		i__1 = leny;
		for (i = 1; i <= leny; ++i) {
		    Y(iy) = 0.;
		    iy += *incy;
/* L30: */
		}
	    } else {
		i__1 = leny;
		for (i = 1; i <= leny; ++i) {
		    Y(iy) = *beta * Y(iy);
		    iy += *incy;
/* L40: */
		}
	    }
	}
    }
    if (*alpha == 0.) {
	return 0;
    }
    if (lsame_(trans, "N")) {

/*        Form  y := alpha*A*x + y. */

	jx = kx;
	if (*incy == 1) {
	    i__1 = *n;
	    for (j = 1; j <= *n; ++j) {
		if (X(jx) != 0.) {
		    temp = *alpha * X(jx);
		    i__2 = *m;
		    for (i = 1; i <= *m; ++i) {
			Y(i) += temp * A(i,j);
/* L50: */
		    }
		}
		jx += *incx;
/* L60: */
	    }
	} else {
	    i__1 = *n;
	    for (j = 1; j <= *n; ++j) {
		if (X(jx) != 0.) {
		    temp = *alpha * X(jx);
		    iy = ky;
		    i__2 = *m;
		    for (i = 1; i <= *m; ++i) {
			Y(iy) += temp * A(i,j);
			iy += *incy;
/* L70: */
		    }
		}
		jx += *incx;
/* L80: */
	    }
	}
    } else {

/*        Form  y := alpha*A'*x + y. */

	jy = ky;
	if (*incx == 1) {
	    i__1 = *n;
	    for (j = 1; j <= *n; ++j) {
		temp = 0.;
		i__2 = *m;
		for (i = 1; i <= *m; ++i) {
		    temp += A(i,j) * X(i);
/* L90: */
		}
		Y(jy) += *alpha * temp;
		jy += *incy;
/* L100: */
	    }
	} else {
	    i__1 = *n;
	    for (j = 1; j <= *n; ++j) {
		temp = 0.;
		ix = kx;
		i__2 = *m;
		for (i = 1; i <= *m; ++i) {
		    temp += A(i,j) * X(ix);
		    ix += *incx;
/* L110: */
		}
		Y(jy) += *alpha * temp;
		jy += *incy;
/* L120: */
	    }
	}
    }

    return 0;

/*     End of DGEMV . */

} /* dgemv_ */

logical lsame_(char *ca, char *cb)
{


    /* System generated locals */
    logical ret_val;

    /* Local variables */
    static integer inta, intb, zcode;


/*  -- LAPACK auxiliary routine (version 2.0) --   
       Univ. of Tennessee, Univ. of California Berkeley, NAG Ltd.,   
       Courant Institute, Argonne National Lab, and Rice University   
       January 31, 1994   


    Purpose   
    =======   

    LSAME returns .TRUE. if CA is the same letter as CB regardless of   
    case.   

    Arguments   
    =========   

    CA      (input) CHARACTER*1   
    CB      (input) CHARACTER*1   
            CA and CB specify the single characters to be compared.   

   ===================================================================== 
  


       Test if the characters are equal */

    ret_val = *(unsigned char *)ca == *(unsigned char *)cb;
    if (ret_val) {
	return ret_val;
    }

/*     Now test for equivalence if both characters are alphabetic. */

    zcode = 'Z';

/*     Use 'Z' rather than 'A' so that ASCII can be detected on Prime   
       machines, on which ICHAR returns a value with bit 8 set.   
       ICHAR('A') on Prime machines returns 193 which is the same as   
       ICHAR('A') on an EBCDIC machine. */

    inta = *(unsigned char *)ca;
    intb = *(unsigned char *)cb;

    if (zcode == 90 || zcode == 122) {

/*        ASCII is assumed - ZCODE is the ASCII code of either lower o
r   
          upper case 'Z'. */

	if (inta >= 97 && inta <= 122) {
	    inta += -32;
	}
	if (intb >= 97 && intb <= 122) {
	    intb += -32;
	}

    } else if (zcode == 233 || zcode == 169) {

/*        EBCDIC is assumed - ZCODE is the EBCDIC code of either lower
 or   
          upper case 'Z'. */

	if (inta >= 129 && inta <= 137 || inta >= 145 && inta <= 153 || inta 
		>= 162 && inta <= 169) {
	    inta += 64;
	}
	if (intb >= 129 && intb <= 137 || intb >= 145 && intb <= 153 || intb 
		>= 162 && intb <= 169) {
	    intb += 64;
	}

    } else if (zcode == 218 || zcode == 250) {

/*        ASCII is assumed, on Prime machines - ZCODE is the ASCII cod
e   
          plus 128 of either lower or upper case 'Z'. */

	if (inta >= 225 && inta <= 250) {
	    inta += -32;
	}
	if (intb >= 225 && intb <= 250) {
	    intb += -32;
	}
    }
    ret_val = inta == intb;

/*     RETURN   

       End of LSAME */

    return ret_val;
} /* lsame_ */
int xerbla_(char *srname, integer *info)
{
/*  -- LAPACK auxiliary routine (version 2.0) --   
       Univ. of Tennessee, Univ. of California Berkeley, NAG Ltd.,   
       Courant Institute, Argonne National Lab, and Rice University   
       September 30, 1994   


    Purpose   
    =======   

    XERBLA  is an error handler for the LAPACK routines.   
    It is called by an LAPACK routine if an input parameter has an   
    invalid value.  A message is printed and execution stops.   

    Installers may consider modifying the STOP statement in order to   
    call system-specific exception-handling facilities.   

    Arguments   
    =========   

    SRNAME  (input) CHARACTER*6   
            The name of the routine which called XERBLA.   

    INFO    (input) INTEGER   
            The position of the invalid parameter in the parameter list   

            of the calling routine.   

   ===================================================================== 
*/

    printf("** On entry to %6s, parameter number %2i had an illegal value\n",
		srname, *info);

/*     End of XERBLA */

    return 0;
} /* xerbla_ */




