#include <gsl/gsl_errno.h>
#include <gsl/matrix/gsl_matrix.h>


#define BASE_DOUBLE
#include <gsl/templates_on.h>
#include <gsl/matrix/matrix_source.c>
#include <gsl/templates_off.h>
#undef BASE_DOUBLE
