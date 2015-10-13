#include <stdlib.h>
#include <gsl/gsl_math.h>
#include <gsl/matrix/gsl_matrix.h>


#define BASE_DOUBLE
#include <gsl/templates_on.h>
#include <gsl/matrix/minmax_source.c>
#include <gsl/templates_off.h>
#undef BASE_DOUBLE
