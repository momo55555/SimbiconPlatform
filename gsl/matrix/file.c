#include <gsl/gsl_errno.h>
#include <gsl/matrix/gsl_matrix.h>
#include <gsl/vector/gsl_vector.h>


#define BASE_DOUBLE
#include <gsl/templates_on.h>
#include <gsl/matrix/file_source.c>
#include <gsl/templates_off.h>
#undef BASE_DOUBLE
