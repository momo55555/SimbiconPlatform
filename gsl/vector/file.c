#include <stdio.h>
#include <gsl/gsl_errno.h>
#include <gsl/block/gsl_block.h>
#include <gsl/vector/gsl_vector.h>


#define BASE_DOUBLE
#include <gsl/templates_on.h>
#include <gsl/vector/file_source.c>
#include <gsl/templates_off.h>
#undef BASE_DOUBLE

