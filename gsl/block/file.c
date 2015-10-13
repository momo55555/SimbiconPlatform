#include <stdio.h>
#include <gsl/gsl_errno.h>
#include <gsl/block/gsl_block.h>

#define BASE_DOUBLE
#include <gsl/templates_on.h>
#include <gsl/block/fwrite_source.c>
#include <gsl/block/fprintf_source.c>
#include <gsl/templates_off.h>
#undef BASE_DOUBLE