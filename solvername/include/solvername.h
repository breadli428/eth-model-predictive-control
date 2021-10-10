/*
Header file containing definitions for C interface of solvername,
 a fast costumized optimization solver.
*/

#ifndef solvername_H
#define solvername_H

#include <stdio.h>

/* For Visual Studio 2015 Compatibility */
#if (_MSC_VER >= 1900)
FILE * __cdecl __iob_func(void);
#endif
/* DATA TYPE ------------------------------------------------------------*/
typedef double solvername_float;

#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif

/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef SET_PRINTLEVEL_solvername
#define SET_PRINTLEVEL_solvername    (2)
#endif

/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
	/* column vector of length 3 */
	solvername_float T0[3];

} solvername_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
	/* column vector of length 3 */
	solvername_float output1[3];

} solvername_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
	/* iteration number */
	solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;

	/* inf-norm of equality constraint residuals */
	solvername_float res_eq;

	/* inf-norm of inequality constraint residuals */
	solvername_float res_ineq;

	/* primal objective */
	solvername_float pobj;

	/* dual objective */
	solvername_float dobj;

	/* duality gap := pobj - dobj */
	solvername_float dgap;

	/* relative duality gap := |dgap / pobj | */
	solvername_float rdgap;

	/* duality measure */
	solvername_float mu;

	/* duality measure (after affine step) */
	solvername_float mu_aff;

	/* centering parameter */
	solvername_float sigma;

	/* number of backtracking line search steps (affine direction) */
	solver_int32_default lsit_aff;

	/* number of backtracking line search steps (combined direction) */
	solver_int32_default lsit_cc;

	/* step size (affine direction) */
	solvername_float step_aff;

	/* step size (combined direction) */
	solvername_float step_cc;

	/* solvertime */
	solvername_float solvetime;

} solvername_info;


/* SOLVER FUNCTION DEFINITION -------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/* examine exitflag before using the result! */
extern solver_int32_default solvername_solve(solvername_params *params, solvername_output *output, solvername_info *info, FILE *fs);

#ifdef __cplusplus
}
#endif

#endif /* solvername_H */
