/*
internal_solvername_1 : A fast customized optimization solver.

Copyright (C) 2013-2021 EMBOTECH AG [info@embotech.com]. All rights reserved.


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

/* Generated by FORCESPRO v4.2.1 on Wednesday, May 12, 2021 at 8:45:14 AM */
#ifndef internal_solvername_1_H
#define internal_solvername_1_H

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#endif


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


/* DATA TYPE ------------------------------------------------------------*/
typedef double internal_solvername_1_float;


typedef double internal_solvername_1interface_float;

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_internal_solvername_1
#define MISRA_C_internal_solvername_1 (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_internal_solvername_1
#define RESTRICT_CODE_internal_solvername_1 (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_internal_solvername_1
#define SET_PRINTLEVEL_internal_solvername_1    (2)
#endif

/* timing */
#ifndef SET_TIMING_internal_solvername_1
#define SET_TIMING_internal_solvername_1    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_internal_solvername_1         (200)	

/* scaling factor of line search (affine direction) */
#define SET_LS_SCALE_AFF_internal_solvername_1  (internal_solvername_1_float)(0.9)      

/* scaling factor of line search (combined direction) */
#define SET_LS_SCALE_internal_solvername_1      (internal_solvername_1_float)(0.95)  

/* minimum required step size in each iteration */
#define SET_LS_MINSTEP_internal_solvername_1    (internal_solvername_1_float)(1E-08)

/* maximum step size (combined direction) */
#define SET_LS_MAXSTEP_internal_solvername_1    (internal_solvername_1_float)(0.995)

/* desired relative duality gap */
#define SET_ACC_RDGAP_internal_solvername_1     (internal_solvername_1_float)(0.0001)

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_internal_solvername_1     (internal_solvername_1_float)(1E-06)

/* desired maximum residual on inequality constraints */
#define SET_ACC_RESINEQ_internal_solvername_1   (internal_solvername_1_float)(1E-06)

/* desired maximum violation of complementarity */
#define SET_ACC_KKTCOMPL_internal_solvername_1  (internal_solvername_1_float)(1E-06)

/* desired maximum violation of stationarity (only checked if value is > 0) */
#define SET_ACC_KKTSTAT_internal_solvername_1  (internal_solvername_1_float)(-1)

/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_internal_solvername_1      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_internal_solvername_1 (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_internal_solvername_1   (2)

/* no progress in line search possible */
#define NOPROGRESS_internal_solvername_1   (-7)

/* fatal internal error - nans occurring */
#define NAN_internal_solvername_1  (-10)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_internal_solvername_1   (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_internal_solvername_1   (-12)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_internal_solvername_1  (-100)


/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct internal_solvername_1_params
{
    /* vector of size 3 */
    internal_solvername_1_float p_1[3];

} internal_solvername_1_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct internal_solvername_1_output
{
    /* vector of size 1 */
    internal_solvername_1_float o_1[1];

    /* vector of size 1 */
    internal_solvername_1_float o_2[1];

    /* vector of size 1 */
    internal_solvername_1_float o_3[1];

} internal_solvername_1_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct internal_solvername_1_info
{
    /* iteration number */
    solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;
	
    /* inf-norm of equality constraint residuals */
    internal_solvername_1_float res_eq;
	
    /* inf-norm of inequality constraint residuals */
    internal_solvername_1_float res_ineq;

    /* primal objective */
    internal_solvername_1_float pobj;	
	
    /* dual objective */
    internal_solvername_1_float dobj;	

    /* duality gap := pobj - dobj */
    internal_solvername_1_float dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    internal_solvername_1_float rdgap;		

	/* infinity norm of gradient of Lagrangian*/
	internal_solvername_1_float gradient_lag_norm;

    /* duality measure */
    internal_solvername_1_float mu;

	/* duality measure (after affine step) */
    internal_solvername_1_float mu_aff;
	
    /* centering parameter */
    internal_solvername_1_float sigma;
	
    /* number of backtracking line search steps (affine direction) */
    solver_int32_default lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    solver_int32_default lsit_cc;
    
    /* step size (affine direction) */
    internal_solvername_1_float step_aff;
    
    /* step size (combined direction) */
    internal_solvername_1_float step_cc;    

	/* solvertime */
	internal_solvername_1_float solvetime;   

} internal_solvername_1_info;









/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Wednesday, May 12, 2021 8:45:15 AM */
/* User License expires on: (UTC) Tuesday, June 15, 2021 9:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Tuesday, June 15, 2021 9:00:00 PM (approx.) */
/* Solver Generation Request Id: edda4235-283d-4249-93c5-d9f96eb7cd7a */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif
extern solver_int32_default internal_solvername_1_solve(internal_solvername_1_params *params, internal_solvername_1_output *output, internal_solvername_1_info *info, FILE *fs);


#ifdef __cplusplus
}
#endif

#endif
