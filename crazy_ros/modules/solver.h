/* Produced by CVXGEN, 2016-04-19 11:47:36 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double x_0[10];
  double r_0[10];
  double Q[100];
  double R[9];
  double r_1[10];
  double r_2[10];
  double r_3[10];
  double r_4[10];
  double r_5[10];
  double r_6[10];
  double r_7[10];
  double r_8[10];
  double r_9[10];
  double r_10[10];
  double A[100];
  double B[30];
  double u_max[3];
  double S_max[1];
  double *x[1];
  double *r[11];
} Params;
typedef struct Vars_t {
  double *u_0; /* 3 rows. */
  double *x_1; /* 10 rows. */
  double *u_1; /* 3 rows. */
  double *x_2; /* 10 rows. */
  double *u_2; /* 3 rows. */
  double *x_3; /* 10 rows. */
  double *u_3; /* 3 rows. */
  double *x_4; /* 10 rows. */
  double *u_4; /* 3 rows. */
  double *x_5; /* 10 rows. */
  double *u_5; /* 3 rows. */
  double *x_6; /* 10 rows. */
  double *u_6; /* 3 rows. */
  double *x_7; /* 10 rows. */
  double *u_7; /* 3 rows. */
  double *x_8; /* 10 rows. */
  double *u_8; /* 3 rows. */
  double *x_9; /* 10 rows. */
  double *u_9; /* 3 rows. */
  double *x_10; /* 10 rows. */
  double *u_10; /* 3 rows. */
  double *t_01; /* 3 rows. */
  double *t_02; /* 3 rows. */
  double *t_03; /* 3 rows. */
  double *t_04; /* 3 rows. */
  double *t_05; /* 3 rows. */
  double *t_06; /* 3 rows. */
  double *t_07; /* 3 rows. */
  double *t_08; /* 3 rows. */
  double *t_09; /* 3 rows. */
  double *t_10; /* 3 rows. */
  double *t_11; /* 3 rows. */
  double *t_12; /* 1 rows. */
  double *t_13; /* 1 rows. */
  double *t_14; /* 1 rows. */
  double *t_15; /* 1 rows. */
  double *t_16; /* 1 rows. */
  double *t_17; /* 1 rows. */
  double *t_18; /* 1 rows. */
  double *t_19; /* 1 rows. */
  double *t_20; /* 1 rows. */
  double *t_21; /* 1 rows. */
  double *x_11; /* 10 rows. */
  double *u[11];
  double *x[12];
} Vars;
typedef struct Workspace_t {
  double h[169];
  double s_inv[169];
  double s_inv_z[169];
  double b[110];
  double q[186];
  double rhs[634];
  double x[634];
  double *s;
  double *z;
  double *y;
  double lhs_aff[634];
  double lhs_cc[634];
  double buffer[634];
  double buffer2[634];
  double KKT[2918];
  double L[4121];
  double d[634];
  double v[634];
  double d_inv[634];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_871133241344[1];
  double quad_548686778368[1];
  double quad_183018455040[1];
  double quad_19734581248[1];
  double quad_653725216768[1];
  double quad_975577374720[1];
  double quad_744270442496[1];
  double quad_560115118080[1];
  double quad_6032879616[1];
  double quad_611032489984[1];
  double quad_522860462080[1];
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
