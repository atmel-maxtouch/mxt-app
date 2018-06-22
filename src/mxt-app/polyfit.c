/**********
 * Copyright 1990 Regents of the University of California.  All rights reserved.
 * Author: 1985 Wayne A. Christopher, U. C. Berkeley CAD Group
 *
 * Spice is covered now covered by the BSD Copyright:
 *
 * Copyright (c) 1985-1991 The Regents of the University of California.
 * All rights reserved.
 *
 * Permission is hereby granted, without written agreement and without license
 * or royalty fees, to use, copy, modify, and distribute this software and its
 * documentation for any purpose, provided that the above copyright notice and
 * the following two paragraphs appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO PROVIDE
 * MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * **********/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "libmaxtouch/log.h"

#include "sensor_variant.h"

double ft_peval(double x, double *coeffs)
{
  double	y;
  int	i;

  if (!coeffs)
    return 0.0;	/* XXX Should not happen */

  y = coeffs[POLY_DEGREE];	/* there are (degree+1) coeffs */

  for (i = POLY_DEGREE - 1; i >= 0; i--) {
    y *= x;
    y += coeffs[i];
  }

  return y;
}

static void print_matrix(struct libmaxtouch_ctx *ctx, double *mat, int n) {
  int i, j;

  mxt_verb(ctx, "\nMatrix:");
  for (i = 0; i < n; i++) {
    /* each row */
    for (j = 0; j < n; j++) {
      /* each column */
      mxt_verb(ctx, "%0.2f\t\t", mat[i * n + j]);
    }
    mxt_verb(ctx, "\n");
  }
  return;
}

/* Takes n = (degree+1) doubles, and fills in result with the n
 * coefficients of the polynomial that will fit them. */
bool ft_polyfit(struct libmaxtouch_ctx *ctx, double *xdata, double *ydata,
                double *result, int len)
{
  int k, j, i;
  int n = POLY_DEGREE + 1;
  double mat1[n * n];
  double mat2[n];
  double d;

  /* build input matrix - x values */
  for (i = 0; i < n; i++) {
    /* each matrix row */
    for (j = 0; j < n; j++) {
      mat1[i * n + j] = 0;

      /* each matrix col */
      for (k = 0; k < len; k++) {
        /* for each x value */
        mat1[i * n + j] += pow(xdata[k], (double)(i + j));
        mxt_verb(ctx, "i: %d j: %d k: %d, mat1[%d] = %0.2f",
            i, j, k, i * n + j, mat1[i * n + j]);
      }
    }
  }

  print_matrix(ctx, mat1, n);

  /* build & print resultant matrix - y values */
  mxt_verb(ctx, "\nResultant:");
  mat2[0] = 0;

  /* first row just the sum of y values */
  for (i = 0; i < len; i++)
    mat2[0] += ydata[i];

  mxt_verb(ctx, "mat2[0] = %0.2f", mat2[0]);

  for (i = 1; i < n; i++) {
    mat2[i] = 0;
    for (j = 0; j < len; j++)
      mat2[i] += ydata[j] * pow(xdata[j], i);

    mxt_verb(ctx, "mat2[%d] = %0.2f", i, mat2[i]);
  }

  /* Do Gauss-Jordan elimination on mat1. */
  for (i = 0; i < n; i++) {
    int lindex;
    double largest;
    /* choose largest pivot */
    for (j=i, largest = mat1[i * n + i], lindex = i; j < n; j++) {
      if (fabs(mat1[j * n + i]) > largest) {
        largest = fabs(mat1[j * n + i]);
        lindex = j;
      }
    }
    if (lindex != i) {
      /* swap rows i and lindex */
      for (k = 0; k < n; k++) {
        d = mat1[i * n + k];
        mat1[i * n + k] = mat1[lindex * n + k];
        mat1[lindex * n + k] = d;
      }
      d = mat2[i];
      mat2[i] = mat2[lindex];
      mat2[lindex] = d;
    }

    /* Make sure we have a non-zero pivot. */
    if (mat1[i * n + i] == 0.0) {
      /* this should be rotated. */
      mxt_err(ctx, "ERROR: Non-zero pivot");
      return false;
    }
    for (j = i + 1; j < n; j++) {
      d = mat1[j * n + i] / mat1[i * n + i];
      for (k = 0; k < n; k++)
        mat1[j * n + k] -= d * mat1[i * n + k];
      mat2[j] -= d * mat2[i];
    }
  }

  for (i = n - 1; i > 0; i--)
    for (j = i - 1; j >= 0; j--) {
      d = mat1[j * n + i] / mat1[i * n + i];
      for (k = 0; k < n; k++)
        mat1[j * n + k] -= d * mat1[i * n + k];
      mat2[j] -= d * mat2[i];
    }

  /* Now write the stuff into the result vector. */
  for (i = 0; i < n; i++) {
    result[i] = mat2[i] / mat1[i * n + i];
    mxt_verb(ctx, "coeff[%d] = %f", i, result[i]);
  }

  return true;
}
