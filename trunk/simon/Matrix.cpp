/**
 * Simon is the legal property of its developers, whose names are too
 * numerous to list here.  Please refer to the COPYRIGHT file
 * distributed with this source distribution.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#include "Matrix.h"

#include <iostream>

using namespace std;

/**
 * \file Matrix.cpp
 * \class Matrix
 * \brief Matrix-class
 *
 * related functions to the matrix template class.
 * 
 * 
 */


/** @brief Cholesky Factorization, Columns Gaxpy version.
 *
 * Given a symmetric positive definite Matrix \f$\mathbf{a} \in \mathbf{R}^{N
 * \times N}\f$, the algorithm computes a lower triangular \f$\mathbf{l} \in
 * \mathbf{R}^{N \times N}\f$ such that \f$\mathbf{a} = \mathbf{l}
 * \mathbf{l}^T\f$. See Gene H. Golub and Charles F. van Loan "Matrix
 * Computations, 3rd ed.", 4.2.4 for further details. The gaxpy was written
 * out.
 *
 * @param a A symmetric positive definite matrix \f$\mathbf{a} \in
 * \mathbf{R}^{N \times N}\f$, that is overwritten by the lower triangular
 * matrix \f$\mathbf{l}\f$ of the cholesky factorization \f$\forall i \le j\f$.
 *
 * @author Nils Hornung <hornung@uni-koblenz.de>
 */
template <unsigned int N>
bool choleskyFactorizationColumns (Matrix <float, N, N> & a)
{
  for (unsigned int j=0; j<N; ++j) {
    for (unsigned int k=0; k<j; ++k) {
      a [j][j] -= a [j][k] * a [j][k];
    }
    if (a [j][j] <= 0.0f)
      return false;
    a [j][j] = (float) sqrt (a [j][j]);
    for (unsigned int i=j+1; i<N; ++i) {
      for (unsigned int k=0; k<j; ++k) {
        a [i][j] -= a [i][k] * a [j][k];
      }
//      if (i==j) {
//        if (a [j][j] <= 0.0f)
//          return false;
//        a [j][j] = (float) sqrt (a [j][j]);
//      }
//      else {
        a [i][j] /= a [j][j];
//      }
    }
  }
  return true;
}

/** @brief Cholesky Factorization, Rows Gaxpy version.
 *
 * Given a symmetric positive definite Matrix \f$\mathbf{a} \in \mathbf{R}^{N
 * \times N}\f$, the algorithm computes an upper triangular \f$\mathbf{u} \in
 * \mathbf{R}^{N \times N}\f$ such that \f$\mathbf{a} = \mathbf{u}^T
 * \mathbf{u} \f$. We derived the implementation similar to the columns version
 * in Gene H. Golub and Charles F. van Loan "Matrix Computations, 3rd ed.",
 * 4.2.4. All formulas simply are changed to be work on the upper triangular
 * part of \f$\mathbf{a}\f$. Nevertheless the right hand side of the inner loop
 * is still accessed by columns. The gaxpy operation was written out.
 *
 * @param a A symmetric positive definite matrix \f$\mathbf{a} \in
 * \mathbf{R}^{N \times N}\f$, that is overwritten by the upper triangular
 * matrix \f$\mathbf{u}\f$ of the cholesky factorization \f$\forall i \le j\f$.
 */
template <unsigned int N>
bool choleskyFactorizationRows (Matrix <float, N, N> & a)
{
  for (unsigned int i=0; i<N; ++i) {
    for (unsigned int k=0; k<i; ++k) {
      a [i][i] -= a [k][i] * a [k][i];
    }
    if (a [i][i] <= 0.0f)
      return false;
    a [i][i] = (float) sqrt (a [i][i]);
    for (unsigned int j=i+1; j<N; ++j) {
      for (unsigned int k=0; k<i; ++k) {
        a [i][j] -= a [k][i] * a [k][j];
      }
      a [i][j] /= a [i][i];
    }
  }
  return true;
}

/** @brief Inverse using Cholesky Factorization.
 *
 * Please note that the matrix to be inverted has to by symmetric positive or
 * negative definite.
 * @todo We should offer an alternative row version.
 *
 * @param aa The matrix to be inverted
 * @return The inverted matrix
 */
template <unsigned int N>
Matrix <float, N, N> choleskyInverse (const Matrix <float, N, N> & aa)
{
  Matrix <float, N, N> x; // return value
  bool positiveDefinite = true;
  Matrix <float, N, N> a = aa;
  if (!choleskyFactorizationColumns (a)) {
    // do the same exception handling as below...
    positiveDefinite = false;
    a = aa * (-1);
    if (!choleskyFactorizationColumns (a)) {
      std::cout << "Cholesky factorization failed.\n"
        << "The Matrix to be inverted is not positive or negative definite and symmetric!\n"
        << "Its inverse cannot be computed by the cholesky method so.\n"
        << "This error might have been caused by previous faults or by bad init values.\n"
        << "Contact: stpalmer@uni-koblenz.de or hornung@uni-koblenz.de\n";
    }
  }

  // compute a^(-1), a is lower triangular
  Matrix <float, N, N> temp;
  for (unsigned int k=0; k<N; ++k) {
    temp [k][k] = 1.0f / a [k][k];
    for (unsigned int i=k+1; i<N; ++i) {
      temp [i][k] = 0.0f;
      for (unsigned int j=k; j<i; ++j) {
        temp [i][k] -= a [i][j] * temp [j][k];
      }
      temp [i][k] /= a [i][i];
    }
  }
  
  // now overwrite x with original a's inverse
  // i.e. a^(-1)^T * a^(-1)
  for (unsigned int i=0; i<N; ++i)
    for (unsigned int j=0; j<N; ++j) {
      x [i][j] = 0.0f;
      for (unsigned int k=MAX(i,j); k<N; ++k)
        x [i][j] += temp [k][i] * temp [k][j];
    }

  if (positiveDefinite)
    return x;
  return x * (-1);
}


template Matrix3x3 choleskyInverse(const Matrix3x3 &);
template Matrix5x5 choleskyInverse(const Matrix5x5 &);
template Matrix6x6 choleskyInverse(const Matrix6x6 &);
template bool choleskyFactorizationColumns (Matrix2x2 &);
template bool choleskyFactorizationRows (Matrix2x2 &);
template Matrix2x2 choleskyInverse(const Matrix2x2 &);

