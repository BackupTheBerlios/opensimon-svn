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


//------------------------------------------------------------------------------
/**
 * \file Matrix.h
 * \class Matrix
 * \brief Matrix class using templates
 * 
 * This file contributes a matrix class.
 *
 * This class uses templates, which means the data type of the matrix content
 * may be chosen somewhat freely (at compile-time).
 * Because of the usage of  templates, there is no code for the template itself
 * in the .cpp file for this .h file. In the .cpp file, there is code for 
 * related functions.
 * 
 * As content type, you can use all types/classes, for which the following
 * operations are defined (*with sense*) / properties apply:
 * - assign float/double
 * - assigning 0 must/should create the 0-element
 * - assign the type itself
 * - addition
 * - multiplication
 * - cast to double
 * - no template type
  
 * Some examples:
 * - Matrix<float,3,3> my3x3FloatMatrix();
 * - Matrix<MyComplexNumberClass,2,100> my2x100ComplexMatrix();
 * - Matrix<MyFancyClassWithAllTheStuffMentionedAbove,3,3> myMatrx(); 
 *
 * Original version by Rodja Trappe
 *
 * <b>Attention!</b> Please note that not all commentaries are up to date
 * because of basic implementation changes.
 */
//------------------------------------------------------------------------------

#ifndef MATRIX_H
#define MATRIX_H

#include <simon/config.h>
#include <simon/Vector3.h>

#include <cstring>

template<class Element, unsigned int Rows, unsigned int Columns> class Matrix {
private:

	//! Array with pointers to the row arrays
	//! \todo Is this efficient !??!
	Element matrix[Rows][Columns];

public: 
	// Construction/destruction

	//! Standard constructor
	Matrix();

	//! Copy contructor
	Matrix(const Matrix<Element, Rows, Columns>&);

	//! Contructor with initial value array
	Matrix(const Element* const initialValues);   

	//! Contructor with initial value for all elements
	Matrix(const Element initialValue);   

	//! Constructor with Vector3
	Matrix(const Vector3<Element> & vec);

	//! desctructor
	~Matrix();


	//Operators		

	//! Access to the matrix rows / row arrays
	Element* operator[] (unsigned int);

	//! Readonly access to the matrix rows / row arrays
	const Element* operator[] (unsigned int) const;

	//! Assignment of a matrix
	void operator= (const Matrix&);

	//! Assign v to each matrix (e.g. to initialize)
	void operator= (const Element& v);

	//! Matrix multiplication
	template <unsigned int Columns2>
		Matrix<Element, Rows, Columns2> operator *(const Matrix<Element, Columns, Columns2>& m) const;

	//! Replaces operator * for performance purpose
	template <unsigned int Columns2>
		void multiply (const Matrix<Element, Columns, Columns2>& mult, Matrix<Element, Rows, Columns2> & prod) const;

	//! Matrix multiplication
	Matrix<Element, Rows, 1> operator * (const Vector3<Element>&) const;

	//! Matrix multiplication (fast)
	void multiply (const Vector3<Element>&, Matrix<Element, Rows, 1> &) const;

	//! Matrix addition (components)
	Matrix<Element, Rows, Columns> operator + (const Matrix<Element, Rows, Columns>&) const;

	//! Replaces operator + for performance purpose
	void add (const Matrix<Element, Rows, Columns>& addent, Matrix<Element, Rows, Columns> & sum) const;

	//! Matrix subtraction (components)
	Matrix<Element, Rows, Columns> operator - (const Matrix<Element, Rows, Columns> &) const;

	//! Multiplication Matrix * Element
	Matrix<Element, Rows, Columns> operator * (const Element) const;

	//! Matrix equality test
	template <unsigned int Rows2, unsigned int Columns2>
		bool operator ==(const Matrix<Element, Rows2, Columns2> &) const;

	//! Matrix not equal test
	template <unsigned int Rows2, unsigned int Columns2>
		bool operator !=(const Matrix<Element, Rows2, Columns2> &) const;

		
	//get-methods		

	//! Returns height/number of rows
	unsigned int getSizeM() const {return Rows;} 

	//! Returns height/number of rows
	unsigned int getSizeRows() const {return Rows;} 

	//! Returns width/number of columns
	unsigned int getSizeN() const {return Columns;} 

	//! Returns width/number of columns
	unsigned int getSizeColumns() const {return Columns;} 


	//! createors

	//! create an identity matrix
	static Matrix<Element, Rows, Rows> createIdentity(); 

	//! create an identity matrix
	static Matrix<Element, Rows, Rows> createDiagonal(Element value); 
		
	// Misc. methods

	//! print the matrix, only works with standard C++ types at the moment
	void show() const;
		
	//! Return the transposed matrix
	Matrix<Element, Columns, Rows> T() const; 		

	//! Return matrix star
	Matrix<Element, 3, 3> star() const;

	//! Return zero matrix
	Matrix<Element, Rows, Columns> zero() const; 

	//! Return Jacobian
	template <unsigned int Rows2>
		Matrix<Element, Rows, 1> jacobian (const Matrix<Element, Rows2, 1> &);

	void get3x3Matrix (unsigned int, unsigned int, Matrix<Element, 3, 3> &) const;
	template <unsigned int M, unsigned int N>
		void getMxNMatrix (unsigned int, unsigned int, Matrix<Element, M, N> &) const;

	//! copys the values of a given matrix into this matrix
	template <unsigned int Rows2, unsigned int Columns2>
		void setValues(const Matrix<Element, Rows2, Columns2> &);

	//! nice output to ostream
	//! \todo implementation weiter unten machen!
	friend std::ostream& operator <<(std::ostream& os, const Matrix<Element, Rows, Columns> &matrix) {
		for (unsigned int i = 0; i < Rows; ++i) {
			for (unsigned int j = 0; j < Columns; ++j) {
				os << matrix[i][j] << "   ";
			}
			os << std::endl;
		}
		return os;	
	}

};

//related functions

//! Returns the inverse, if matrix is positive/negative definite
template<unsigned int Rows, unsigned int Columns> 
Matrix<float, Rows, Columns> choleskyInverseOld (const Matrix<float, Rows, Columns>&); 

template <unsigned int N> bool choleskyFactorizationColumns (Matrix <float, N, N> & a);
template <unsigned int N> bool choleskyFactorizationRows (Matrix <float, N, N> & a);                                                                                                                                                            template <unsigned int N, unsigned int M> bool choldc (Matrix <float, N, M> & a,    Matrix <float, N, 1> & p);

template <unsigned int N>
Matrix <float, N, N> choleskyInverse (const Matrix <float, N, N> & aa);

//! Returns m, where every element is multiplyed with e
template<class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, Columns> operator *(const Element &e, const Matrix<Element, Rows, Columns> &m);

// ######## Implementation ##############


/*
	\brief Default Constructor
	                               
	Create a Matrix with M rows and Columns columns of type Element
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, Columns>::Matrix() { 
	
	//! \todo wegmachen!
	//*this = 0;
}

/*
	\brief Default Constructor
	                               
	Create a Matrix with M rows and Columns columns of type Element
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, Columns>::Matrix(const Element initialValue) { 

	//! \todo This should be optional
	*this = initialValue;//initialize with 0
}

/*
	\brief Constructor with rows and columns and initial values array
	\param Element* initialValues, one-dimensional(!) array with initial values
	                               
	Create a Matrix with "Rows" and "Columns". The initial values are given with
	a pointer to an one-dimensional(!) array. This must have the right size, or
	the	program will crash! The array must hold the lines of the matrix, one
	line after the other.
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, Columns>::Matrix(const Element* const initialValues) { 

	memcpy(matrix, initialValues, Rows * Columns * sizeof(Element));
}

/*
	\brief	constructor with a Vector3
	\param	Vector3

	Create a Matrix (3,1) with argument of a 3-dimensional Vector of class
	Vector3.
*/
template<class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, Columns>::Matrix(const Vector3 <Element>& vec) {

	assert(Columns == 1 && Rows == 3);

	// fill with values from original
	for (unsigned int i = 0; i < 3 ; ++i) {
			matrix[i][0] = vec[i];
	}
}

/*!
	\brief Copy-constructor
	\param Matrix<Element, Rows, Columns>, matrix to be copied.

	Create a Matrix as a copy of an other
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, Columns>::Matrix(const Matrix<Element, Rows, Columns> &m) { 

	memcpy(matrix, m.matrix[0], Rows * Columns * sizeof(Element));
}


/*!
  \brief Destructor
  
  Destroy the matrix<Element>
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, Columns>::~Matrix() {
	// nn
}


/*!
  \brief Show matrix

  Prints the matrix to stdio
  //! \todo better write some stpd::ostream overloading
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
void Matrix<Element, Rows, Columns>::show () const { 

	cout << *matrix << endl;
}


/*!
  \brief Index operator "[]"
  \param int m, number of row in matrix
  \return      

  Returning the matrix row at position m
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
Element* Matrix<Element, Rows, Columns>::operator[](const unsigned int m) { 

	return matrix[m]; 
}

/*!
  \brief Const index operator "[]"
  \param int m, number of row in matrix
  \return pointer to row of matrix

  Read-only variant of: returning the matrix row at position m
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
const Element* Matrix<Element, Rows, Columns>::operator [](const unsigned int m) const { 

	return matrix[m]; 
}

/*!
	\brief Assignement operator 
	\param Matrix<Element, Rows, Columns>, matrix
                                                                                
	Assign one matrix to an other
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
					 void Matrix<Element, Rows, Columns>::operator=(const Matrix<Element, Rows, Columns> &m) { 
	
	// Do not allow assining a matrix to itself! To rare, only costly.
	assert( this != &m );

/* 
// we can skip a lot if the matrices have the same size.
if (m.getSizeRows() != Rows || m.getSizeColumns() != Columns) {


//! \todo is !matrix possible?
if (matrix) {
//free memory for matrix data
delete[] matrix[0];
//free memory for row pointers
delete[] matrix;
}

// vector of size x*y 
Element* array = new Element[Rows * Columns];
// generate T matrix 
matrix = new Element*[Rows];

// fill in vector pointers 
for (unsigned int i = 0; i < Rows; ++i) {
matrix[i] = & (array[i*Columns]); 

}
}
*/	
 	memcpy(matrix[0], m.matrix[0], Rows * Columns*sizeof(Element));

}

/*!
  \brief Assignment operator 
  \param v, the matrix element type
                                                                                
  Assign Element to each element of the matrix, e.g. to initialize.
*/
template <class Element, unsigned int Rows, unsigned int Columns> void Matrix<Element, Rows, Columns>::operator=(const Element& v) { 
    	
	//! \todo consider changing to memfill() here, especially when this is called
	//! in every constructor.
	// fill with a constant calue
	for (unsigned int i = 0; i < Rows * Columns ; ++i) {
		matrix[0][i] = v;
	}
}


/*!
  \brief multiply two matrixes (operator *)
  \param Matrix<Element, Columns, Columns2> m
  \return Matrix<Element, Rows, Columns2>, the multiplyed matrixes
  
  Multiply operator matrix * Element
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
	template <unsigned int Columns2> 
Matrix<Element, Rows, Columns2> Matrix<Element, Rows, Columns>::operator *(const Matrix<Element, Columns, Columns2>& m) const {

	Matrix<Element, Rows, Columns2> prod;

	multiply(m,prod);

	return prod;
}

/*!
  \brief multiply two matrixes (operator *) for performance
  The user has to assert the size of the result matrix!
  \todo not yet tested!
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
	template <unsigned int Columns2> 
void Matrix<Element, Rows, Columns>::multiply (const Matrix<Element, Columns, Columns2> & mult, Matrix<Element, Rows, Columns2> & prod) const {

  for (unsigned int i = 0; i < Rows; ++i) {
    for (unsigned int j = 0; j < Columns2; ++j) {
      prod[i][j] = 0;
      for (unsigned int k = 0; k < Columns; ++k) {
        prod[i][j] += (matrix[i][k] * mult[k][j]);
      }
    }
  }

}

/*!
  \brief Multiply a matrix with a Vector3<Type> (operator *)
  \param v The Vector to multiply with.
  \return the product

  Multiply operator matrix * Element
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, 1> Matrix<Element, Rows, Columns>::operator * (const Vector3<Element>& v) const {

	assert(Columns==3);
	
	Matrix<Element,Rows,1> prod;  
	multiply(v,prod);
	
	return prod;
}
 
/*!
  \brief Multiply a matrix with a Vector3<Type> (operator *)
  \param v The Vector to multiply with.
  \param product result of the operation
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
void Matrix<Element, Rows, Columns>::multiply (const Vector3<Element> &v, Matrix<Element, Rows, 1> &product) const
{
	assert(Columns==3);

	Element sum;
	for (unsigned int i = 0; i < Rows; ++i) {
		sum = 0;
		for (unsigned int j = 0; j < 3; ++j) {
			sum = sum + matrix[i][j] * v[j];
		}
		if (i < 3)
			product[i][0] = sum;
		else
			product[i][0] = matrix[i][0];
	}
}

/*!
  \brief multiply with a scalar (matrix * Element)
  \param e the scalar
  \return Matrix<Element, Rows, Columns>, the result matrix

  Multiply operator
*/
template<class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, Columns> Matrix<Element, Rows, Columns>::operator *(const Element e) const {
	
	//copy of matrix
	Matrix<Element, Rows, Columns> prod(*this);

	for (unsigned int i = 0; i < Rows; ++i) {
		for (unsigned int j = 0; j < Columns; ++j) {
			prod[i][j] *= e;
		}
	}
	return prod;
}

/*!
  \brief Multiply Element * matrix (operator *)
  \param e the scalar
  \param Matrix<Element, Rows, Columns> m
  \return Matrix<Element, Rows, Columns>, the multiplyed matrix

  Multiply operator Element * matrix
*/
template<class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, Columns> operator *(const Element &e, const Matrix<Element, Rows, Columns>& m) {
	return m * e;
}

/*!
  \brief Subtract two matrixes (operator -)
  \param Matrix<Element, Rows, Columns> m
  \return Matrix<Element, Rows, Columns>, the subtracted matrixes

  Subtract operator, works per component.
*/
template<class Element, unsigned int Rows, unsigned int Columns> 
	Matrix<Element, Rows, Columns> 
	Matrix<Element, Rows, Columns>::operator -(const Matrix<Element, Rows, Columns> &m) const {

	Matrix<Element, Rows, Columns> sub;
	//Element sum;
	for (unsigned int i = 0; i < Rows; ++i) {
		for (unsigned int j = 0; j < Columns; ++j) {
			sub[i][j] = matrix[i][j] - m[i][j];
		}
	}
	return sub;
}

/*!
  \brief Add two matrixes per component (operator +)
  \param Matrix<Element, Rows, Columns> m
  \return Matrix<Element, Rows, Columns>, the added matrixes

  Addition operator, works per component.
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, Columns> 
Matrix<Element, Rows, Columns>::operator + (const Matrix<Element, Rows, Columns>& m) const {

	Matrix<Element, Rows, Columns> sum;
	add(m, sum);
	return sum;
}

/*!
 \brief Addition of matrices for performance

 The user is in charge!
 \todo not yet tested!
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
void Matrix<Element, Rows, Columns>::add (const Matrix<Element, Rows, Columns> & addent, Matrix<Element, Rows, Columns> & sum) const {

  for (unsigned int i = 0; i < Rows; ++i) {
    for (unsigned int j = 0; j < Columns; ++j) {
      sum [i][j] = matrix [i][j] + addent [i][j];
    }
  }
}


/**
 * \brief Matrix equality test
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
	template <unsigned int Rows2, unsigned int Columns2>
	bool Matrix<Element, Rows, Columns>::operator ==(const Matrix<Element, Rows2, Columns2> &m) const {
	if (Rows == Rows2 && Columns == Columns2) {
		bool isEqual = true;
		for (unsigned int i = 0; i < Rows; ++i)
			for (unsigned int j = 0; j < Columns; ++j)
				if (matrix[i][j] != m[i][j])
					return false;
		if (isEqual)
			return true;
	}
	return false;
}

	
/**
 * \brief Matrix not equal test
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
	template <unsigned int Rows2, unsigned int Columns2>
	bool Matrix<Element, Rows, Columns>::operator !=(const Matrix<Element, Rows2, Columns2> &m) const {
	return !(*this == m);
}



/*!
  \brief Transposed matrix
  \return Matrix<Element, Rows, Columns>, the transposed matrix

  This method returns the transposed matrix to the original matrix.
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Columns, Rows> Matrix<Element, Rows, Columns>::T() const {

	Matrix<Element, Columns, Rows> transposed;
	for (unsigned int i = 0; i < Rows; ++i) {
		for (unsigned int j = 0; j < Columns; ++j) {
			transposed[j][i] = matrix[i][j];
		}
	}
	return transposed;
}

/*!
  \brief Star operator
  \return Matrix<Element, 3, 3>, the star (3,3) of matrix (3,1) 

  This method returns the so called star-matrix.
  Warning: The class Element has to be able to provide an element "0".
  This is no problem as it only makes sense to use it with float or double
  or any other similar type. See also the header of this file.
*/
template<class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, 3, 3> Matrix<Element, Rows, Columns>::star() const {

	assert (Rows == 3 && Columns == 1);
	Matrix<Element, 3, 3> starMatrix(0.0);
	for (unsigned int i = 0; i < 3; ++i)
		starMatrix [i][i] = 0;

	starMatrix [0][1] = - matrix [2][0];
	starMatrix [0][2] =   matrix [1][0];
	starMatrix [1][0] =   matrix [2][0];
	starMatrix [1][2] = - matrix [0][0];
	starMatrix [2][0] = - matrix [1][0];
	starMatrix [2][1] =   matrix [0][0];

	return starMatrix;
}

/*!
  \brief Zero operator
  \return Matrix<Element, Rows, Columns>, the zero matrix of given size

  This method returns the so called zero-matrix.
  Warning: The class Element has to be able to provide an element "0".
  This is no problem as it only makes sense to use it with float or double
  or any other similar type. See also the header of this file.
*/
template<class Element, unsigned int Rows, unsigned int Columns> 
Matrix<Element, Rows, Columns> Matrix<Element, Rows, Columns>::zero() const{

	Matrix<Element, Rows, Columns> zeroMatrix;
	return zeroMatrix = 0;
}

/*!  
  \brief Identity Matrix createor.  
  \return Matrix<Element, Rows,
  Columns>, the identity matrix of given size

  This method returns the so called zero-matrix. Designed with the 
  "Factory Method Pattern" form Gamma et al.

  <b>Examples:</b>

  To create a 3x3 identity float Matrix call:
  Matrix<float, 3, 3> m = Matrix<float, 3, 3>::createIdentity();


  <b>Warning:</b> The class Element has to be able to provide an element "0" and "1".
  This is no problem as it only makes sense to use it with float or double
  or any other similar type. See also the header of this file.
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
	Matrix<Element, Rows, Rows> Matrix<Element, Rows, Columns>::createIdentity(){

	return createDiagonal(1.0);
}


/*!  
  \brief Diagonal Matrix createor.  
  
  \param value The value which should be filled in.
  \return Matrix<Element, Rows, Columns>, a matrix of given size with it's
  diagonal filled with a given Element.

  This method returns a diagonal Matrix with a given Element.
  Designed with the "Factory Method Pattern" form Gamma et al.

  <b>Examples:</b>

  To create a 3x3 float Matrix with 2.2 in diagonal call:
  Matrix<float, 3, 3> m = Matrix<float, 3, 3>::createDiagonal(2.2);
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
	Matrix<Element, Rows, Rows> 
	Matrix<Element, Rows, Columns>::createDiagonal(Element value){

	assert(Rows == Columns);
	Matrix<Element, Rows, Rows> diagonalMatrix(0.0f);
	for (unsigned int i = 0; i < Rows; ++i) {
		diagonalMatrix[i][i] = value;
	}
	return diagonalMatrix;
}


/*!
  \brief Jacobian
  \return jacobian

  This method returns the Jacobian of two vectors. Columnsote that
  it cannot be called from a Matrix with Columns other than 1, neither
  can it be given an argument Matrix with Columns other than 1. The
  vectors have to be the "virtual" differences, i.e. real differences
  as we can do nothing but descrete computations.

  Completely outdated and never finished. Should never ever be used!!!
*/
template <class Element, unsigned int Rows, unsigned int Columns> 
	template <unsigned int Rows2>
Matrix<Element, Rows, 1> Matrix<Element, Rows, Columns>::jacobian (const Matrix<Element, Rows2, 1> &deltaArgument)
{
	assert(Columns == 1);
                                                                                 
	Matrix<Element, Rows, Rows2> jacobian;
	for (unsigned int i = 0; i < Rows; ++i) {
		for (unsigned int j = 0; j < Rows2; ++j) {
			if (deltaArgument [j][0] != 0.0) {
				jacobian = matrix [i][0] /
					deltaArgument [j][0];
			}
		}
	}
	return jacobian;

}

/*
 * \brief get matrix part of matrix
 * \param matrix3x3 destiny copied by reference
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
void Matrix<Element, Rows, Columns>::get3x3Matrix (unsigned int ii, unsigned int jj, Matrix<Element, 3, 3> & matrix3x3) const {

  assert (Rows >= ii+3 && Columns >= jj+3);
  assert (matrix3x3.getSizeRows () == 3 &&
          matrix3x3.getSizeColumns () == 3);

  for (unsigned int i=0; i<3; i++) {
    for (unsigned int j=0; j<3; j++) {
      matrix3x3 [i][j] = matrix [i+ii][j+jj];
    }
  }

}


/**
 * The given matrix needs to be <= the actual one!
 *
 * \param m Matrix which should be written into the actual
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
template <unsigned int Rows2, unsigned int Columns2> 
void Matrix<Element,Rows,Columns>::setValues(const Matrix<Element, Rows2, Columns2> &m) {

	//! \todo muss wieder rein. Gefahr in verzug!
	//assert(Rows2 <= Rows && Columns <= Columns2);

	for (unsigned int i=0; i < Rows2; ++i)
	  for (unsigned int j=0; j < Columns2; ++j)
		matrix[i][j] = m[i][j];
}

/*
 * \brief get matrix part of matrix
 * \param matrixMxColumns destiny copied by reference
 */
template <class Element, unsigned int Rows, unsigned int Columns> 
	template <unsigned int M, unsigned int N> 
	void Matrix<Element, Rows, Columns>::getMxNMatrix (unsigned int ii, unsigned int jj, Matrix<Element, M, N> &resultMatrix) const{

  for (unsigned int i=0; i < M; i++) {
    for (unsigned int j=0; j < N; j++) {
      resultMatrix[i][j] = matrix [i+ii][j+jj];
    }
  }
}

//! tasty typedefs for easy handling of standard matrices
typedef Matrix<float,1,1> Matrix1x1;
typedef Matrix<float,1,3> Matrix1x3;
typedef Matrix<float,3,1> Matrix3x1;
typedef Matrix<float,3,3> Matrix3x3;
typedef Matrix<float,3,6> Matrix3x6;
typedef Matrix<float,5,1> Matrix5x1;
typedef Matrix<float,5,5> Matrix5x5;
typedef Matrix<float,5,6> Matrix5x6;
typedef Matrix<float,6,1> Matrix6x1;
typedef Matrix<float,6,5> Matrix6x5;
typedef Matrix<float,6,6> Matrix6x6;
typedef Matrix<float,2,2> Matrix2x2;
typedef Matrix<float,2,1> Matrix2x1;

#endif
