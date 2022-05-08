#pragma once
#ifndef _H_MATH_MATRICES_
#define _H_MATH_MATRICES_

#include "vectors.h"

//			STRUCTURE DEFINITIONS
// MAT2
typedef struct mat2
{
	union
	{
		struct
		{
			float _11, _12,
				_21, _22;
		};
		float asArray[4];
	};

	// Default inline Constructor
	inline mat2()
	{
		_11 = _22 = 1.0f;
		_12 = _21 = 0.0f;
	}

	inline mat2(float f11, float f12,
		float f21, float f22)
	{
		_11 = f11; _12 = f12;
		_21 = f21; _22 = f22;
	}

	inline float* operator[] (int i)
	{
		return &(asArray[i * 2]);
	}
} mat2;

// MAT3
typedef struct mat3
{
	union
	{
		struct
		{
			float _11, _12, _13,
				_21, _22, _23,
				_31, _32, _33;
		};
		float asArray[9];
	};

	// Default inline Constructor
	inline mat3()
	{
		_11 = _22 = _33 = 1.0f;
		_12 = _13 = _21 = 0.0f;
		_23 = _31 = _32 = 0.0f;
	}

	inline mat3(float f11, float f12, float f13,
		float f21, float f22, float f23,
		float f31, float f32, float f33)
	{
		_11 = f11; _12 = f12; _13 = f13;
		_21 = f21; _22 = f22; _23 = f23;
		_31 = f31; _32 = f32; _33 = f33;
	}

	inline float* operator[] (int i)
	{
		return &(asArray[i * 3]);
	}
} mat3;

// MAT4
typedef struct mat4
{
	union
	{
		struct
		{
			float _11, _12, _13, _14,
				_21, _22, _23, _24,
				_31, _32, _33, _34,
				_41, _42, _43, _44;
		};
		float asArray[16];
	};

	// Default inline Constructor
	inline mat4()
	{
		_11 = _22 = _33 = _44 = 1.0f;
		_12 = _13 = _14 = _21 = 0.0f;
		_23 = _24 = _31 = _32 = 0.0f;
		_34 = _41 = _42 = _43 = 0.0f;
	}

	inline mat4(float f11, float f12, float f13, float f14,
		float f21, float f22, float f23, float f24,
		float f31, float f32, float f33, float f34,
		float f41, float f42, float f43, float f44)
	{
		_11 = f11; _12 = f12; _13 = f13; _14 = f14;
		_21 = f21; _22 = f22; _23 = f23; _24 = f24;
		_31 = f31; _32 = f32; _33 = f33; _34 = f34;
		_41 = f41; _42 = f42; _43 = f43; _44 = f44;
	}

	inline float* operator[] (int i)
	{
		return &(asArray[i * 4]);
	}
} mat4;


//			METHOD DECLARATIONS
// Transpose
// Gereric Transpose
void Transpose(const float *srcMat, float *dstMat,
	int srcRows, int srcCols);
// Specific Matrices Transposes
mat2 Transpose(const mat2& matrix);
mat3 Transpose(const mat3& matrix);
mat4 Transpose(const mat4& matrix);

// Multiplication
// Scalar Multiplication
mat2 operator* (const mat2& matrix, float scalar);
mat3 operator* (const mat3& matrix, float scalar);
mat4 operator* (const mat4& matrix, float scalar);
// Matrix-Matrix Multiplication
// Generic Multiplication
bool Multiply(float* out, const float* matA, int aRows,
	int aCols, const float* matB, int bRows, int bCols);
// Specific Matrices Multiplications
mat2 operator* (const mat2& matA, const mat2& matB);
mat3 operator* (const mat3& matA, const mat3& matB);
mat4 operator* (const mat4& matA, const mat4& matB);

// Determinant
// 2x2 Matrix Determinant
float Determinant(const mat2& matrix);

// Matrix of Minors
mat2 Cut(const mat3& mat, int row, int col);
mat2 Minor(const mat2& mat);
mat3 Minor(const mat3& mat);

#endif