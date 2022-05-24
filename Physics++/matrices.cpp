#include "matrices.h"
#include <cmath>
#include <cfloat>

#define CMP(x, y) (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

	//		METHOD IMPLEMENTATIONS

// TRANSPOSE
// Generic Transpose
void Transpose(const float* srcMat, float* dstMat,
	int srcRows, int srcCols)
{
	for (int i = 0; i < srcRows * srcCols; i ++)
	{
		int row = i / srcRows;
		int col = i % srcRows;
		dstMat[i] = srcMat[srcCols * col + row];
	}
}

// Specific Matrices Transposes
mat2 Transpose(const mat2& matrix)
{
	mat2 result;
	Transpose(matrix.asArray, result.asArray, 2, 2);
	return result;
}

mat3 Transpose(const mat3& matrix)
{
	mat3 result;
	Transpose(matrix.asArray, result.asArray, 3, 3);
	return result;
}

mat4 Transpose(const mat4& matrix)
{
	mat4 result;
	Transpose(matrix.asArray, result.asArray, 4, 4);
	return result;
}

// Multiplacation
// Scalar Multiplication
mat2 operator* (const mat2& matrix, float scalar)
{
	mat2 result;
	for (int i = 0; i < 4; i++)
	{
		result.asArray[i] = matrix.asArray[i] * scalar;
	}
	return result;
}

mat3 operator* (const mat3& matrix, float scalar)
{
	mat3 result;
	for (int i = 0; i < 9; i++)
	{
		result.asArray[i] = matrix.asArray[i] * scalar;
	}
	return result;
}

mat4 operator* (const mat4& matrix, float scalar)
{
	mat4 result;
	for (int i = 0; i < 16; i++)
	{
		result.asArray[i] = matrix.asArray[i] * scalar;
	}
	return result;
}

// Matrix-Matrix Multiplication
// Generic Multiplication
bool Multiply(float* out, const float* matA, int aRows,
	int aCols, const float* matB, int bRows, int bCols)
{
	if (aCols != bRows)
	{
		return false;
	}

	for (int i = 0; i < aRows; i++)
	{
		for (int j = 0; j < bCols; j++)
		{
			out[bCols * i + j] = 0.0f;
			for (int k = 0; k < bRows; k++)
			{
				int a = aCols * i + k;
				int b = bCols * k + j;
				out[bCols * i + j] += matA[a] * matB[b];
			}
		}
	}

	return true;
}

// Specific Matrix-Matrix Multiplications
mat2 operator* (const mat2& matA, const mat2& matB)
{
	mat2 result;
	Multiply(result.asArray, matA.asArray, 
		2, 2, matB.asArray, 2, 2);
	return result;
}

mat3 operator* (const mat3& matA, const mat3& matB)
{
	mat3 result;
	Multiply(result.asArray, matA.asArray,
		3, 3, matB.asArray, 3, 3);
	return result;
}

mat4 operator* (const mat4& matA, const mat4& matB)
{
	mat4 result;
	Multiply(result.asArray, matA.asArray,
		4, 4, matB.asArray, 4, 4);
	return result;
}

// Determinants
// Determinant of a 2x2 Matrix
float Determinant(const mat2& matrix)
{
	return matrix._11 * matrix._22 -
		matrix._12 * matrix._21;
}

// Determinant of a 3x3 Matrix
float Determinant(const mat3& matrix)
{
	float result = 0.0f;
	mat3 cofactor = Cofactor(matrix);

	for (int j = 0; j < 3; ++j)
	{
		int index = 3 * 0 + j;
		result += matrix.asArray[index] * cofactor[0][j];
	}

	return result;
}

// Matrix of Minors
mat2 Cut(const mat3& mat, int row, int col)
{
	mat2 result;
	int index = 0;

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (i == row || j == col)
			{
				continue;
			}
			int target = index++;
			int source = 3 * i + j;
			result.asArray[target] = mat.asArray[source];
		}
	}

	return result;
}

mat2 Minor(const mat2& mat)
{
	return mat2(mat._22, mat._21,
		mat._12, mat._11);
}

mat3 Minor(const mat3& mat)
{
	mat3 result;

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++ j)
		{
			result[i][j] = Determinant(Cut(mat, i, j));
		}
	}
}

// Cofactor
// Generic Cofactor
void Cofactor(float* out, const float* minor,
	int rows, int cols)
{
	for (int i= 0; i < rows; ++i)
	{
		for (int j = 0; j < cols; ++j)
		{
			int t = cols * j + i; // Target index
			int s = cols * j + i; // Source index
			float sign = powf(-1.0, i + j); // + or -
			out[t] = minor[s] * sign;
		}
	}
}

// Specific Matrix Cofactors
mat2 Cofactor(const mat2& mat)
{
	mat2 result;
	Cofactor(result.asArray, Minor(mat).asArray, 2, 2);
	return result;
}

mat3 Cofactor(const mat3& mat)
{
	mat3 result;
	Cofactor(result.asArray, Minor(mat).asArray, 3, 3);
	return result;
}

// For operations on 4x4 matrices
mat3 Cut(const mat4& mat, int row, int col)
{
	mat3 result;
	int index = 0;

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			if (i == row || j == col)
			{
				continue;
			}
			int target = index++;
			int source = 4 * i + j;
			result.asArray[target] = mat.asArray[source];
		}
	}

	return result;
}

mat4 Minor(const mat4& mat)
{
	mat4 result;

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			result[i][j] = Determinant(Cut(mat, i, j));
		}
	}
}

mat4 Cofactor(const mat4& mat)
{
	mat4 result;
	Cofactor(result.asArray, Minor(mat).asArray, 4, 4);
	return result;
}

float Determinant(const mat4& mat)
{
	float result = 0.0f;
	mat4 cofactor = Cofactor(mat);

	for (int j = 0; j < 4; ++j)
	{
		result += mat.asArray[4 * 0 + j] * cofactor[0][j];
	}

	return result;
}

// Adjugate Matrix
mat2 Adjugate(const mat2& mat)
{
	return Transpose(Cofactor(mat));
}

mat3 Adjugate(const mat3& mat)
{
	return Transpose(Cofactor(mat));
}

mat4 Adjugate(const mat4& mat)
{
	return Transpose(Cofactor(mat));
}

// Matrix Inverse
mat2 Inverse(const mat2& mat)
{
	float det = Determinant(mat);
	if (CMP(det, 0.0f)) { return mat2(); }
	return Adjugate(mat) * (1.0f / det);
}

mat3 Inverse(const mat3& mat)
{
	float det = Determinant(mat);
	if (CMP(det, 0.0f)) { return mat3(); }
	return Adjugate(mat) * (1.0f / det);
}

mat4 Inverse(const mat4& mat)
{
	float det = Determinant(mat);
	if (CMP(det, 0.0f)) { return mat4(); }
	return Adjugate(mat) * (1.0f / det);
}


//		MATRIX TRANSFORMATIONS
// Translation
mat4 Translation(float x, float y, float z)
{
	return mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		x,    y,    z,    1.0f
	);
}

mat4 Translation(const vec3& pos)
{
	{
		return mat4(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			pos.x,pos.y,pos.z,1.0f
		);
	}
}

vec3 GetTranslation(const mat4& mat)
{
	return { mat._41, mat._42, mat._43 };
}

// Scaling
mat4 Scale(float x, float y, float z)
{
	return mat4(
		x,    0.0f, 0.0f, 0.0f,
		0.0f, y,    0.0f, 0.0f,
		0.0f, 0.0f, z   , 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

mat4 Scale(const vec3& vec)
{
	return mat4(
		vec.x, 0.0f, 0.0f, 0.0f,
		0.0f, vec.y, 0.0f, 0.0f,
		0.0f, 0.0f, vec.z, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

vec3 GetScale(const mat4& mat)
{
	return { mat._11, mat._22, mat._33 };
}

// Rotation
mat4 Rotation(float pitch, float yaw, float roll)
{
	return ZRotation(roll) *
		XRotation(pitch) *
		YRotation(yaw);
}

mat3 Rotation3x3(float pitch, float yaw, float roll)
{
	return ZRotation3x3(roll) *
		XRotation3x3(pitch) *
		YRotation3x3(yaw);
}

mat4 ZRotation(float angle)
{
	angle = DEG2RAD(angle);
	return mat4(
		cosf(angle), sinf(angle), 0.0f, 0.0f,
		-sinf(angle), cos(angle), 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

mat3 ZRotation3x3(float angle)
{
	angle = DEG2RAD(angle);
	return mat3(
		cosf(angle), sinf(angle), 0.0f,
		-sinf(angle), cos(angle), 0.0f,
		0.0f, 0.0f, 1.0f
	);
}

mat4 XRotation(float angle)
{
	angle = DEG2RAD(angle);
	return mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, cosf(angle), sinf(angle), 0.0f,
		0.0f, -sinf(angle), cosf(angle), 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
		);
}
mat3 XRotation3x3(float angle)
{
	angle = DEG2RAD(angle);
	return mat3(
		1.0f, 0.0f, 0.0f,
		0.0f, cosf(angle), sinf(angle),
		0.0f, -sinf(angle), cosf(angle)
		);
}

mat4 YRotation(float angle)
{
	angle = DEG2RAD(angle);
	return mat4(
		cosf(angle), 0.0f, -sinf(angle), 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		sinf(angle), 0.0f, cosf(angle), 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

mat3 YRotation3x3(float angle)
{
	angle = DEG2RAD(angle);
	return mat3(
		cosf(angle), 0.0f, -sinf(angle),
		0.0f, 1.0f, 0.0f,
		sinf(angle), 0.0f, cosf(angle)
	);
}

mat4 AxisAngle(const vec3& axis, float angle)
{
	angle = DEG2RAD(angle);
	float c = cosf(angle);
	float s = sinf(angle);
	float t = 1.0f - cosf(angle);

	float x = axis.x;
	float y = axis.y;
	float z = axis.z;

	if (CMP(MagnitudeSq(axis), 1.0f))
	{
		float inv_len = 1.0f / Magnitude(axis);
		x *= inv_len; // Normalize x
		y *= inv_len; // Normalize y
		z *= inv_len; // Normalize z
	}

	return mat4(
		t*(x*x) + c, t*x*y + s*z, t*x*z - s*y, 0.0f,
		t*x*y - s*z, t*(y*y) + c, t*y*z + s*x, 0.0f,
		t*x*z + s*y, t*y*z - s*x, t*(z*z) + c, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

mat3 AxisAngle3x3(const vec3& axis, float angle)
{
	angle = DEG2RAD(angle);
	float c = cosf(angle);
	float s = sinf(angle);
	float t = 1.0f - cosf(angle);

	float x = axis.x;
	float y = axis.y;
	float z = axis.z;

	if (CMP(MagnitudeSq(axis), 1.0f))
	{
		float inv_len = 1.0f / Magnitude(axis);
		x *= inv_len; 
		y *= inv_len; 
		z *= inv_len; 
	}

	return mat3(
		t * (x * x) + c, t * x * y + s * z, t * x * z - s * y,
		t * x * y - s * z, t * (y * y) + c, t * y * z + s * x,
		t * x * z + s * y, t * y * z - s * x, t * (z * z) + c
	);
}