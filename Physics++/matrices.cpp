#include "Matrices.h"
#include <cmath>
#include <cfloat>

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

Mat2 Transpose(const Mat2& matrix)
{
	Mat2 result;
	Transpose(matrix.asArray, result.asArray, 2, 2);
	return result;
}

Mat3 Transpose(const Mat3& matrix)
{
	Mat3 result;
	Transpose(matrix.asArray, result.asArray, 3, 3);
	return result;
}

Mat4 Transpose(const Mat4& matrix)
{
	Mat4 result;
	Transpose(matrix.asArray, result.asArray, 4, 4);
	return result;
}

Mat2 operator* (const Mat2& matrix, float scalar)
{
	Mat2 result;
	for (int i = 0; i < 4; i++)
	{
		result.asArray[i] = matrix.asArray[i] * scalar;
	}
	return result;
}

Mat3 operator* (const Mat3& matrix, float scalar)
{
	Mat3 result;
	for (int i = 0; i < 9; i++)
	{
		result.asArray[i] = matrix.asArray[i] * scalar;
	}
	return result;
}

Mat4 operator* (const Mat4& matrix, float scalar)
{
	Mat4 result;
	for (int i = 0; i < 16; i++)
	{
		result.asArray[i] = matrix.asArray[i] * scalar;
	}
	return result;
}

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

Mat2 operator* (const Mat2& matA, const Mat2& matB)
{
	Mat2 result;
	Multiply(result.asArray, matA.asArray, 
		2, 2, matB.asArray, 2, 2);
	return result;
}

Mat3 operator* (const Mat3& matA, const Mat3& matB)
{
	Mat3 result;
	Multiply(result.asArray, matA.asArray,
		3, 3, matB.asArray, 3, 3);
	return result;
}

Mat4 operator* (const Mat4& matA, const Mat4& matB)
{
	Mat4 result;
	Multiply(result.asArray, matA.asArray,
		4, 4, matB.asArray, 4, 4);
	return result;
}

float Determinant(const Mat2& matrix)
{
	return matrix._11 * matrix._22 -
		matrix._12 * matrix._21;
}

float Determinant(const Mat3& matrix)
{
	float result = 0.0f;
	Mat3 cofactor = Cofactor(matrix);

	for (int j = 0; j < 3; j++)
	{
		int index = 3 * 0 + j;
		result += matrix.asArray[index] * cofactor[0][j];
	}

	return result;
}

Mat2 Cut(const Mat3& mat, int row, int col)
{
	Mat2 result;
	int index = 0;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
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

Mat2 Minor(const Mat2& mat)
{
	return Mat2(mat._22, mat._21,
		mat._12, mat._11);
}

Mat3 Minor(const Mat3& mat)
{
	Mat3 result;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			result[i][j] = Determinant(Cut(mat, i, j));
		}
	}

	return result;
}

void Cofactor(float* out, const float* minor,
	int rows, int cols)
{
	for (int i= 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			int t = cols * j + i;
			int s = cols * j + i;
			float sign = powf(-1.0, i + j);
			out[t] = minor[s] * sign;
		}
	}
}

Mat2 Cofactor(const Mat2& mat)
{
	Mat2 result;
	Cofactor(result.asArray, Minor(mat).asArray, 2, 2);
	return result;
}

Mat3 Cofactor(const Mat3& mat)
{
	Mat3 result;
	Cofactor(result.asArray, Minor(mat).asArray, 3, 3);
	return result;
}

Mat3 Cut(const Mat4& mat, int row, int col)
{
	Mat3 result;
	int index = 0;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
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

Mat4 Minor(const Mat4& mat)
{
	Mat4 result;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			result[i][j] = Determinant(Cut(mat, i, j));
		}
	}

	return result;
}

Mat4 Cofactor(const Mat4& mat)
{
	Mat4 result;
	Cofactor(result.asArray, Minor(mat).asArray, 4, 4);
	return result;
}

float Determinant(const Mat4& mat)
{
	float result = 0.0f;
	Mat4 cofactor = Cofactor(mat);

	for (int i = 0; i < 4; i++)
	{
		result += mat.asArray[4 * 0 + i] * cofactor[0][i];
	}

	return result;
}

Mat2 Adjugate(const Mat2& mat)
{
	return Transpose(Cofactor(mat));
}

Mat3 Adjugate(const Mat3& mat)
{
	return Transpose(Cofactor(mat));
}

Mat4 Adjugate(const Mat4& mat)
{
	return Transpose(Cofactor(mat));
}

Mat2 Inverse(const Mat2& mat)
{
	float det = Determinant(mat);
	if (CMP(det, 0.0f)) 
	{
		return Mat2(); 
	}

	return Adjugate(mat) * (1.0f / det);
}

Mat3 Inverse(const Mat3& mat)
{
	float det = Determinant(mat);
	if (CMP(det, 0.0f)) 
	{
		return Mat3(); 
	}

	return Adjugate(mat) * (1.0f / det);
}

Mat4 Inverse(const Mat4& mat)
{
	float det = Determinant(mat);
	if (CMP(det, 0.0f)) 
	{
		return Mat4(); 
	}

	return Adjugate(mat) * (1.0f / det);
}

Mat4 Translation(float x, float y, float z)
{
	return Mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		x,    y,    z,    1.0f
	);
}

Mat4 Translation(const Vec3& pos)
{
	{
		return Mat4(
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			pos.x,pos.y,pos.z,1.0f
		);
	}
}

Vec3 GetTranslation(const Mat4& mat)
{
	return Vec3(mat._41, mat._42, mat._43);
}

Mat4 Scale(float x, float y, float z)
{
	return Mat4(
		x,    0.0f, 0.0f, 0.0f,
		0.0f, y,    0.0f, 0.0f,
		0.0f, 0.0f, z   , 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

Mat4 Scale(const Vec3& vec)
{
	return Mat4(
		vec.x, 0.0f, 0.0f, 0.0f,
		0.0f, vec.y, 0.0f, 0.0f,
		0.0f, 0.0f, vec.z, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

Vec3 GetScale(const Mat4& mat)
{
	return Vec3(mat._11, mat._22, mat._33);
}

Mat4 Rotation(float pitch, float yaw, float roll)
{
	return ZRotation(roll) *
		XRotation(pitch) *
		YRotation(yaw);
}

Mat3 Rotation3x3(float pitch, float yaw, float roll)
{
	return ZRotation3x3(roll) *
		XRotation3x3(pitch) *
		YRotation3x3(yaw);
}

Mat4 ZRotation(float angle)
{
	angle = DEG2RAD(angle);
	return Mat4(
		cosf(angle), sinf(angle), 0.0f, 0.0f,
		-sinf(angle), cos(angle), 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

Mat3 ZRotation3x3(float angle)
{
	angle = DEG2RAD(angle);
	return Mat3(
		cosf(angle), sinf(angle), 0.0f,
		-sinf(angle), cos(angle), 0.0f,
		0.0f, 0.0f, 1.0f
	);
}

Mat4 XRotation(float angle)
{
	angle = DEG2RAD(angle);
	return Mat4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, cosf(angle), sinf(angle), 0.0f,
		0.0f, -sinf(angle), cosf(angle), 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
		);
}
Mat3 XRotation3x3(float angle)
{
	angle = DEG2RAD(angle);
	return Mat3(
		1.0f, 0.0f, 0.0f,
		0.0f, cosf(angle), sinf(angle),
		0.0f, -sinf(angle), cosf(angle)
		);
}

Mat4 YRotation(float angle)
{
	angle = DEG2RAD(angle);
	return Mat4(
		cosf(angle), 0.0f, -sinf(angle), 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		sinf(angle), 0.0f, cosf(angle), 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

Mat3 YRotation3x3(float angle)
{
	angle = DEG2RAD(angle);
	return Mat3(
		cosf(angle), 0.0f, -sinf(angle),
		0.0f, 1.0f, 0.0f,
		sinf(angle), 0.0f, cosf(angle)
	);
}

Mat4 AxisAngle(const Vec3& axis, float angle)
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

	return Mat4(
		t*(x*x) + c, t*x*y + s*z, t*x*z - s*y, 0.0f,
		t*x*y - s*z, t*(y*y) + c, t*y*z + s*x, 0.0f,
		t*x*z + s*y, t*y*z - s*x, t*(z*z) + c, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

Mat3 AxisAngle3x3(const Vec3& axis, float angle)
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

	return Mat3(
		t * (x * x) + c, t * x * y + s * z, t * x * z - s * y,
		t * x * y - s * z, t * (y * y) + c, t * y * z + s * x,
		t * x * z + s * y, t * y * z - s * x, t * (z * z) + c
	);
}

Vec3 MultiplyPoint(const Vec3& vec, const Mat4& mat)
{
	Vec3 result;
	result.x = vec.x * mat._11 + vec.y * mat._21 + 
		       vec.z * mat._31 + 1.0f * mat._41;
	result.y = vec.x * mat._12 + vec.y * mat._22 +
               vec.z * mat._32 + 1.0f * mat._42;
	result.z = vec.x * mat._13 + vec.y * mat._23 +
               vec.z * mat._33 + 1.0f * mat._43;

	return result;
}

Vec3 MultiplyVector(const Vec3& vec, const Mat4& mat)
{
	Vec3 result;
	result.x = vec.x * mat._11 + vec.y * mat._21 +
		vec.z * mat._31 + 0.0f * mat._41;
	result.y = vec.x * mat._12 + vec.y * mat._22 +
		vec.z * mat._32 + 0.0f * mat._42;
	result.z = vec.x * mat._13 + vec.y * mat._23 +
		vec.z * mat._33 + 0.0f * mat._43;

	return result;
}

Vec3 MultiplyVector(const Vec3& vec, const Mat3& mat)
{
	Vec3 result;
	result.x = Dot(vec, Vec3(mat._11, mat._21, mat._31));
	result.y = Dot(vec, Vec3(mat._12, mat._22, mat._32));
	result.z = Dot(vec, Vec3(mat._13, mat._23, mat._33));

	return result;
}

Mat4 Transform(const Vec3& scale, const Vec3& eulerRotation,
	const Vec3& translate)
{
	return Scale(scale) *
		Rotation(eulerRotation.x, 
			     eulerRotation.y,
			     eulerRotation.z) *
		Translation(translate);
}

Mat4 Transform(const Vec3& scale, const Vec3& rotationAxis,
	float rotationAngle, const Vec3& translate)
{
	return Scale(scale) *
		AxisAngle(rotationAxis, rotationAngle) *
		Translation(translate);
}

Mat4 LookAt(const Vec3& position, const Vec3& target, const Vec3& up)
{
	Vec3 forward = Normalized(target - position);
	Vec3 right = Normalized(Cross(up, forward));
	Vec3 newUp = Cross(forward, right);

	return Mat4(
		right.x, newUp.x, forward.x, 0.0f,
		right.y, newUp.y, forward.y, 0.0f,
		right.z, newUp.z, forward.z, 0.0f,
		-Dot(right, position),
		-Dot(newUp, position),
		-Dot(forward, position), 1.0f
	);
}

Mat4 Projection(float fov, float aspect,
	float zNear, float zFar)
{
	float tanHalfFov = tanf(DEG2RAD((fov * 0.5f)));
	float fovY = 1.0f / tanHalfFov;
	float fovX = fovY / aspect;
	
	Mat4 result;
	result._11 = fovX;
	result._22 = fovY;
	result._33 = zFar / (zFar - zNear);
	result._34 = 1.0f;
	result._43 = -zNear * result._33;
	result._44 = 0.0f;

	return result;
}

Mat4 Ortho(float left, float right, float bottom,
	float top, float zNear, float zFar)
{
	float _11 = 2.0f / (right - left);
	float _22 = 2.0f / (top - bottom);
	float _33 = 1.0f / (zFar - zNear);
	float _41 = (left + right) / (left - right);
	float _42 = (top + bottom) / (top - bottom);
	float _43 = (zNear) / (zNear - zFar);

	return Mat4(
		_11, 0.0f, 0.0f, 0.0f,
		0.0f, _22, 0.0f, 0.0f,
		0.0f, 0.0f, _33, 0.0f,
		_41, _42, _43, 1.0f
	);
}