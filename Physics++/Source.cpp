#include "vectors.h"
#include <cmath>
#include <cfloat>

#define CMP(x, y) (fabsf((x)-(y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

	//		METHOD IMPLEMENTATION

// Addition
vec2 operator+(const vec2& v1, const vec2& v2)
{
	return { v1.x + v2.x, v1.y + v2.y };
}

vec3 operator+(const vec3& v1, const vec3& v2)
{
	return { v1.x + v2.x, v1.y + v2.y, v1.z+ v2.z };
}

// Substraction
vec2 operator-(const vec2& v1, const vec2& v2)
{
	return { v1.x - v2.x, v1.y - v2.y };
}

vec3 operator-(const vec3& v1, const vec3& v2)
{
	return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

// Vector Multiplication
vec2 operator*(const vec2& v1, const vec2& v2)
{
	return { v1.x * v2.x, v1.y * v2.y };
}

vec3 operator*(const vec3& v1, const vec3& v2)
{
	return { v1.x * v2.x, v1.y * v2.y, v1.z * v2.z };
}

// Scalar Multiplication
vec2 operator*(const vec2& v, float f)
{
	return { v.x * f, v.y * f };
}

vec3 operator*(const vec3& v, float f)
{
	return { v.x * f, v.y * f, v.z * f };
}

// Equality
bool operator==(const vec2& v1, const vec2& v2)
{
	return CMP(v1.x, v2.x) && CMP(v1.y, v2.y);
}

bool operator==(const vec3& v1, const vec3& v2)
{
	return CMP(v1.x, v2.x) && CMP(v1.y, v2.y) && CMP(v1.z, v2.z);
}

bool operator!=(const vec2& v1, const vec2& v2)
{
	return !(v1 == v2);
}

bool operator!=(const vec3& v1, const vec3& v2)
{
	return !(v1 == v2);
}