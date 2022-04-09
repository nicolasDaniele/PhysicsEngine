#include "vectors.h"
#include <cmath>
#include <cfloat>

#define CMP(x, y) (fabsf((x)-(y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

	//		METHOD IMPLEMENTATION

// Operators Overload
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

// Dot Product
float Dot(const vec2& v1, const vec2& v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

float Dot(const vec3& v1, const vec3& v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

// Magnitude
float Magnitude(const vec2& v) 
{
	return sqrtf(Dot(v, v));
}

float Magnitude(const vec3& v)
{
	return sqrtf(Dot(v, v));
}

// Squared Magnitude
float MagnitudeSq(const vec2& v)
{
	return Dot(v, v);
}

float MagnitudeSq(const vec3& v)
{
	return Dot(v, v);
}

// Vector Normalizing
void Normalize(vec2& v)
{
	v = v * (1.0f / Magnitude(v));
}

void Normalize(vec3& v)
{
	v = v * (1.0f / Magnitude(v));
}

vec2 Normalized(const vec2& v)
{
	return v * (1.0f / Magnitude(v));
}

vec3 Normalized(const vec3& v)
{
	return v * (1.0f / Magnitude(v));
}

// Cross Product
vec3 Cross(const vec3 v1, const vec3 v2)
{
	vec3 result;

	result.x = v1.y * v2.z - v1.z * v2.y;
	result.y = v1.z * v2.x - v1.x * v2.z;
	result.z = v1.x * v2.y - v1.y * v2.x;

	return result;
}