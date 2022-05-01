#include "vectors.h"
#include <cmath>
#include <cfloat>

#define CMP(x, y) (fabsf((x)-(y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

	//		METHOD IMPLEMENTATIONS

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

// Angle
float Angle(const vec2& v1, const vec2& v2)
{
	float m = sqrtf(MagnitudeSq(v1) * MagnitudeSq(v2));
	return acos(Dot(v1, v2) / m);
}

float Angle(const vec3& v1, const vec3& v2)
{
	float m = sqrtf(MagnitudeSq(v1) * MagnitudeSq(v2));
	return acos(Dot(v1, v2) / m);
}

// Projection
vec2 Project(const vec2& length, const vec2& direction)
{
	float dot = Dot(length, direction);
	float magSq = MagnitudeSq(direction);
	return direction * (dot / magSq);
}

vec3 Project(const vec3& length, const vec3& direction)
{
	float dot = Dot(length, direction);
	float magSq = MagnitudeSq(direction);
	return direction * (dot / magSq);
}

vec2 Peroendicular(const vec2& length, const vec2& direction)
{
	return length - Project(length, direction);
}

vec3 Peroendicular(const vec3& length, const vec3& direction)
{
	return length - Project(length, direction);
}

// Reflection
vec2 Reflection(const vec2& vec, const vec2& normal)
{
	float dot = Dot(vec, normal);
	return vec - normal * (dot * 2.0f); 
}

vec3 Reflection(const vec3& vec, const vec3& normal)
{
	float dot = Dot(vec, normal);
	return vec - normal * (dot * 2.0f);
}