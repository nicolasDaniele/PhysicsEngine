#pragma once
#ifndef _H_MATH_VETORS_
#define _H_MATH_VETORS_

#define RAD2DEG(x) ((x) * 57.295754f)
#define DEG2RAD(x) ((x) * 0.0174533f)

//			 STRUCTURE DEFINITIONS
// VEC2
typedef struct vec2
{
	union 
	{
		struct
		{
			float x;
			float y;
		};
		float asArray[2];
	};

	float& operator[](int i)
	{
		return asArray[i];
	}
} vec2;

// VEC3
typedef struct vec3
{
	union 
	{
		struct
		{
			float x;
			float y;
			float z;
		};

		float asArray[3];
	};

	float& operator[](int i)
	{
		return asArray[i];
	}
} vec3;


//			METHOD DECLARATIONS

// Operators Overload
// Addition
vec2 operator+(const vec2& v1, const vec2& v2);
vec3 operator+(const vec3& v1, const vec3& v2);
// Subtraction
vec2 operator-(const vec2& v1, const vec2& v2);
vec3 operator-(const vec3& v1, const vec3& v2);
// Vector Multiplication
vec2 operator*(const vec2& v1, const vec2& v2);
vec3 operator*(const vec3& v1, const vec3& v2);
// Scalar multiplication
vec2 operator*(const vec2& v, float f);
vec3 operator*(const vec3& v, float f);
// Equality 
bool operator==(const vec2& v1, const vec2& v2);
bool operator==(const vec3& v1, const vec3& v);
bool operator!=(const vec2& v1, const vec2& v2);
bool operator!=(const vec3& v1, const vec3& v2);

// Dot Product
float Dot(const vec2& v1, const vec2& v2);
float Dot(const vec3& v1, const vec3& v2);

// Magnitude
float Magnitude(const vec2& v);
float Magnitude(const vec3& v);
// Squared Magnitude
float MagnitudeSq(const vec2& v);
float MagnitudeSq(const vec3& v);

// Vector Normalizing
void Normalize(vec2& v);
void Normalize(vec3& v);
vec2 Normalized(const vec2& v);
vec3 Normalized(const vec3& v);

// Cross Product
vec3 Cross(const vec3 v1, const vec3 v2);

// Angle
float Angle(const vec2& v1, const vec2& v2);
float Angle(const vec3& v1, const vec3& v2);

// Projection
vec2 Project(const vec2& length, const vec2& direction);
vec3 Project(const vec3& length, const vec3& direction);
vec2 Peroendicular(const vec2& length, const vec2& direction);
vec3 Peroendicular(const vec3& length, const vec3& direction);

// Reflection
vec2 Reflection(const vec2& vec, const vec2& normal);
vec3 Reflection(const vec3& vec, const vec3& normal);

#endif
