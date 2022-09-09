#include "Geometry3D.h"
#include <cmath>
#include <cfloat>

// Line methods
float Lenght(const Line& line)
{
	return Magnitude(line.start - line.end);
}

float LenghtSq(const Line& line)
{
	return MagnitudeSq(line.start - line.end);
}

// Ray method
Ray FromPoints(const Point& from, const Point& to)
{
	return Ray(from, Normalized(to - from));
}

// AABB methods
vec3 GetMin(const AABB& aabb)
{
	vec3 p1 = aabb.position + aabb.size;
	vec3 p2 = aabb.position - aabb.size;

	return vec3(fminf(p1.x, p2.x),
		fminf(p1.y, p2.y), 
		fminf(p1.z, p2.z));
}

vec3 GetMax(const AABB& aabb)
{
	vec3 p1 = aabb.position + aabb.size;
	vec3 p2 = aabb.position - aabb.size;

	return vec3(fmaxf(p1.x, p2.x),
		fmaxf(p1.y, p2.y),
		fmaxf(p1.z, p2.z));
}

AABB FormMinMax(const vec3& min, const vec3& max)
{
	return AABB((min + max) * 0.5f, (min - max) * 0.5f);
}