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

// Plane method
float PlaneEquation(const Point& point, const Plane& plane)
{
	return Dot(point, plane.normal) - plane.distance;
}


// Point test methods
// Point-Sphere
bool PointInSphere(const Point& point, const Sphere& sphere)
{
	float magSq = MagnitudeSq(point - sphere.position);
	float radSq = sphere.radius * sphere.radius;

	return magSq < radSq;
}

Point ClosestPoint(const Sphere& sphere, const Point& point)
{
	vec3 sphereToPoint = point - sphere.position;
	Normalize(sphereToPoint);
	sphereToPoint = sphereToPoint * sphere.radius;

	return sphereToPoint + sphere.position;
}

// Point-AABB
bool PointInAABB(const Point& point, const AABB& aabb)
{
	Point min = GetMin(aabb);
	Point max = GetMax(aabb);

	if (point.x < min.x || point.y < min.y || point.z < min.z)
	{
		return false;
	}
	
	if (point.x > max.x || point.y > max.y || point.z > max.z)
	{
		return false;
	}

	return true;
}

Point ClosestPoint(const AABB& aabb, const Point& point)
{
	Point result = point;
	Point min = GetMin(aabb);
	Point max = GetMax(aabb);

	result.x = (result.x < min.x) ? min.x : result.x;
	result.y = (result.y < min.y) ? min.y : result.y;
	result.z = (result.z < min.z) ? min.z : result.z;

	result.x = (result.x > max.x) ? max.x : result.x;
	result.y = (result.y > max.y) ? max.y : result.y;
	result.z = (result.z > max.z) ? max.z : result.z;

	return result;
}