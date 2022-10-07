#include "Geometry3D.h"
#include <cmath>
#include <cfloat>

#define CMP(x, y) (fabsf((x)-(y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

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

// Point-OBB
bool PointInOBB(const Point& point, const OBB& obb)
{
	vec3 dir = point - obb.position;

	for (int i = 0; i < 3; ++i)
	{
		const float* orientation = &obb.orientation.asArray[i * 3];
		vec3 axis(
			orientation[0],
			orientation[1],
			orientation[2]);

		float distance = Dot(dir, axis);
		if (distance > obb.size.asArray[i])
		{
			return false;
		}
		if (distance < -obb.size.asArray[i])
		{
			return false;
		}
	}

	return true;
}

Point ClosestPoint(const OBB& obb, const Point& point)
{
	Point result = obb.position;
	vec3 dir = point - obb.position;

	for (int i = 0; i < 3; ++i)
	{
		const float* orientation = &obb.orientation.asArray[i * 3];
		vec3 axis(
			orientation[0],
			orientation[1], 
			orientation[2]);

		float distance = Dot(dir, axis);
		if (distance > obb.size.asArray[i])
		{
			distance = obb.size.asArray[i];
		}
		if (distance < -obb.size.asArray[i])
		{
			distance = -obb.size.asArray[i];
		}

		result = result + (axis * distance);
	}

	return result;
}

// Point-Plane
bool PointOnPlane(const Point& point, const Plane& plane)
{
	float dot = Dot(point, plane.normal);
	return CMP(dot - plane.distance, 0.0f);
}

Point ClosestPoint(const Plane& plane, const Point& point)
{
	float dot = Dot(plane.normal, point);
	float distance = dot - plane.distance;
	return point - plane.normal * distance;
}

// Point-Line
bool PointOnLine(const Point& point, const Line& line)
{
	Point closest = ClosestPoint(line, point);
	float distanceSq = MagnitudeSq(closest - point);

	return CMP(distanceSq, 0.0f);
}

Point ClosestPoint(const Line& line, const Point& point)
{
	vec3 lVec = line.end - line.start;
	float t = Dot(point - line.start, lVec) / Dot(lVec, lVec);
	t = fmaxf(t, 0.0f);
	t = fminf(t, 1.0f);

	return line.start + lVec * t;
}

// Point-Ray
bool PointOnRay(const Point& point, const Ray& ray)
{
	if (point == ray.origin)
	{
		return true;
	}

	vec3 norm = point - ray.origin;
	Normalize(norm);
	float diff = Dot(norm, ray.direction);

	return CMP(diff, 1.0); // Both vectors point in the same direction
}

Point ClosestPoint(const Ray& ray, const Point& point)
{
	float t = Dot(point - ray.origin, ray.direction);
	t /= Dot(ray.direction, ray.direction); // Normalize the direction, if needed
	t = fmaxf(t, 0.0f);

	return Point(ray.origin + ray.direction * t);
}


// 3D Shape Intersections
bool SphereShpere(const Sphere& sphere1, const Sphere& sphere2)
{
	float radiiSum = sphere1.radius + sphere2.radius;
	float sqDistance = MagnitudeSq(sphere1.position - sphere2.position);

	return sqDistance < (radiiSum * radiiSum);
}

bool SphereAABB(const Sphere& sphere, const AABB& aabb)
{
	Point closestpoint = ClosestPoint(aabb, sphere.position);
	float sqDistance = MagnitudeSq(sphere.position - closestpoint);
	float sqRadius = sphere.radius * sphere.radius;

	return sqDistance < sqRadius;
}

bool SphereOBB(const Sphere& sphere, const OBB& obb)
{
	Point closestPoint = ClosestPoint(obb, sphere.position);
	float sqDistance = MagnitudeSq(sphere.position - closestPoint);
	float sqRadius = sphere.radius * sphere.radius;

	return sqDistance < sqRadius;
}

bool SpherePlane(const Sphere& sphere, const Plane& plane)
{
	Point closestPoint = ClosestPoint(plane, sphere.position);
	float sqDistance = MagnitudeSq(sphere.position - closestPoint);
	float sqRadius = sphere.radius * sphere.radius;

	return sqDistance < sqRadius;
}

bool AABBAABB(const AABB& aabb1, const AABB& aabb2)
{
	Point aMin = GetMin(aabb1);
	Point aMax = GetMax(aabb1);

	Point bMin = GetMin(aabb2);
	Point bMax = GetMax(aabb2);

	return (aMin.x <= bMax.x && aMax.x >= bMin.x) &&
		   (aMin.y <= bMax.y && aMax.y >= bMin.y) &&
		   (aMin.z <= bMax.z && aMax.z >= bMin.z);
}

bool AABBOBB(const AABB& aabb, const OBB& obb)
{
	const float* orientation = obb.orientation.asArray;

	vec3 test[15] =
	{
		// AABB axes
		vec3(1, 0, 0),
		vec3(0, 1, 0),
		vec3(0, 0, 1),
		// OBB axes
		vec3(orientation[0], orientation[1], orientation[2]),
		vec3(orientation[3], orientation[4], orientation[5]),
		vec3(orientation[6], orientation[7], orientation[8])
	};

	// Product between AABB axes and OBB axes
	for (int i = 0; i < 3; ++i)
	{
		test[6 + i * 3 + 0] = Cross(test[i], test[0]);
		test[6 + i * 3 + 1] = Cross(test[i], test[1]);
		test[6 + i * 3 + 2] = Cross(test[i], test[2]);
	}

	for (int i = 0; i < 15; ++i)
	{
		if (!OverlapOnAxis(aabb, obb, test[i]))
		{
			return false; // Separating axis found
		}
	}

	return true; // No separating axis found
}

bool AABBPlane(const AABB& aabb, const Plane& plane)
{
	float pLen = aabb.size.x* fabsf(plane.normal.x) +
		aabb.size.y * fabsf(plane.normal.y) +
		aabb.size.z * fabsf(plane.normal.z);

	float dot = Dot(plane.normal, aabb.position);
	float distance = dot - plane.distance;

	return fabsf(distance) <= pLen;
}

bool OverlapOnAxis(const AABB& aabb, const OBB& obb, const vec3& axis)
{
	Interval a = GetInterval(aabb, axis);
	Interval b = GetInterval(obb, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}

// Interval methods
Interval GetInterval(const AABB& aabb, const vec3& axis)
{
	vec3 min = GetMin(aabb);
	vec3 max = GetMax(aabb);

	vec3 vertex[8] =
	{
		vec3(min.x, max.y, max.z),
		vec3(min.x, max.y, min.z),
		vec3(min.x, min.y, max.z),
		vec3(min.x, min.y, min.z),
		vec3(max.x, max.y, max.z),
		vec3(max.x, max.y, min.z),
		vec3(max.x, min.y, max.z),
		vec3(max.x, min.y, min.z)
	};

	Interval result;
	result.min = result.max = Dot(axis, vertex[0]);

	for (int i = 0; i < 8; ++i)
	{
		float projection = Dot(axis, vertex[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

Interval GetInterval(const OBB& obb, const vec3& axis)
{
	vec3 vertex[8];
	vec3 C = obb.position; // OBB center
	vec3 E = obb.size; // OBB extents

	const float* O = obb.orientation.asArray; // OBB orientation

	vec3 A[] = // OBB axis
	{
		vec3(O[0], O[1], O[2]),
		vec3(O[3], O[4], O[5]),
		vec3(O[6], O[7], O[8])
	};

	vertex[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	vertex[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	Interval result;
	result.min = result.max = Dot(axis, vertex[0]);

	for (int i = 0; i < 8; ++i)
	{
		float projection = Dot(axis, vertex[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}