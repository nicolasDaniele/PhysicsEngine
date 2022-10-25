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

bool OBBOBB(const OBB& obb1, const OBB& obb2)
{
	const float* orientation1 = obb1.orientation.asArray;
	const float* orientation2 = obb2.orientation.asArray;

	vec3 test[15] =
	{
		// OBB1 axes
		vec3(orientation1[0], orientation1[1], orientation1[2]),
		vec3(orientation1[3], orientation1[4], orientation1[5]),
		vec3(orientation1[6], orientation1[7], orientation1[8]),
		// OBB2 axes
		vec3(orientation2[0], orientation2[1], orientation2[2]),
		vec3(orientation2[3], orientation2[4], orientation2[5]),
		vec3(orientation2[6], orientation2[7], orientation2[8])
	};

	// Product between OBB1 and OBB2
	for (int i = 0; i < 3; ++i)
	{
		test[6 + i * 3 + 0] = Cross(test[i], test[0]);
		test[6 + i * 3 + 1] = Cross(test[i], test[1]);
		test[6 + i * 3 + 2] = Cross(test[i], test[2]);
	}

	for (int i = 0; i < 15; ++i)
	{
		if (!OverlapOnAxis(obb1, obb2, test[i]))
		{
			return false; // Separating axis found
		}
	}

	return true; // No separating axis found
}

bool OBBPlane(const OBB& obb, const Plane& plane)
{
	const float* orientation = obb.orientation.asArray;
	vec3 rotation[] =
	{
		vec3(orientation[0], orientation[1], orientation[2]),
		vec3(orientation[3], orientation[4], orientation[5]),
		vec3(orientation[6], orientation[7], orientation[8])
	};

	vec3 normal = plane.normal;
	float pLen = obb.size.x * fabsf(Dot(normal, rotation[0])) +
		obb.size.y * fabsf(Dot(normal, rotation[1])) +
		obb.size.z * fabsf(Dot(normal, rotation[2]));

	float dot = Dot(plane.normal, obb.position);
	float distance = dot - plane.distance;

	return fabsf(distance) <= pLen;
}

bool PlanePlane(const Plane& plane1, const Plane& plane2)
{
	vec3 d = Cross(plane1.normal, plane2.normal);
	return CMP(Dot(d, d), 0);
}

// Overlap on axis methods
bool OverlapOnAxis(const AABB& aabb, const OBB& obb, const vec3& axis)
{
	Interval a = GetInterval(aabb, axis);
	Interval b = GetInterval(obb, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}

bool OverlapOnAxis(const OBB& obb1, const OBB& obb2, const vec3& axis)
{
	Interval a = GetInterval(obb1, axis);
	Interval b = GetInterval(obb2, axis);

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

// Raycast methods
float Raycast(const Sphere& sphere, const Ray& ray)
{
	vec3 e = sphere.position - ray.origin;
	float rSq = sphere.radius * sphere.radius;
	float eSq = MagnitudeSq(e);
	float a = Dot(e, ray.direction);

	float bSq = eSq - (a * a);
	float f = sqrt(rSq - bSq);

	if (rSq - (eSq - (a * a)) < 0.0f) // No collision
	{
		return -1;
	}
	else if (eSq < rSq) // Ray starts inside the sphere
	{
		return a + f;
	}
	else // Normal intersection
	{
		return a - f;
	}
}

float Raycast(const AABB& aabb, const Ray& ray)
{
	vec3 min = GetMin(aabb);
	vec3 max = GetMax(aabb);

	// Avoid division for 0!
	Ray testRay = ray;

	if (testRay.direction.x == 0.0f)
	{
		testRay.direction.x = 0.00001f;
	}
	if (testRay.direction.y == 0.0f)
	{
		testRay.direction.y = 0.00001f;
	}
	if (testRay.direction.z == 0.0f)
	{
		testRay.direction.z = 0.00001f;
	}

	float t1 = (min.x - testRay.origin.x) / testRay.direction.x;
	float t2 = (max.x - testRay.origin.x) / testRay.direction.x;
	float t3 = (min.y - testRay.origin.y) / testRay.direction.y;
	float t4 = (max.y - testRay.origin.y) / testRay.direction.y;
	float t5 = (min.z - testRay.origin.z) / testRay.direction.z;
	float t6 = (max.z - testRay.origin.z) / testRay.direction.z;

	float tmin = fmaxf(
		fmaxf(fminf(t1, t2), fminf(t3, t4)), 
		fminf(t5, t6));

	float tmax = fminf(
		fminf(fmaxf(t1, t2), fmaxf(t3, t4)),
		fmaxf(t5, t6));

	if (tmax < 0) // No intersection
	{
		return -1;
	}

	if (tmin > tmax) // No intersection
	{
		return -1;
	}

	if (tmin < 0) // Ray starts inside AABB
	{
		return tmax;
	}

	// Intersection
	return tmin;
}

float Raycast(const OBB& obb, const Ray& ray)
{
	const float* orientation = obb.orientation.asArray;
	const float* size = obb.size.asArray;

	vec3 xAxis(orientation[0], orientation[1], orientation[2]);
	vec3 yAxis(orientation[3], orientation[4], orientation[5]);
	vec3 zAxis(orientation[6], orientation[7], orientation[8]);

	vec3 p = obb.position - ray.origin;

	vec3 f (
		Dot(xAxis, ray.direction),
		Dot(yAxis, ray.direction),
		Dot(zAxis, ray.direction)
		);

	vec3 e (
		Dot(xAxis, p),
		Dot(yAxis, p),
		Dot(zAxis, p)
		);

	float t[6] = { 0, 0, 0, 0, 0, 0 };
	for (int i = 0; i < 3; ++i)
	{
		if (CMP(f[i], 0))
		{
			if (-e[i] - size[i] > 0 || -e[i] + size[i] < 0)
			{
				return -1;
			}

			// Avoid dividing by 0
			f[i] = 0.00001f;
		}

		t[i * 2 + 0] = (e[i] + size[i]) / f[i]; // min
		t[i * 2 + 1] = (e[i] - size[i]) / f[i]; // max
	}

	float tmin = fmaxf(
		fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])),
		fminf(t[4], t[5]));

	float tmax = fminf(
		fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])),
		fmaxf(t[4], t[5]));

	if (tmax < 0) // No intersection
	{
		return -1;
	}

	if (tmin < tmax) // No intersection
	{
		return -1;
	}

	if (tmin < 0.0f) // Ray starts inside OBB
	{
		return tmax;
	}

	return tmin; // Normal intersection
}

float Raycast(const Plane& plane, const Ray& ray)
{
	float nd = Dot(ray.direction, plane.normal);
	float pn = Dot(ray.origin, plane.normal);

	if (nd >= 0.0f)
	{
		return -1; 
	}

	float t = (plane.distance - pn) / nd;
	if (t >= 0.0f)
	{
		return t;
	}

	return -1;
}

// Linecast methods
bool Linecast(const Sphere& sphere, const Line& line)
{
	Point closest = ClosestPoint(line, sphere.position);
	float distSq = MagnitudeSq(sphere.position - closest);
	
	return distSq <= (sphere.radius * sphere.radius);
}

bool Linecast(const AABB& aabb, const Line& line)
{
	Ray ray;
	ray.origin = line.start;
	ray.direction = Normalized(line.end - line.start);
	float t = Raycast(aabb, ray);

	return t >= 0 && t * t <= LenghtSq(line);
}

bool Linecast(const OBB& obb, const Line& line)
{
	Ray ray;
	ray.origin = line.start;
	ray.direction = Normalized(line.end - line.start);
	float t = Raycast(obb, ray);

	return t >= 0 && t * t <= LenghtSq(line);
}

bool Linecast(const Plane& plane, const Line& line)
{
	vec3 ab = line.end - line.start;

	float nA = Dot(plane.normal, line.start);
	float nAB = Dot(plane.normal, ab);

	// Avoid divide by 0
	if (nAB == 0.0f)
	{
		nAB = 0.0001f;
	}
	float t = (plane.distance - nA) / nAB;
	
	return t >= 0.0f && t <= 1.0f;
}

// Triangle methods
bool PointInTriangle(const Point& point, const Triangle& triangle)
{
	vec3 a = triangle.a - point;
	vec3 b = triangle.b - point;
	vec3 c = triangle.c - point;

	vec3 normPBC = Cross(b, c);
	vec3 normPCA = Cross(c, a);
	vec3 normPAB = Cross(a, b);

	if (Dot(normPCA, normPAB) < 0.0f)
	{
		return false;
	}
	else if (Dot(normPBC, normPAB) < 0.0f)
	{
		return false;
	}

	return true;
}

Plane FromTriangle(const Triangle& triangle)
{
	Plane result;
	result.normal = Normalized(Cross(
		triangle.b - triangle.a, triangle.c- triangle. a));
	result.distance = Dot(result.normal, triangle.a);

	return result;
}

Point ClosestPoint(const Triangle& triangle, const Point& point)
{
	Plane plane = FromTriangle(triangle);
	Point closest = ClosestPoint(plane, point);

	if (PointInTriangle(closest, triangle))
	{
		return closest;
	}

	Point c1 = ClosestPoint(Line(triangle.a, triangle.b), point);
	Point c2 = ClosestPoint(Line(triangle.b, triangle.c), point);
	Point c3 = ClosestPoint(Line(triangle.c, triangle.a), point);

	float magSq1 = MagnitudeSq(point - c1);
	float magSq2 = MagnitudeSq(point - c2);
	float magSq3 = MagnitudeSq(point - c3);

	if (magSq1 < magSq2 && magSq1 < magSq3)
	{
		return c1;
	}
	else if (magSq2 < magSq1 && magSq2 < magSq3)
	{
		return c2;
	}

	return c3;
}