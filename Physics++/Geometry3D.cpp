#include "Geometry3D.h"
#include <cmath>
#include <cfloat>
#include <list>

void Model::SetContent(Mesh* mesh)
{
	content = mesh;

	if (content != 0)
	{
		Vec3 min = mesh->vertices[0];
		Vec3 max = mesh->vertices[0];

		for (int i = 1; i < mesh->numTriangles * 3; ++i)
		{
			min.x = fminf(mesh->vertices[i].x, min.x);
			min.y = fminf(mesh->vertices[i].y, min.y);
			min.z = fminf(mesh->vertices[i].z, min.z);

			max.x = fmaxf(mesh->vertices[i].x, max.x);
			max.y = fmaxf(mesh->vertices[i].y, max.y);
			max.z = fmaxf(mesh->vertices[i].z, max.z);
		}

		bounds = FromMinMax(min, max);
	}
}

float Lenght(const Line& line)
{
	return Magnitude(line.start - line.end);
}

float LenghtSq(const Line& line)
{
	return MagnitudeSq(line.start - line.end);
}

Ray FromPoints(const Point& from, const Point& to)
{
	return Ray(from, Normalized(to - from));
}

Vec3 GetMin(const AABB& aabb)
{
	Vec3 p1 = aabb.position + aabb.size;
	Vec3 p2 = aabb.position - aabb.size;

	return Vec3(fminf(p1.x, p2.x),
		fminf(p1.y, p2.y), 
		fminf(p1.z, p2.z));
}

Vec3 GetMax(const AABB& aabb)
{
	Vec3 p1 = aabb.position + aabb.size;
	Vec3 p2 = aabb.position - aabb.size;

	return Vec3(fmaxf(p1.x, p2.x),
		fmaxf(p1.y, p2.y),
		fmaxf(p1.z, p2.z));
}

AABB FromMinMax(const Vec3& min, const Vec3& max)
{
	return AABB((min + max) * 0.5f, (min - max) * 0.5f);
}

float PlaneEquation(const Point& point, const Plane& plane)
{
	return Dot(point, plane.normal) - plane.distance;
}

bool PointInSphere(const Point& point, const Sphere& sphere)
{
	float magSq = MagnitudeSq(point - sphere.position);
	float radSq = sphere.radius * sphere.radius;

	return magSq < radSq;
}

Point ClosestPoint(const Sphere& sphere, const Point& point)
{
	Vec3 sphereToPoint = point - sphere.position;
	Normalize(sphereToPoint);
	sphereToPoint = sphereToPoint * sphere.radius;

	return sphereToPoint + sphere.position;
}

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

bool PointInOBB(const Point& point, const OBB& obb)
{
	Vec3 dir = point - obb.position;

	for (int i = 0; i < 3; ++i)
	{
		const float* orientation = &obb.orientation.asArray[i * 3];
		Vec3 axis(
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
	Vec3 dir = point - obb.position;

	for (int i = 0; i < 3; ++i)
	{
		const float* orientation = &obb.orientation.asArray[i * 3];
		Vec3 axis(
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

bool PointOnLine(const Point& point, const Line& line)
{
	Point closest = ClosestPoint(line, point);
	float distanceSq = MagnitudeSq(closest - point);

	return CMP(distanceSq, 0.0f);
}

Point ClosestPoint(const Line& line, const Point& point)
{
	Vec3 lVec = line.end - line.start;
	float t = Dot(point - line.start, lVec) / Dot(lVec, lVec);
	t = fmaxf(t, 0.0f);
	t = fminf(t, 1.0f);

	return line.start + lVec * t;
}

bool PointOnRay(const Point& point, const Ray& ray)
{
	if (point == ray.origin)
	{
		return true;
	}

	Vec3 norm = point - ray.origin;
	Normalize(norm);
	float diff = Dot(norm, ray.direction);

	return CMP(diff, 1.0f);
}

Point ClosestPoint(const Ray& ray, const Point& point)
{
	float t = Dot(point - ray.origin, ray.direction);
	t /= Dot(ray.direction, ray.direction);
	t = fmaxf(t, 0.0f);

	return Point(ray.origin + ray.direction * t);
}

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

	Vec3 test[15] =
	{
		Vec3(1, 0, 0),
		Vec3(0, 1, 0),
		Vec3(0, 0, 1),
		Vec3(orientation[0], orientation[1], orientation[2]),
		Vec3(orientation[3], orientation[4], orientation[5]),
		Vec3(orientation[6], orientation[7], orientation[8])
	};

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
			return false;
		}
	}

	return true;
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

	Vec3 test[15] =
	{
		Vec3(orientation1[0], orientation1[1], orientation1[2]),
		Vec3(orientation1[3], orientation1[4], orientation1[5]),
		Vec3(orientation1[6], orientation1[7], orientation1[8]),
		Vec3(orientation2[0], orientation2[1], orientation2[2]),
		Vec3(orientation2[3], orientation2[4], orientation2[5]),
		Vec3(orientation2[6], orientation2[7], orientation2[8])
	};

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
			return false;
		}
	}

	return true;
}

bool OBBPlane(const OBB& obb, const Plane& plane)
{
	const float* orientation = obb.orientation.asArray;
	Vec3 rotation[] =
	{
		Vec3(orientation[0], orientation[1], orientation[2]),
		Vec3(orientation[3], orientation[4], orientation[5]),
		Vec3(orientation[6], orientation[7], orientation[8])
	};

	Vec3 normal = plane.normal;
	float pLen = obb.size.x * fabsf(Dot(normal, rotation[0])) +
		obb.size.y * fabsf(Dot(normal, rotation[1])) +
		obb.size.z * fabsf(Dot(normal, rotation[2]));

	float dot = Dot(plane.normal, obb.position);
	float distance = dot - plane.distance;

	return fabsf(distance) <= pLen;
}

bool PlanePlane(const Plane& plane1, const Plane& plane2)
{
	Vec3 d = Cross(plane1.normal, plane2.normal);
	return CMP(Dot(d, d), 0);
}

bool OverlapOnAxis(const AABB& aabb, const OBB& obb, const Vec3& axis)
{
	Interval a = GetInterval(aabb, axis);
	Interval b = GetInterval(obb, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}

bool OverlapOnAxis(const OBB& obb1, const OBB& obb2, const Vec3& axis)
{
	Interval a = GetInterval(obb1, axis);
	Interval b = GetInterval(obb2, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}

Interval GetInterval(const AABB& aabb, const Vec3& axis)
{
	Vec3 min = GetMin(aabb);
	Vec3 max = GetMax(aabb);

	Vec3 vertex[8] =
	{
		Vec3(min.x, max.y, max.z),
		Vec3(min.x, max.y, min.z),
		Vec3(min.x, min.y, max.z),
		Vec3(min.x, min.y, min.z),
		Vec3(max.x, max.y, max.z),
		Vec3(max.x, max.y, min.z),
		Vec3(max.x, min.y, max.z),
		Vec3(max.x, min.y, min.z)
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

Interval GetInterval(const OBB& obb, const Vec3& axis)
{
	Vec3 vertex[8];
	Vec3 C = obb.position;
	Vec3 E = obb.size;

	const float* O = obb.orientation.asArray;

	Vec3 A[] =
	{
		Vec3(O[0], O[1], O[2]),
		Vec3(O[3], O[4], O[5]),
		Vec3(O[6], O[7], O[8])
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

bool Raycast(const Sphere& sphere, const Ray& ray, RaycastResult* outResult)
{
	ResetRaycastResult(outResult);

	Vec3 e = sphere.position - ray.origin;
	float rSq = sphere.radius * sphere.radius;
	float eSq = MagnitudeSq(e);
	float a = Dot(e, ray.direction);

	float bSq = eSq - (a * a);
	float f = sqrt(rSq - bSq);
	float t = a - f;

	if (rSq - (eSq - (a * a)) < 0.0f)
	{
		return false;
	}
	else if (eSq < rSq)
	{
		t = a + f;
	}
	
	if (outResult != 0)
	{
		outResult->t = t;
		outResult->hit = true;
		outResult->point = ray.origin + ray.direction * t;
		outResult->normal = Normalized(outResult->point - sphere.position);
	}

	return true;
}

bool Raycast(const AABB& aabb, const Ray& ray, RaycastResult* outResult)
{
	ResetRaycastResult(outResult);

	Vec3 min = GetMin(aabb);
	Vec3 max = GetMax(aabb);

	float t[] = { 0, 0, 0, 0, 0, 0 };

	if (!CMP(ray.direction.x, 0)) t[0] = (min.x - ray.origin.x) / ray.direction.x;
	if (!CMP(ray.direction.x, 0)) t[1] = (max.x - ray.origin.x) / ray.direction.x;
	if (!CMP(ray.direction.y, 0)) t[2] = (min.y - ray.origin.y) / ray.direction.y;
	if (!CMP(ray.direction.y, 0)) t[3] = (max.y - ray.origin.y) / ray.direction.y;
	if (!CMP(ray.direction.z, 0)) t[4] = (min.z - ray.origin.z) / ray.direction.z;
	if (!CMP(ray.direction.z, 0)) t[5] = (max.z - ray.origin.z) / ray.direction.z;

	float tmin = fmaxf(
		fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])), 
		fminf(t[4], t[5]));

	float tmax = fminf(
	fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])),
		fmaxf(t[4], t[5]));

	if (tmax < 0) return false;
	if (tmin > tmax) return false;

	float t_result = tmin;
	if (tmin < 0) t_result = tmax;

	if (outResult != 0)
	{
		outResult->t = t_result;
		outResult->hit = true;
		outResult->point = ray.origin + ray.direction * t_result;

		Vec3 normals[] = 
		{
			Vec3(-1, 0, 0), Vec3(1, 0, 0),
			Vec3(0, -1, 0), Vec3(0, 1, 0),
			Vec3(0, 0, -1), Vec3(0, 0, 1)
		};

		for (int i = 0; i < 6; ++i)
		{
			if (CMP(t_result, t[i]))
			{
				outResult->normal = normals[i];
			}
		}
	}

	return true;
}

bool Raycast(const OBB& obb, const Ray& ray, RaycastResult* outResult)
{
	ResetRaycastResult(outResult);

	const float* orientation = obb.orientation.asArray;
	const float* size = obb.size.asArray;
	Vec3 p = obb.position - ray.origin;

	Vec3 xAxis(orientation[0], orientation[1], orientation[2]);
	Vec3 yAxis(orientation[3], orientation[4], orientation[5]);
	Vec3 zAxis(orientation[6], orientation[7], orientation[8]);	

	Vec3 f (
		Dot(xAxis, ray.direction),
		Dot(yAxis, ray.direction),
		Dot(zAxis, ray.direction)
		);

	Vec3 e (
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
				return false;
			}

			f[i] = 0.00001f;
		}

		t[i * 2 + 0] = (e[i] + size[i]) / f[i];
		t[i * 2 + 1] = (e[i] - size[i]) / f[i];
	}

	float tmin = fmaxf(
		fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])),
		fminf(t[4], t[5]));

	float tmax = fminf(
		fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])),
		fmaxf(t[4], t[5]));

	if (tmax < 0) return false;
	if (tmin > tmax) return false;

	float t_result = tmin;
	if (tmin < 0.0f) return tmax;

	if (outResult != 0)
	{
		outResult->hit = true;
		outResult->t = t_result;
		outResult->point = ray.origin + ray.direction * t_result;

		Vec3 normals[] =
		{
			xAxis, xAxis * -1.0f,
			yAxis, yAxis * -1.0f,
			zAxis, zAxis * -1.0f
		};

		for (int i = 0; i < 6; ++i)
		{
			if (CMP(t_result, t[i]))
			{
				outResult->normal = Normalized(normals[i]);
			}
		}
	}

	return true;
}

bool Raycast(const Plane& plane, const Ray& ray, RaycastResult* outResult)
{
	ResetRaycastResult(outResult);

	float nd = Dot(ray.direction, plane.normal);
	float pn = Dot(ray.origin, plane.normal);

	if (nd >= 0.0f)
	{
		return false; 
	}

	float t = (plane.distance - pn) / nd;
	if (t >= 0.0f)
	{
		if (outResult != 0)
		{
			outResult->t = t;
			outResult->hit = true;
			outResult->point = ray.origin + ray.direction * t;
			outResult->normal = Normalized(plane.normal);
		}

		return true;
	}

	return false;
}

bool Raycast(const Triangle& triangle, const Ray& ray, RaycastResult* outResult)
{
	ResetRaycastResult(outResult);

	Plane plane = FromTriangle(triangle);
	RaycastResult planeResult;

	if (!Raycast(plane, ray, &planeResult))
	{
		return false;
	}

	float t = planeResult.t;
	Point resultPoint = ray.origin + ray.direction * t;
	Vec3 barycentric = Barycentric(resultPoint, triangle);

	if (barycentric.x >= 0.0f && barycentric.x <= 1.0f &&
		barycentric.y >= 0.0f && barycentric.y <= 1.0f &&
		barycentric.z >= 0.0f && barycentric.z <= 1.0f)
	{
		if (outResult != 0)
		{
			outResult->t = t;
			outResult->hit = true;
			outResult->point = ray.origin + ray.direction * t;
			outResult->normal = plane.normal;
		}

		return true;
	}

	return false;
}

void ResetRaycastResult(RaycastResult* outResult)
{
	if (outResult != 0)
	{
		outResult->t = -1;
		outResult->hit = false;
		outResult->normal = Vec3(0, 0, 1);
		outResult->point = Vec3(0, 0, 0);
	}
}

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
	
	RaycastResult result;
	if (!Raycast(aabb, ray, &result))
	{
		return false;
	}

	float t = result.t;

	return t >= 0 && t * t <= LenghtSq(line);
}

bool Linecast(const OBB& obb, const Line& line)
{
	Ray ray;
	ray.origin = line.start;
	ray.direction = Normalized(line.end - line.start);

	RaycastResult result;
	if (!Raycast(obb, ray, &result))
	{
		return false;
	}

	float t = result.t;

	return t >= 0 && t * t <= LenghtSq(line);
}

bool Linecast(const Plane& plane, const Line& line)
{
	Vec3 ab = line.end - line.start;

	float nA = Dot(plane.normal, line.start);
	float nAB = Dot(plane.normal, ab);

	if (nAB == 0.0f)
	{
		nAB = 0.0001f;
	}
	float t = (plane.distance - nA) / nAB;
	
	return t >= 0.0f && t <= 1.0f;
}

bool Linecast(const Triangle& triangle, const Line& line)
{
	Ray ray;
	ray.origin = line.start;
	ray.origin = Normalized(line.end - line.start);
	RaycastResult result;
	
	if (!Raycast(triangle, ray, &result))
	{
		return false;
	}

	float t = result.t;

	return t >= 0.0f && t * t <= LenghtSq(line);
}

bool PointInTriangle(const Point& point, const Triangle& triangle)
{
	Vec3 a = triangle.a - point;
	Vec3 b = triangle.b - point;
	Vec3 c = triangle.c - point;

	Vec3 normPBC = Cross(b, c);
	Vec3 normPCA = Cross(c, a);
	Vec3 normPAB = Cross(a, b);

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

Interval GetInterval(const Triangle& triangle, const Vec3& axis)
{
	Interval result;
	result.min = Dot(axis, triangle.points[0]);
	result.max = result.min;

	for (int i = 0; i < 3; ++i)
	{
		float value = Dot(axis, triangle.points[i]);
		result.min = fminf(result.min, value);
		result.max = fmaxf(result.max, value);
	}

	return result;
}

bool OverlapOnAxis(const AABB& aabb, const Triangle& triangle, const Vec3& axis)
{
	Interval a = GetInterval(aabb, axis);
	Interval b = GetInterval(triangle, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}
/*
bool OverlapOnAxis(const OBB& obb, const Triangle& triangle, const Vec3& axis)
{
	Interval a = GetInterval(obb, axis);
	Interval b = GetInterval(triangle, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}*/

bool OverlapOnAxis(const Triangle& triangle1, const Triangle& triangle2, const Vec3& axis)
{
	Interval a = GetInterval(triangle1, axis);
	Interval b = GetInterval(triangle2, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}

Vec3 SATCrossEdge(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
{
	Vec3 ab = a - b;
	Vec3 cd = c - d;
	Vec3 result = Cross(ab, cd);

	if (!CMP(MagnitudeSq(result), 0))
	{
		return result;
	}
	else
	{
		Vec3 axis = Cross(ab, c - a);
		result = Cross(ab, axis);

		if (!CMP(MagnitudeSq(result), 0))
		{
			return result;
		}
	}

	return Vec3();
}

Vec3 Barycentric(const Point& point, const Triangle& triangle)
{
	Vec3 ap = point - triangle.a;
	Vec3 bp = point - triangle.b;
	Vec3 cp = point - triangle.c;

	Vec3 ab = triangle.b - triangle.a;
	Vec3 ac = triangle.c - triangle.a;

	Vec3 bc = triangle.c - triangle.b;
	Vec3 cb = triangle.b - triangle.c;
	Vec3 ca = triangle.a - triangle.c;

	Vec3 v = ab - Project(ab, cb);
	float a = 1.0f - (Dot(v, ap) / Dot(v, ab));

	v = bc - Project(bc, ac);
	float b = 1.0f - (Dot(v, bp) / Dot(v, bc));

	v = ca - Project(ca, ab);
	float c = 1.0f - (Dot(v, cp) / Dot(v, ca));

	return Vec3(a, b, c);
}

bool TriangleSphere(const Triangle& triangle, const Sphere& sphere)
{
	Point closest = ClosestPoint(triangle, sphere.position);
	float magSq = MagnitudeSq(closest - sphere.position);

	return magSq <= (sphere.radius * sphere.radius);
}

bool TriangleAABB(const Triangle& triangle, const AABB& aabb)
{
	Vec3 f0 = triangle.b - triangle.a;
	Vec3 f1 = triangle.c - triangle.b;
	Vec3 f2 = triangle.a - triangle.c;

	Vec3 u0(1.0f, 0.0f, 0.0f);
	Vec3 u1(0.0f, 1.0f, 0.0f);
	Vec3 u2(0.0f, 0.0f, 1.0f);

	Vec3 test[13] =
	{
		u0, u1, u2,
		Cross(f0, f1),
		Cross(u0, f0), Cross(u0, f1), Cross(u0, f2),
		Cross(u1, f0), Cross(u1, f1), Cross(u1, f2),
		Cross(u2, f0), Cross(u2, f1), Cross(u2, f2)
	};

	for (int i = 0; i < 13; ++i)
	{
		if (!OverlapOnAxis(aabb, triangle, test[i]))
		{
			return false;
		}
	}

	return true;
}

bool TriangleOBB(const Triangle& triangle, const OBB& obb)
{
	Vec3 f0 = triangle.b - triangle.a;
	Vec3 f1 = triangle.c - triangle.b;
	Vec3 f2 = triangle.a - triangle.c;

	const float* orientation = obb.orientation.asArray;
	Vec3 u0(orientation[0], orientation[1], orientation[2]);
	Vec3 u1(orientation[3], orientation[4], orientation[5]);
	Vec3 u2(orientation[6], orientation[7], orientation[8]);

	Vec3 test[13] =
	{
		u0, u1, u2, 
		Cross(f0, f1),
		Cross(u0, f0), Cross(u0, f1), Cross(u0, f2),
		Cross(u1, f0), Cross(u1, f1), Cross(u1, f2),
		Cross(u2, f0), Cross(u2, f1), Cross(u2, f2)
	};

	for (int i = 0; i < 13; ++i)
	{
		if (!OverlapOnAxis(obb, triangle, test[i]))
		{
			return false;
		}
	}

	return true;
}

bool OverlapOnAxis(const OBB& obb, const Triangle& triangle, const Vec3& axis)
{
	Vec3 f0 = triangle.b - triangle.a;
	Vec3 f1 = triangle.c - triangle.b;
	Vec3 f2 = triangle.a - triangle.c;

	const float* orientation = obb.orientation.asArray;
	Vec3 u0(orientation[0], orientation[1], orientation[2]);
	Vec3 u1(orientation[3], orientation[4], orientation[5]);
	Vec3 u2(orientation[6], orientation[7], orientation[8]);

	Vec3 test[13] =
	{
		u0, u1, u2,
		Cross(f0, f1),
		Cross(u0, f0), Cross(u0, f1), Cross(u0, f2),
		Cross(u1, f0), Cross(u1, f1), Cross(u1, f2),
		Cross(u2, f0), Cross(u2, f1), Cross(u2, f2)
	};

	for (int i = 0; i < 13; ++i)
	{
		if (!OverlapOnAxis(obb, triangle, test[i]))
		{
			return false;
		}
	}

	return true;
}

bool TrianglePlane(const Triangle& triangle, const Plane& plane)
{
	float side1 = PlaneEquation(triangle.a, plane);
	float side2 = PlaneEquation(triangle.c, plane);
	float side3 = PlaneEquation(triangle.a, plane);

	if (CMP(side1, 0) && CMP(side2, 0) && CMP(side3, 0))
	{
		return true;
	}

	if (side1 > 0 && side2 > 0 && side3 > 0)
	{
		return false;
	}

	if (side1 < 0 && side2 < 0 && side3 < 0)
	{
		return false;
	}

	return true;
}

bool TriangleTriangle(const Triangle& triangle1, const Triangle& triangle2)
{
	Vec3 t1_f0 = triangle1.b - triangle1.a;
	Vec3 t1_f1 = triangle1.c - triangle1.b;
	Vec3 t1_f2 = triangle1.a - triangle1.c;

	Vec3 t2_f0 = triangle2.b - triangle2.a;
	Vec3 t2_f1 = triangle2.c - triangle2.b;
	Vec3 t2_f2 = triangle2.a - triangle2.c;

	Vec3 axisToTest[] =
	{
		Cross(t1_f0,t1_f1),
		Cross(t2_f0,t2_f1),
		Cross(t2_f0,t1_f0), Cross(t2_f0,t1_f1), Cross(t2_f0,t1_f2),
		Cross(t2_f1,t1_f0), Cross(t2_f1,t1_f1), Cross(t2_f1,t1_f2),
		Cross(t2_f2,t1_f0), Cross(t2_f2,t1_f1), Cross(t2_f2,t1_f2)
	};

	for (int i = 0; i < 11; ++i)
	{
		if (!OverlapOnAxis(triangle1, triangle2, axisToTest[i]))
		{
			return false;
		}
	}

	return true;
}

bool TriangleTriangleRobust(const Triangle& triangle1, const Triangle& triangle2)
{
	Vec3 axisToTest[] =
	{
		SATCrossEdge(triangle1.a, triangle1.b, triangle1.b, triangle1.c),
		SATCrossEdge(triangle2.a, triangle2.b, triangle2.b, triangle2.c),

		SATCrossEdge(triangle2.a, triangle2.b, triangle1.a, triangle1.b),
		SATCrossEdge(triangle2.a, triangle2.b, triangle1.b, triangle1.c),
		SATCrossEdge(triangle2.a, triangle2.b, triangle1.c, triangle1.a),

		SATCrossEdge(triangle2.b, triangle2.c, triangle1.a, triangle1.b),
		SATCrossEdge(triangle2.b, triangle2.c, triangle1.b, triangle1.c),
		SATCrossEdge(triangle2.b, triangle2.c, triangle1.c, triangle1.a),

		SATCrossEdge(triangle2.c, triangle2.a, triangle1.a, triangle1.b),
		SATCrossEdge(triangle2.c, triangle2.a, triangle1.b, triangle1.c),
		SATCrossEdge(triangle2.c, triangle2.a, triangle1.c, triangle1.a)
	};

	for (int i = 0; i < 11; ++i)
	{
		if (!OverlapOnAxis(triangle1, triangle2, axisToTest[i]))
		{
			if (!CMP(MagnitudeSq(axisToTest[i]), 0))
			{
				return false;
			}
		}
	}

	return true;
}

void AccelarateMesh(Mesh& mesh)
{
	if (mesh.accelerator != 0)
	{
		return;
	}

	Vec3 min = mesh.vertices[0];
	Vec3 max = mesh.vertices[0];
	for (int i = 0; i < mesh.numTriangles * 3; ++i)
	{
		min.x = fminf(mesh.vertices[i].x, min.x);
		min.y = fminf(mesh.vertices[i].y, min.y);
		min.z = fminf(mesh.vertices[i].z, min.z);
		max.x = fmaxf(mesh.vertices[i].x, max.x);
		max.y = fmaxf(mesh.vertices[i].y, max.y);
		max.z = fmaxf(mesh.vertices[i].z, max.z);
	}

	mesh.accelerator = new BVHNode();
	mesh.accelerator->bounds = FromMinMax(min, max);
	mesh.accelerator->numTriangles = mesh.numTriangles;
	mesh.accelerator->triangles = new int[mesh.numTriangles];

	for (int i = 0; i < mesh.numTriangles; ++i)
	{
		mesh.accelerator->triangles[i] = i;
	}

	SplitBVHNode(mesh.accelerator, mesh, 3);
}

void SplitBVHNode(BVHNode* node, const Mesh& model, int depth)
{
	if (depth-- == 0)
	{
		return;
	}

	if (node->children == 0)
	{
		if (node->numTriangles > 0)
		{
			node->children = new BVHNode[8];
			Vec3 c = node->bounds.position;
			Vec3 e = node->bounds.size * 0.5f;

			node->children[0].bounds = AABB(c + Vec3(-e.x, +e.y, -e.z), e);
			node->children[1].bounds = AABB(c + Vec3(+e.x, +e.y, -e.z), e);
			node->children[2].bounds = AABB(c + Vec3(-e.x, +e.y, +e.z), e);
			node->children[3].bounds = AABB(c + Vec3(+e.x, +e.y, +e.z), e);
			node->children[4].bounds = AABB(c + Vec3(-e.x, -e.y, -e.z), e);
			node->children[5].bounds = AABB(c + Vec3(+e.x, -e.y, -e.z), e);
			node->children[6].bounds = AABB(c + Vec3(-e.x, -e.y, +e.z), e);
			node->children[7].bounds = AABB(c + Vec3(+e.x, -e.y, +e.z), e);
		}
	}

	if (node->children != 0 && node->numTriangles > 0)
	{
		for (int i = 0; i < 8; ++i)
		{
			node->children[i].numTriangles = 0;

			for (int j = 0; j < node->numTriangles; ++j)
			{
				Triangle t = model.triangles[node->triangles[j]];

				if (TriangleAABB(t, node->children[i].bounds))
				{
					node->children[i].numTriangles += 1;
				}
			}

			if (node->children[i].numTriangles == 0)
			{
				continue;
			}

			node->children[i].triangles = new int[node->children[i].numTriangles];
			int index = 0;

			for (int j = 0; j < node->numTriangles; ++j)
			{
				Triangle t = model.triangles[node->triangles[j]];

				if (TriangleAABB(t, node->children->bounds))
				{
					node->children[i].triangles[index++] = node->triangles[j];
				}
			}
		}

		node->numTriangles = 0;
		delete[] node->triangles;
		node->triangles = 0;

		for (int i = 0; i < 8; ++i)
		{
			SplitBVHNode(&node->children[i], model, depth);
		}
	}
}

void FreeBVHNode(BVHNode* node)
{
	if (node->children != 0)
	{
		for (int i = 0; i < 8; ++i)
		{
			FreeBVHNode(&node->children[i]);
		}
		
		delete[] node->children;
		node->children = 0;

		if (node->numTriangles != 0 || node->triangles != 0)
		{
			delete[] node->triangles;
			node->triangles = 0;
			node->numTriangles = 0;
		}
	}
}

float MeshRay(const Mesh& mesh, const Ray& ray)
{
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; ++i)
		{
			RaycastResult raycastResult;
			Raycast(mesh.triangles[i], ray, &raycastResult);
			float result = raycastResult.t;
			if (result >= 0)
			{
				return result;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles < 0)
			{
				for (int i = 0; i < iterator->numTriangles; ++i)
				{
					RaycastResult raycastResult;
					Raycast(mesh.triangles[iterator->triangles[i]], ray, &raycastResult);
					float r = raycastResult.t;
					if (r >= 0)
					{
						return r;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					RaycastResult result;
					Raycast(iterator->children[i].bounds, ray, &result);

					if (result.t >= 0)
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}

	return -1;
}

bool LineTest(const Mesh& mesh, const Line& line)
{
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; ++i)
		{
			if (Linecast(mesh.triangles[i], line))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; ++i)
				{
					if (Linecast(mesh.triangles[i], line))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (Linecast(mesh.triangles[i], line))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	
	return false;
}

bool MeshSphere(const Mesh& mesh, const Sphere& sphere)
{
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; ++i)
		{
			if (TriangleSphere(mesh.triangles[i], sphere))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; ++i)
				{
					if (TriangleSphere(mesh.triangles[iterator->triangles[i]], sphere))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (AABBShpere(iterator->children[i].bounds, sphere))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}

	return false;
}

bool MeshAABB(const Mesh& mesh, const AABB& aabb)
{
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; ++i)
		{
			if (TriangleAABB(mesh.triangles[i], aabb))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; ++i)
				{
					if (TriangleAABB(mesh.triangles[iterator->triangles[i]], aabb))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (AABBAABB(iterator->children[i].bounds, aabb))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}

	return false;
}

bool MeshOBB(const Mesh& mesh, const OBB& obb)
{
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; ++i)
		{
			if (TriangleOBB(mesh.triangles[i], obb))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; ++i)
				{
					if (TriangleOBB(mesh.triangles[iterator->triangles[i]], obb))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (AABBOBB(iterator->children[i].bounds, obb))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}

	return false;
}

bool MeshPlane(const Mesh& mesh, const Plane& plane)
{
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; ++i)
		{
			if (TrianglePlane(mesh.triangles[i], plane))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; ++i)
				{
					if (TrianglePlane(mesh.triangles[iterator->triangles[i]], plane))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (AABBPlane(iterator->children[i].bounds, plane))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}

	return false;
}

bool MeshTriangle(const Mesh& mesh, const Triangle& triangle)
{
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; ++i)
		{
			if (TriangleTriangle(mesh.triangles[i], triangle))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);
		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());

			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; ++i)
				{
					if (TriangleTriangle(mesh.triangles[iterator->triangles[i]], triangle))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (AABBTriangle(iterator->children[i].bounds, triangle))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}

	return false;
}

Mat4 GetWorldMatrix(const Model& model)
{
	Mat4 translation = Translation(model.position);
	Mat4 rotation = Rotation(
		model.rotation.x, 
		model.rotation.y, 
		model.rotation.z);

	Mat4 localMat = rotation * translation;
	
	Mat4 parentMat;
	if (model.parent != 0)
	{
		parentMat = GetWorldMatrix(*model.parent);
	}

	return localMat * parentMat;
}

OBB GetOBB(const Model& model)
{
	Mat4 world = GetWorldMatrix(model);
	AABB aabb = model.GetBounds();
	OBB obb;

	obb.size = aabb.size;
	obb.position = MultiplyPoint(aabb.position, world);
	obb.orientation = Cut(world, 3, 3);

	return obb;
}

float ModelRay(const Model& model, const Ray& ray)
{
	Mat4 world = GetWorldMatrix(model);
	Mat4 inv = Inverse(world);
	Ray local;
	local.origin = MultiplyPoint(ray.origin, inv);
	local.direction = MultiplyVector(ray.direction, inv);
	local.NormalizeDirection();

	if (model.GetMesh() != 0)
	{
		return MeshRay(*(model.GetMesh()), local);
	}

	return -1;
}

bool LineTest(const Model& model, const Line& line)
{
	Mat4 world = GetWorldMatrix(model);
	Mat4 inv = Inverse(world);

	Line local;
	local.start = MultiplyPoint(line.start, inv);
	local.end = MultiplyPoint(line.end, inv);

	if (model.GetMesh() != 0)
	{
		return LineTest(*(model.GetMesh()), local);
	}

	return false;
}

bool ModelSphere(const Model& model, const Sphere& sphere)
{
	Mat4 world = GetWorldMatrix(model);
	Mat4 inv = Inverse(world);

	Sphere local;
	local.position = MultiplyPoint(sphere.position, inv);

	if (model.GetMesh() != 0)
	{
		return MeshSphere(*(model.GetMesh()), local);
	}

	return false;
}

bool ModelAABB(const Model& model, const AABB& aabb)
{
	Mat4 world = GetWorldMatrix(model);
	Mat4 inv = Inverse(world);

	OBB local;
	local.size = aabb.size;
	local.position = MultiplyPoint(aabb.position, inv);
	local.orientation = Cut(inv, 3, 3);

	if (model.GetMesh() != 0)
	{
		return MeshOBB(*(model.GetMesh()), local);
	}

	return false;
}

bool ModelOBB(const Model& model, const OBB& obb)
{
	Mat4 world = GetWorldMatrix(model);
	Mat4 inv = Inverse(world);

	OBB local;
	local.size = obb.size;
	local.position = MultiplyPoint(obb.position, inv);
	local.orientation = obb.orientation * Cut(inv, 3, 3);

	if (model.GetMesh() != 0)
	{
		return MeshOBB(*(model.GetMesh()), local);
	}

	return false;
}

bool ModelPlane(const Model& model, const Plane& plane)
{
	Mat4 world = GetWorldMatrix(model);
	Mat4 inv = Inverse(world);

	Plane local;
	local.normal = MultiplyVector(plane.normal, inv);
	local.distance = plane.distance;

	if (model.GetMesh() != 0)
	{
		return MeshPlane(*(model.GetMesh()), local);
	}

	return false;
}

bool ModelTriangle(const Model& model, const Triangle& triangle)
{
	Mat4 world = GetWorldMatrix(model);
	Mat4 inv = Inverse(world);

	Triangle local;
	local.a = MultiplyPoint(triangle.a, inv);
	local.b = MultiplyPoint(triangle.b, inv);
	local.c = MultiplyPoint(triangle.c, inv);

	if (model.GetMesh() != 0)
	{
		return MeshTriangle(*(model.GetMesh()), local);
	}

	return false;
}

Point Intersection(Plane plane1, Plane plane2, Plane plane3) 
{
	Mat3 D(
		plane1.normal.x, plane2.normal.x, plane3.normal.x,
		plane1.normal.y, plane2.normal.y, plane3.normal.y, 
		plane1.normal.z, plane2.normal.z, plane3.normal.z
	);

	Vec3 A(-plane1.distance, -plane2.distance, -plane3.distance);

	Mat3 Dx = D;
	Mat3 Dy = D;
	Mat3 Dz = D;
	Dx._11 = A.x; Dx._12 = A.y; Dx._13 = A.z;
	Dy._21 = A.x; Dx._22 = A.y; Dx._23 = A.z;
	Dz._31 = A.x; Dx._32 = A.y; Dx._33 = A.z;

	float detD = Determinant(D);
	if (CMP(detD, 0))
	{
		return Point();
	}

	float detDx = Determinant(Dx);
	float detDy = Determinant(Dy);
	float detDz = Determinant(Dz);

	return Point(detDx / detD, detDy / detD, detDz / detD);
}

void GetCorners(const Frustum& frustum, Vec3* outCorners)
{
	outCorners[0] = Intersection(frustum.near, frustum.top,    frustum.left);
	outCorners[1] = Intersection(frustum.near, frustum.top,    frustum.right);
	outCorners[2] = Intersection(frustum.near, frustum.bottom, frustum.left);
	outCorners[3] = Intersection(frustum.near, frustum.bottom, frustum.right);
	outCorners[4] = Intersection(frustum.far,  frustum.top,    frustum.left);
	outCorners[5] = Intersection(frustum.far,  frustum.top,    frustum.right);
	outCorners[6] = Intersection(frustum.far,  frustum.bottom, frustum.left);
	outCorners[7] = Intersection(frustum.far,  frustum.bottom, frustum.right);
}

bool Intersects(const Frustum& frustum, const Point& point)
{
	for (int i = 0; i < 6; ++i)
	{
		Vec3 normal = frustum.planes[i].normal;
		float distance = frustum.planes[i].distance;
		float side = Dot(point, normal) + distance;

		if (side < 0.0f)
		{
			return false;
		}
	}

	return true;
}

bool Intersects(const Frustum& frustum, const Sphere& sphere)
{
	for (int i = 0; i < 6; ++i)
	{
		Vec3 normal = frustum.planes[i].normal;
		float distance = frustum.planes[i].distance;
		float side = Dot(sphere.position, normal) + distance;

		if (side < -sphere.radius)
		{
			return false;
		}
	}

	return true;
}

float Classify(const AABB& aabb, const Plane& plane)
{
	float r = fabsf(aabb.size.x * plane.normal.x) +
		fabsf(aabb.size.y * plane.normal.y) +
		fabsf(aabb.size.z * plane.normal.z);
	float d = Dot(plane.normal, aabb.position) + plane.distance;

	if (fabsf(d) < r)
	{
		return 0.0f;
	}
	else if (d < 0.0f)
	{
		return d + r;
	}

	return d - r;
}

float Classify(const OBB& obb, const Plane& plane)
{
	Vec3 normal = MultiplyVector(plane.normal, obb.orientation);
	float r = fabs(obb.size.x * plane.normal.x) +
		fabs(obb.size.y * plane.normal.y) +
		fabs(obb.size.z * plane.normal.z);
	float d = Dot(plane.normal, obb.position) + plane.distance;

	if (fabsf(d) < r)
	{
		return 0.0f;
	}
	else if (d < 0.0f)
	{
		return d + r;
	}

	return d - r;
}

bool Intersects(const Frustum& frustum, const AABB& aabb)
{
	for (int i = 0; i < 6; ++i)
	{
		if (Classify(aabb, frustum.planes[i]) < 0)
		{
			return false;
		}
	}

	return true;
}

bool Intersects(const Frustum& frustum, const OBB& obb)
{
	for (int i = 0; i < 6; ++i)
	{
		if (Classify(obb, frustum.planes[i]) < 0)
		{
			return false;
		}
	}

	return true;
}

Vec3 Uproject(const Vec3& viewportPoint, const Vec2& viewportOrigin,
	const Vec2& viewportSize, const Mat4& view, const Mat4& projection)
{
	float normalized[4] = {
		(viewportPoint.x - viewportOrigin.x) / viewportSize.x,
		(viewportPoint.y - viewportOrigin.y) / viewportSize.y,
		viewportPoint.z,
		1.0f
	};

	float ndcSpace[4] = {
		normalized[0],
		normalized[1],
		normalized[2],
		normalized[3],
	};

	ndcSpace[0] = ndcSpace[0] * 2.0f - 1.0f;
	ndcSpace[1] = 1.0f - ndcSpace[1] * 2.0f;

	if (ndcSpace[2] < 0.0f)
	{
		ndcSpace[2] = 0.0f;
	}
	if (ndcSpace[2] > 1.0f)
	{
		ndcSpace[2] = 1.0f;
	}

	Mat4 invProjection = Inverse(projection);
	float eyeSpace[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	Multiply(eyeSpace, ndcSpace, 1, 4, invProjection.asArray, 4, 4);

	Mat4 invView = Inverse(view);
	float worldSpace[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	Multiply(worldSpace, eyeSpace, 1, 4, invView.asArray, 4, 4);

	if (!CMP(worldSpace[3], 0.0f))
	{
		worldSpace[0] /= worldSpace[3];
		worldSpace[1] /= worldSpace[3];
		worldSpace[2] /= worldSpace[3];
	}

	return Vec3(worldSpace[0], worldSpace[1], worldSpace[2]);
}

Ray GetPickRay(const Vec2& viewportPoint, const Vec2& viewportOrigin,
	const Vec2& viewportSize, const Mat4& view, const Mat4& projection)
{
	Vec3 nearPoint(viewportPoint.x, viewportPoint.y, 0.0f);
	Vec3 farPoint(viewportPoint.x, viewportPoint.y, 1.0f);
	
	Vec3 pNear = Uproject(nearPoint, viewportOrigin, viewportSize,
		view, projection);
	Vec3 pFar = Uproject(farPoint, viewportOrigin, viewportSize,
		view, projection);

	Vec3 normal = Normalized(pFar - pNear);
	Vec3 origin = pNear;

	return Ray(origin, normal);
}