#pragma once
#ifndef _H_GEMOETRY_3D_
#define _H_GEMOETRY_3D_

#include "vectors.h"
#include "matrices.h"

typedef vec3 Point;

typedef struct Line 
{
	Point start;
	Point end;

	inline Line() { }
	inline Line(const Point& s, const Point& e) :
	start(s), end(e) { }
} Line;

typedef struct Ray
{
	Point origin;
	vec3 direction;

	inline Ray() : direction(0.0f, 0.0f, 1.0f) { }
	inline Ray(const Point& o, const vec3& d) :
		origin(o), direction(d)
	{
		NormalizeDirection();
	}
	inline void NormalizeDirection()
	{
		Normalize(direction);
	}
} Ray;

typedef struct Sphere
{
	Point position;
	float radius;

	inline Sphere() : radius(1.0f) { }
	inline Sphere(const Point& p, float r) :
		position(p), radius(r) { }
} Sphere;

typedef struct AABB
{
	Point origin;
	vec3 size;

	inline AABB() : size(1, 1, 1) { }
	inline AABB(const Point& o, const vec3& s) :
		origin(o), size(s) { }
} AABB;

// Line methods
float Lenght(const Line& line);
float LenghtSq(const Line& line);

// Ray method
Ray FromPoints(const Point& from, const Point& to);

// AABB methods
vec3 GetMin(const AABB& aabb);
vec3 GetMax(const AABB& aabb);
AABB FormMinMax(const vec3& min, const vec3& max);

#endif
