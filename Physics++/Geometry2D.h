#pragma once

#ifndef _H_2D_GEOMETRY_
#define _H_2D_GEOMETRY_

#include "vectors.h"

typedef vec2 Point2D;

typedef struct Line2D
{
	Point2D start;
	Point2D end;

	inline Line2D() {}
	inline Line2D(const Point2D& s, const Point2D& e)
		: start(s), end(e) {}
} Line2D;

typedef struct Circle
{
	Point2D position;
	float radius;

	inline Circle() : radius(1.0f) {}
	inline Circle(const Point2D& p, float r)
		: position(p), radius(r) {}
} Circle;

float Legth(const Line2D& line);
float LengthSq(const Line2D& line);

#endif