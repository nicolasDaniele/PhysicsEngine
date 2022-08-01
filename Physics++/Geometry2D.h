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

typedef struct Rectangle2D
{
	Point2D origin;
	vec2 size;

	inline Rectangle2D() : size({1, 1}) {}
	inline Rectangle2D(const Point2D& o, const vec2& s)
		: origin(o), size(s) {}

} Rectangle2D;

typedef struct OrientedRectangle
{
	Point2D position;
	vec2 halfExtents;
	float rotation;

	inline OrientedRectangle() 
		: halfExtents({1.0f, 1.0f}), rotation(0.0f) {}
	inline OrientedRectangle(const Point2D& p, vec2& e)
		: position(p), halfExtents(e), rotation(0.0f) {}
	inline OrientedRectangle(const Point2D& p, const vec2& e, float r)
		: position(p), halfExtents(e), rotation(r) {}

} OrientedRectangle;

typedef struct Interval2D
{
	float min;
	float max;
} Interval2D;

// Line2D methods
float Legth(const Line2D& line);
float LengthSq(const Line2D& line);

// Rectangle2D methods
vec2 GetMin(const Rectangle2D& rect);
vec2 GetMax(const Rectangle2D& rect);
Rectangle2D FromMinMax(const vec2& min, const vec2& max);

// Point Containment
bool PointOnLine(const Point2D& point, const Line2D& line);
bool PointInCircle(const Point2D& point, const Circle& circle);
bool PointInRectangle(const Point2D& point, const Rectangle2D& rectangle);
bool PointInOrientedRectangle(const Point2D& point, 
	const OrientedRectangle& rectangle);

// Line Intersection
bool LineCircle(const Line2D& line, const Circle& circle);
bool LineRectangle(const Line2D& line, const Rectangle2D& rectangle);
bool LineOrientedRectangle(const Line2D& line, const OrientedRectangle& rect);

#define PointLine(point, line) PointOnLine(point, line)
#define LinePoint(line, point) PointOnLine(point, line)
#define CircleLine(circle, line) LineCircle(line, circle)
#define RectangleLine(rectangle, line) LineRectangle(line, rectangle)
#define OrientedRectangleLine(rect, line) LineOrientedRectangle(line, rect)

//		COLLISIONS
bool CircleCircle(const Circle& circle1, const Circle& circle2);
bool CircleRectangle(const Circle& circle, const Rectangle2D& rectangle);
#define RectangleCircle(rectangle, circle) CircleRectancle(circle, rectangle)
bool CircleOrientedRectangle(const Circle& circle, const OrientedRectangle& rectangle);
#define OrientedRectangleCircle(rectangle, circle) CircleOrientedRectangle(circle, rectangle)
bool RectangleRectangle(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2);

//		COLLISIONS USING THE SEPARATING AXIS THEOREM (SAT)
// Rectangle-Rectangle
Interval2D GetInteval(const Rectangle2D& rectangle, const vec2& axis);
bool OverlapOnAxis(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2, const vec2& axis);
bool RectangleRectangleSAT(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2);
// Rectangle-OrientedRectangle
Interval2D GetInterval(const OrientedRectangle& rectangle, const vec2& axis);
bool OverlapOnAxis(const Rectangle2D& rectangle1, 
	const OrientedRectangle& rectangle2, const vec2& axis);
bool RectangleOrientedRectangle(const Rectangle2D& rectangle1, const OrientedRectangle& rectangle2);
#define OrientedRectangleRectangle(oriented, normal) RectangleOrientedRectangle(normal, oriented)
// OrientedRectangle-OrientedRectangle
bool OrientedRectangleOrientedRectangle(const OrientedRectangle& rectangle1, 
	const OrientedRectangle& rectangle2);

#endif