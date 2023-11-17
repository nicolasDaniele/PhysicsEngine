#pragma once

#include "Vectors.h"

#define CLAMP(number, minimum, maximum) number = (number < minimum) ? minimum : (number > maximum) ? maximum : number
#define OVERLAP(aMin, aMax, bMin, bMax) ((bMin <= aMax) && (aMin <= bMax|))

#define PointLine(point, line) PointOnLine(point, line)
#define LinePoint(line, point) PointOnLine(point, line)
#define CircleLine(circle, line) LineCircle(line, circle)
#define RectangleLine(rectangle, line) LineRectangle(line, rectangle)
#define OrientedRectangleLine(rect, line) LineOrientedRectangle(line, rect)
#define RectangleCircle(rectangle, circle) CircleRectancle(circle, rectangle)
#define OrientedRectangleCircle(rectangle, circle) CircleOrientedRectangle(circle, rectangle)
#define OrientedRectangleRectangle(oriented, normal) RectangleOrientedRectangle(normal, oriented)

typedef Vec2 Point2D;

struct Line2D
{
	Point2D start;
	Point2D end;

	inline Line2D() {}
	inline Line2D(const Point2D& s, const Point2D& e)
		: start(s), end(e) {}
};
struct Circle
{
	Point2D position;
	float radius;

	inline Circle() : radius(1.0f) {}
	inline Circle(const Point2D& p, float r)
		: position(p), radius(r) {}
};

struct Rectangle2D
{
	Point2D origin;
	Vec2 size;

	inline Rectangle2D() : size({1, 1}) {}
	inline Rectangle2D(const Point2D& o, const Vec2& s)
		: origin(o), size(s) {}

};

struct OrientedRectangle
{
	Point2D position;
	Vec2 halfExtents;
	float rotation;

	inline OrientedRectangle() 
		: halfExtents({1.0f, 1.0f}), rotation(0.0f) {}
	inline OrientedRectangle(const Point2D& p, Vec2& e)
		: position(p), halfExtents(e), rotation(0.0f) {}
	inline OrientedRectangle(const Point2D& p, const Vec2& e, float r)
		: position(p), halfExtents(e), rotation(r) {}

};

struct Interval2D
{
	float min;
	float max;
};

struct BoundingShape
{
	int numCircles;
	Circle* circles;
	int numRectangles;
	Rectangle2D* rectangles;

	inline BoundingShape() :
		numCircles(0), circles(0),
		numRectangles(0), rectangles(0) { }
};

float Legth(const Line2D& line);
float LengthSq(const Line2D& line);

Vec2 GetMin(const Rectangle2D& rect);
Vec2 GetMax(const Rectangle2D& rect);
Rectangle2D FromMinMax(const Vec2& min, const Vec2& max);

bool PointOnLine(const Point2D& point, const Line2D& line);
bool PointInCircle(const Point2D& point, const Circle& circle);
bool PointInRectangle(const Point2D& point, const Rectangle2D& rectangle);
bool PointInOrientedRectangle(const Point2D& point, 
	const OrientedRectangle& rectangle);

bool LineCircle(const Line2D& line, const Circle& circle);
bool LineRectangle(const Line2D& line, const Rectangle2D& rectangle);
bool LineOrientedRectangle(const Line2D& line, const OrientedRectangle& rect);

bool CircleCircle(const Circle& circle1, const Circle& circle2);
bool CircleRectangle(const Circle& circle, const Rectangle2D& rectangle);
bool CircleOrientedRectangle(const Circle& circle, const OrientedRectangle& rectangle);
bool RectangleRectangle(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2);

Interval2D GetInteval(const Rectangle2D& rectangle, const Vec2& axis);
bool OverlapOnAxis(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2, const Vec2& axis);
bool RectangleRectangleSAT(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2);
Interval2D GetInterval(const OrientedRectangle& rectangle, const Vec2& axis);
bool OverlapOnAxis(const Rectangle2D& rectangle1, 
	const OrientedRectangle& rectangle2, const Vec2& axis);
bool RectangleOrientedRectangle(const Rectangle2D& rectangle1, const OrientedRectangle& rectangle2);
bool OrientedRectangleOrientedRectangle(const OrientedRectangle& rectangle1, 
	const OrientedRectangle& rectangle2);

Circle ContainingCircle(Point2D* pointsArray, int ArrayCount);
Rectangle2D ContainingRectangle(Point2D* pointArray, int arrayCount);
bool PointInShape(const BoundingShape& shape, const Point2D& point);