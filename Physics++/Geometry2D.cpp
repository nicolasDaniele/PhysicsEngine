#include "Geometry2D.h"
#include "matrices.h"
#include <cmath>
#include <cfloat>

#define CMP(x, y) (fabsf((x)-(y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))
#define CLAMP(number, minimum, maximum) number = (number < minimum) ? minimum : (number > maximum) ? maximum : number
#define OVERLAP(aMin, aMax, bMin, bMax) ((bMin <= aMax) && (aMin <= bMax|))

// Line2D methods
float Legth(const Line2D& line)
{
	return Magnitude(line.end - line.start);
}

float LengthSq(const Line2D& line)
{
	return MagnitudeSq(line.end - line.start);
}

// Rectangle2D methods
vec2 GetMin(const Rectangle2D& rect)
{
	vec2 p1 = rect.origin;
	vec2 p2 = rect.origin + rect.size;

	return { fminf(p1.x, p2.x), fminf(p1.y, p2.y) };
}

vec2 GetMax(const Rectangle2D& rect)
{
	vec2 p1 = rect.origin;
	vec2 p2 = rect.origin + rect.size;

	return { fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y) };
}

Rectangle2D FromMinMax(const vec2& min, const vec2& max)
{
	return Rectangle2D(min, max - min);
}

// Point Containment
bool PointOnLine(const Point2D& point, const Line2D& line)
{
	// Find the slope
	float dy = (line.end.y - line.start.y);
	float dx = (line.end.x - line.start.x);
	float M = dy / dx;

	// Find the Y-Intercept
	float B = line.start.y - M * line.start.x;

	// Check line equation
	return CMP(point.y, M * point.x + B);
}

bool PointInCircle(const Point2D& point, const Circle& circle)
{
	Line2D line(point, circle.position);

	if (LengthSq(line) < circle .radius * circle.radius)
	{
		return true;
	}

	return false;
}

bool PointInRectangle(const Point2D& point, const Rectangle2D& rectangle)
{
	vec2 min = GetMin(rectangle);
	vec2 max = GetMax(rectangle);

	return min.x <= point.x &&
		min.y <= point.y &&
		point.x <= max.x &&
		point.y <= max.y;
}

bool PointInOrientedRectangle(const Point2D& point,
	const OrientedRectangle& rectangle)
{
	vec2 rotVector = point - rectangle.position;
	float theta = -DEG2RAD(rectangle.rotation);
	float zRotation2x2[] =
	{
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta)
	};

	Multiply(rotVector.asArray, 
		vec2(rotVector.x, rotVector.y).asArray,
		1, 2, zRotation2x2, 2, 2);

	Rectangle2D localRectangle(Point2D(),
		rectangle.halfExtents * 2.0f);
	vec2 localPoint = rotVector + rectangle.halfExtents;

	return PointInRectangle(localPoint, localRectangle);
}

// Line Intersection
bool LineCircle(const Line2D& line, const Circle& circle)
{
	vec2 ab = line.end - line.start;
	float t = Dot(circle.position - line.start, ab) / Dot(ab, ab);

	if (t < 0.0f || t > 1.0f)
	{
		return false;
	}

	Point2D closestPoint = line.start + ab * t;
	Line2D circleToClosest(circle.position, closestPoint);

	return LengthSq(circleToClosest) < circle.radius * circle.radius;
}

bool LineRectangle(const Line2D& line, const Rectangle2D& rectangle)
{
	if (PointInRectangle(line.start, rectangle) ||
		PointInRectangle(line.end, rectangle))
	{
		return true;
	}

	vec2 norm = Normalized(line.end - line.start);
	norm.x = (norm.x != 0) ? 1.0f / norm.x : 0;
	norm.y = (norm.y != 0) ? 1.0f / norm.y : 0;
	
	vec2 min = (GetMin(rectangle) - line.start) * norm;
	vec2 max = (GetMax(rectangle) - line.start) * norm;

	float tmin = fmaxf
	(
		fminf(min.x, max.x),
		fminf(min.y, max.y)
	);
	float tmax = fminf
	(
		fmaxf(min.x, max.x),
		fmaxf(min.y, max.y)
	);

	if (tmax < 0 || tmin > tmax)
	{
		return false;
	}

	float t = (tmin < 0.0f) ? tmax : tmin;

	return t > 0.0f && t * t < LengthSq(line);
}

bool LineOrientedRectangle(const Line2D& line, const OrientedRectangle& rect)
{
	float theta = -DEG2RAD(rect.rotation);
	float zRotation2x2[] =
	{
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta)
	};
	
	Line2D localLine;
	vec2 rotVector = line.start - rect.position;

	Multiply(rotVector.asArray, vec2(rotVector.x, rotVector.y).asArray,
		1, 2, zRotation2x2, 2, 2);
	
	localLine.start = rotVector + rect.halfExtents;
	rotVector = line.end - rect.position;

	Multiply(rotVector.asArray, vec2(rotVector.x, rotVector.y).asArray,
		1, 2, zRotation2x2, 2, 2);

	localLine.end = rotVector + rect.halfExtents;

	Rectangle2D localRectangle(Point2D(), 
		rect.halfExtents * 2.0f);

	return LineRectangle(localLine, localRectangle);
}

//		COLLISIONS
bool CircleCircle(const Circle& circle1, const Circle& circle2)
{
	Line2D line(circle1.position, circle2.position);
	float radiiSum = circle1.radius + circle2.radius;

	return LengthSq(line) <= radiiSum * radiiSum;
}

bool CircleRectangle(const Circle& circle, const Rectangle2D& rectangle)
{
	vec2 min = GetMin(rectangle);
	vec2 max = GetMax(rectangle);
	Point2D closestPoint = circle.position;

	CLAMP(closestPoint.x, min.x, max.x);
	CLAMP(closestPoint.y, min.y, max.y);

	Line2D line(circle.position, closestPoint);
	return LengthSq(line) <= circle.radius * circle.radius;
}

bool CircleOrientedRectangle(const Circle& circle, const OrientedRectangle& rectangle)
{
	vec2 r = circle.position - rectangle.position;
	float theta = -DEG2RAD(rectangle.rotation);
	float zRotation2x2[] =
	{
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta)
	};

	Multiply(r.asArray, vec2(r.x, r.y).asArray, 1, 2, zRotation2x2, 2, 2);

	Circle localCircle(r + rectangle.halfExtents, circle.radius);
	Rectangle2D localRectangle(Point2D(), rectangle.halfExtents * 2.0f);

	return CircleRectangle(localCircle, localRectangle);
}

bool RectangleRectangle(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2)
{
	vec2 aMin = GetMin(rectangle1);
	vec2 aMax = GetMax(rectangle1);
	vec2 bMin = GetMin(rectangle2);
	vec2 bMax = GetMax(rectangle2);

	bool overX = ((bMin.x <= aMax.x) && (aMin.x <= bMax.x));
	bool overY = ((bMin.y <= aMax.y) && (aMin.y <= bMax.y));

	return overX && overY;
}

//		COLLISIONS USING THE SEPARATING AXIS THEOREM (SAT)
// Rectangle-Rectangle
Interval2D GetInteval(const Rectangle2D& rectangle, const vec2& axis)
{
	Interval2D result;
	vec2 min = GetMin(rectangle);
	vec2 max = GetMax(rectangle);

	// Get all vertices of rectangle
	vec2 vertices[] =
	{
		vec2(min.x, min.y), vec2(min.x, max.y),
		vec2(max.x, max.y), vec2(max.x, min.y)
	};

	result.min = result.max = Dot(axis, vertices[0]);

	for(int i = 0; i < 4; ++i)
	{
		float projection = Dot(axis, vertices[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

bool OverlapOnAxis(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2, const vec2& axis)
{
	Interval2D a = GetInteval(rectangle1, axis);
	Interval2D b = GetInteval(rectangle2, axis);

	return((b.min <= a.max) && (a.min <= b.max));
}

bool RectangleRectangleSAT(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2)
{
	vec2 axisToTest[] = { vec2(1, 0), vec2(0, 1) };

	for (int i = 0; i < 2; ++i)
	{
		// Intervals don't overlap, separating axis found
		if (!OverlapOnAxis(rectangle1, rectangle2, axisToTest[i]))
			return false; // No collision found
	}

	// All intervals overlap, separating axis not found
	return true; // Collision found
}

//Rectangle-OrientedRectangle
Interval2D GetInterval(const OrientedRectangle& rectangle, const vec2& axis)
{
	Rectangle2D r = Rectangle2D(Point2D(rectangle.position - rectangle.halfExtents),
		rectangle.halfExtents * 2);

	vec2 min = GetMin(r);
	vec2 max = GetMax(r);
	vec2 vertices[] =
	{
		min, max,
		vec2(min.x, max.y), vec2(max.x, min.y)
	};

	float t = DEG2RAD(rectangle.rotation);
	float zRot[] =
	{
		cosf(t), sinf(t),
		-sinf(t), cosf(t)
	};

	for (int i = 0; i < 4; ++i)
	{
		vec2 r = vertices[i] - rectangle.position;
		Multiply(r.asArray, vec2(r.x, r.y).asArray, 1, 2, zRot, 2, 2);
		vertices[i] = r + rectangle.position;
	}

	Interval2D result;
	result.min = result.max = Dot(axis, vertices[0]);
	for (int i = 0; i < 4; ++i)
	{
		float projection = Dot(axis, vertices[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

bool OverlapOnAxis(const Rectangle2D& rectangle1,
	const OrientedRectangle& rectangle2, const vec2& axis)
{
	Interval2D a = GetInteval(rectangle1, axis);
	Interval2D b = GetInterval(rectangle2, axis);

	return((b.min <= a.max) && (a.min <= b.max));
}

bool RectangleOrientedRectangle(const Rectangle2D& rectangle1, const OrientedRectangle& rectangle2)
{
	vec2 axisToTest[] =
	{
		vec2(1, 0), vec2(0, 1),
		vec2(), vec2()
	};

	float t = DEG2RAD(rectangle2.rotation);
	float zRot[] =
	{
		cosf(t), sinf(t),
		-sinf(t), cosf(t)
	};

	vec2 axis = Normalized(vec2(rectangle2.halfExtents.x, 0));
	Multiply(axisToTest[2].asArray, axis.asArray, 1, 2, zRot, 2, 2);

	axis = Normalized(vec2(0, rectangle2.halfExtents.y));
	Multiply(axisToTest[3].asArray, axis.asArray, 1, 2, zRot, 2, 2);

	for (int i = 0; i < 4; ++i)
	{
		if (!OverlapOnAxis(rectangle1, rectangle2, axis))
			return false; // No collision was found
	}

	return true; // Collision was found
}