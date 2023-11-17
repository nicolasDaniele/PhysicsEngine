#include "Geometry2D.h"
#include "Matrices.h"
#include <cmath>
#include <cfloat>

float Legth(const Line2D& line)
{
	return Magnitude(line.end - line.start);
}

float LengthSq(const Line2D& line)
{
	return MagnitudeSq(line.end - line.start);
}

Vec2 GetMin(const Rectangle2D& rect)
{
	Vec2 p1 = rect.origin;
	Vec2 p2 = rect.origin + rect.size;

	return { fminf(p1.x, p2.x), fminf(p1.y, p2.y) };
}

Vec2 GetMax(const Rectangle2D& rect)
{
	Vec2 p1 = rect.origin;
	Vec2 p2 = rect.origin + rect.size;

	return { fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y) };
}

Rectangle2D FromMinMax(const Vec2& min, const Vec2& max)
{
	return Rectangle2D(min, max - min);
}

bool PointOnLine(const Point2D& point, const Line2D& line)
{
	float dy = (line.end.y - line.start.y);
	float dx = (line.end.x - line.start.x);
	float M = dy / dx;

	float B = line.start.y - M * line.start.x;

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
	Vec2 min = GetMin(rectangle);
	Vec2 max = GetMax(rectangle);

	return min.x <= point.x &&
		min.y <= point.y &&
		point.x <= max.x &&
		point.y <= max.y;
}

bool PointInOrientedRectangle(const Point2D& point,
	const OrientedRectangle& rectangle)
{
	Vec2 rotVector = point - rectangle.position;
	float theta = -DEG2RAD(rectangle.rotation);
	float zRotation2x2[] =
	{
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta)
	};

	Multiply(rotVector.asArray, 
		Vec2(rotVector.x, rotVector.y).asArray,
		1, 2, zRotation2x2, 2, 2);

	Rectangle2D localRectangle(Point2D(),
		rectangle.halfExtents * 2.0f);
	Vec2 localPoint = rotVector + rectangle.halfExtents;

	return PointInRectangle(localPoint, localRectangle);
}

bool LineCircle(const Line2D& line, const Circle& circle)
{
	Vec2 ab = line.end - line.start;
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

	Vec2 norm = Normalized(line.end - line.start);
	norm.x = (norm.x != 0) ? 1.0f / norm.x : 0;
	norm.y = (norm.y != 0) ? 1.0f / norm.y : 0;
	
	Vec2 min = (GetMin(rectangle) - line.start) * norm;
	Vec2 max = (GetMax(rectangle) - line.start) * norm;

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
	Vec2 rotVector = line.start - rect.position;

	Multiply(rotVector.asArray, Vec2(rotVector.x, rotVector.y).asArray,
		1, 2, zRotation2x2, 2, 2);
	
	localLine.start = rotVector + rect.halfExtents;
	rotVector = line.end - rect.position;

	Multiply(rotVector.asArray, Vec2(rotVector.x, rotVector.y).asArray,
		1, 2, zRotation2x2, 2, 2);

	localLine.end = rotVector + rect.halfExtents;

	Rectangle2D localRectangle(Point2D(), 
		rect.halfExtents * 2.0f);

	return LineRectangle(localLine, localRectangle);
}

bool CircleCircle(const Circle& circle1, const Circle& circle2)
{
	Line2D line(circle1.position, circle2.position);
	float radiiSum = circle1.radius + circle2.radius;

	return LengthSq(line) <= radiiSum * radiiSum;
}

bool CircleRectangle(const Circle& circle, const Rectangle2D& rectangle)
{
	Vec2 min = GetMin(rectangle);
	Vec2 max = GetMax(rectangle);
	Point2D closestPoint = circle.position;

	CLAMP(closestPoint.x, min.x, max.x);
	CLAMP(closestPoint.y, min.y, max.y);

	Line2D line(circle.position, closestPoint);
	return LengthSq(line) <= circle.radius * circle.radius;
}

bool CircleOrientedRectangle(const Circle& circle, const OrientedRectangle& rectangle)
{
	Vec2 r = circle.position - rectangle.position;
	float theta = -DEG2RAD(rectangle.rotation);
	float zRotation2x2[] =
	{
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta)
	};

	Multiply(r.asArray, Vec2(r.x, r.y).asArray, 1, 2, zRotation2x2, 2, 2);

	Circle localCircle(r + rectangle.halfExtents, circle.radius);
	Rectangle2D localRectangle(Point2D(), rectangle.halfExtents * 2.0f);

	return CircleRectangle(localCircle, localRectangle);
}

bool RectangleRectangle(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2)
{
	Vec2 aMin = GetMin(rectangle1);
	Vec2 aMax = GetMax(rectangle1);
	Vec2 bMin = GetMin(rectangle2);
	Vec2 bMax = GetMax(rectangle2);

	bool overX = ((bMin.x <= aMax.x) && (aMin.x <= bMax.x));
	bool overY = ((bMin.y <= aMax.y) && (aMin.y <= bMax.y));

	return overX && overY;
}

Interval2D GetInteval(const Rectangle2D& rectangle, const Vec2& axis)
{
	Interval2D result;
	Vec2 min = GetMin(rectangle);
	Vec2 max = GetMax(rectangle);

	Vec2 vertices[] =
	{
		Vec2(min.x, min.y), Vec2(min.x, max.y),
		Vec2(max.x, max.y), Vec2(max.x, min.y)
	};

	result.min = result.max = Dot(axis, vertices[0]);

	for(int i = 0; i < 4; i++)
	{
		float projection = Dot(axis, vertices[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

bool OverlapOnAxis(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2, const Vec2& axis)
{
	Interval2D a = GetInteval(rectangle1, axis);
	Interval2D b = GetInteval(rectangle2, axis);

	return((b.min <= a.max) && (a.min <= b.max));
}

bool RectangleRectangleSAT(const Rectangle2D& rectangle1, const Rectangle2D& rectangle2)
{
	Vec2 axisToTest[] = { Vec2(1, 0), Vec2(0, 1) };

	for (int i = 0; i < 2; i++)
	{
		if (!OverlapOnAxis(rectangle1, rectangle2, axisToTest[i]))
			return false;
	}

	return true;
}

Interval2D GetInterval(const OrientedRectangle& rectangle, const Vec2& axis)
{
	Rectangle2D r = Rectangle2D(Point2D(rectangle.position - rectangle.halfExtents),
		rectangle.halfExtents * 2);

	Vec2 min = GetMin(r);
	Vec2 max = GetMax(r);
	Vec2 vertices[] =
	{
		min, max,
		Vec2(min.x, max.y), Vec2(max.x, min.y)
	};

	float t = DEG2RAD(rectangle.rotation);
	float zRot[] =
	{
		cosf(t), sinf(t),
		-sinf(t), cosf(t)
	};

	for (int i = 0; i < 4; i++)
	{
		Vec2 r = vertices[i] - rectangle.position;
		Multiply(r.asArray, Vec2(r.x, r.y).asArray, 1, 2, zRot, 2, 2);
		vertices[i] = r + rectangle.position;
	}

	Interval2D result;
	result.min = result.max = Dot(axis, vertices[0]);
	for (int i = 0; i < 4; i++)
	{
		float projection = Dot(axis, vertices[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

bool OverlapOnAxis(const Rectangle2D& rectangle1,
	const OrientedRectangle& rectangle2, const Vec2& axis)
{
	Interval2D a = GetInteval(rectangle1, axis);
	Interval2D b = GetInterval(rectangle2, axis);

	return((b.min <= a.max) && (a.min <= b.max));
}

bool RectangleOrientedRectangle(const Rectangle2D& rectangle1, const OrientedRectangle& rectangle2)
{
	Vec2 axisToTest[] =
	{
		Vec2(1, 0), Vec2(0, 1),
		Vec2(), Vec2()
	};

	float t = DEG2RAD(rectangle2.rotation);
	float zRot[] =
	{
		cosf(t), sinf(t),
		-sinf(t), cosf(t)
	};

	Vec2 axis = Normalized(Vec2(rectangle2.halfExtents.x, 0));
	Multiply(axisToTest[2].asArray, axis.asArray, 1, 2, zRot, 2, 2);

	axis = Normalized(Vec2(0, rectangle2.halfExtents.y));
	Multiply(axisToTest[3].asArray, axis.asArray, 1, 2, zRot, 2, 2);

	for (int i = 0; i < 4; i++)
	{
		if (!OverlapOnAxis(rectangle1, rectangle2, axis))
			return false;
	}

	return true;
}

bool OrientedRectangleOrientedRectangle(const OrientedRectangle& rectangle1,
	const OrientedRectangle& rectangle2)
{
	Rectangle2D local1(Point2D(), rectangle1.halfExtents * 2.0f);
	Vec2 r = rectangle2.position - rectangle1.position;

	OrientedRectangle local2(rectangle2.position, 
		rectangle2.halfExtents, rectangle2.rotation);
	local2.rotation = rectangle2.rotation - rectangle1.rotation;

	float t = -DEG2RAD(rectangle1.rotation);
	float zRot[] =
	{
		cosf(t), sinf(t),
		-sinf(t), cosf(t)
	};

	Multiply(r.asArray, Vec2(r.x, r.y).asArray, 1, 2, zRot, 2, 2);
	local2.position = r + rectangle1.halfExtents;

	return RectangleOrientedRectangle(local1, local2);
}

Circle ContainingCircle(Point2D* pointsArray, int arrayCount)
{
	Point2D center;

	for (int i = 0; i < arrayCount; i++)
	{
		center = center + pointsArray[i];
	}

	center = center * (1.0f / (float)arrayCount);

	Circle result(center, 1.0f);
	result.radius = MagnitudeSq(center - pointsArray[0]);
	for (int i = 1; i < arrayCount; i++)
	{
		float distance = MagnitudeSq(center - pointsArray[i]);
		if (distance > result.radius)
		{
			result.radius = distance;
		}
	}

	result.radius = sqrtf(result.radius);
	return result;
}

Rectangle2D ContainingRectangle(Point2D* pointArray, int arrayCount)
{
	Vec2 min = pointArray[0];
	Vec2 max = pointArray[0];

	for (int i = 0; i < arrayCount; i++)
	{
		min.x = pointArray[i].x < min.x ?
			pointArray[i].x : min.x;
		min.y = pointArray[i].y < min.y ?
			pointArray[i].y : min.y;
		max.x = pointArray[i].x > max.x ?
			pointArray[i].x : max.x;
		max.y = pointArray[i].y > max.y ?
			pointArray[i].y : max.y;
	}

	return FromMinMax(min, max);
}

bool PointInShape(const BoundingShape& shape, const Point2D& point)
{
	for (int i = 0; i < shape.numCircles; i++)
	{
		if (PointInCircle(point, shape.circles[i]))
		{
			return true;
		}
	}

	for (int i = 0; i < shape.numRectangles; i++)
	{
		if (PointInRectangle(point, shape.rectangles[i]))
		{
			return true;
		}
	}

	return false;
}