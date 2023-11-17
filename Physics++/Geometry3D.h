#pragma once

#include "Vectors.h"
#include "Matrices.h"

typedef Vec3 Point;
#define AABBShpere(aabb, sphere)    SphereAABB(sphere, aabb)
#define OBBShpere(obb, sphere)      SphereOBB(sphere, obb)
#define PlaneSphere(plane, sphere) SpherePlane(sphere, plane)
#define OBBAABB(obb, aabb) AABBOBB(aabb, obb)
#define PlaneAABB(plane, aabb) AABBPlane(aabb, plane)
#define PlaneOBB(plane, obb) OBBPlane(obb, plane)
#define SphereTriangle(sphere, triangle) TriangleSphere(triangle, sphere)
#define AABBTriangle(aabb, triangle) TriangleAABB(triangle, aabb)
#define OBBTriangle(obb, triangle) TriangleOBB(triangle, obb)
#define PlaneTriangle(plane, triangle) TrianglePlane(triangle, plane)

struct Line 
{
	Point start;
	Point end;

	inline Line() { }
	inline Line(const Point& s, const Point& e) :
	start(s), end(e) { }
};

struct Ray
{
	Point origin;
	Vec3 direction;

	inline Ray() : direction(0.0f, 0.0f, 1.0f) { }
	inline Ray(const Point& o, const Vec3& d) :
		origin(o), direction(d)
	{
		NormalizeDirection();
	}
	inline void NormalizeDirection()
	{
		Normalize(direction);
	}
};

struct Sphere
{
	Point position;
	float radius;

	inline Sphere() : radius(1.0f) { }
	inline Sphere(const Point& p, float r) :
		position(p), radius(r) { }
};

struct AABB
{
	Point position;
	Vec3 size;

	inline AABB() : size(1, 1, 1) { }
	inline AABB(const Point& p, const Vec3& s) :
		position(p), size(s) { }
};

struct OBB
{
	Point position;
	Vec3 size;
	Mat3 orientation;

	inline OBB() : size(1, 1, 1) { }
	inline OBB(const Point& p, const Vec3& s) :
		position(p), size(s) { }
	inline OBB(const Point& p, const Vec3& s, const Mat3& o) :
		position(p), size(s), orientation(o) { }
};

struct Plane
{
	Vec3 normal;
	float distance;

	inline Plane() : normal(1, 0, 0) { }
	inline Plane(const Vec3& n, float d) :
		normal(n), distance(d) { }
};

typedef struct Triangle
{
	union
	{
		struct 
		{
			Point a;
			Point b;
			Point c;
		};

		Point points[3];
		float values[9];
	};

	inline Triangle() { }
	inline Triangle(const Point& p1, const Point& p2, const Point& p3) :
		a(p1), b(p2), c(p3) { }
} Triangle;

struct Interval
{
	float min;
	float max;
};

struct BVHNode
{
	AABB bounds;
	BVHNode* children;
	int numTriangles;
	int* triangles;
	BVHNode() : children(0), numTriangles(0), triangles(0) { }
};

typedef struct Mesh
{
	int numTriangles;

	union
	{
		Triangle* triangles;
		Point* vertices;
		float* values;
	};

	BVHNode* accelerator;
	Mesh() : numTriangles(0), values(0), accelerator(0) { }
} Mesh;

#undef near
#undef far

typedef struct Frustum
{
	union
	{
		struct
		{
			Plane top;
			Plane bottom;
			Plane left;
			Plane right;
			Plane near;
			Plane far;
		};
		Plane planes[6];
	};
	inline Frustum() { }
} Frustum;

struct RaycastResult
{
	Vec3 point;
	Vec3 normal;
	float t;
	bool hit;
};

class Model
{
protected:
	Mesh* content;
	AABB bounds;

public:
	Vec3 position;
	Vec3 rotation;
	Model* parent;

	inline Model() : parent(0) { }
	inline Mesh* GetMesh() const { return content; }
	inline AABB GetBounds() const { return bounds; }
	void SetContent(Mesh* mesh);
};

float Lenght(const Line& line);
float LenghtSq(const Line& line);

Ray FromPoints(const Point& from, const Point& to);

Vec3 GetMin(const AABB& aabb);
Vec3 GetMax(const AABB& aabb);
AABB FromMinMax(const Vec3& min, const Vec3& max);

float PlaneEquation(const Point& point, const Plane& plane);

bool PointInSphere(const Point& point, const Sphere& sphere);
Point ClosestPoint(const Sphere& sphere, const Point& point);

bool PointInAABB(const Point& point, const AABB& aabb);
Point ClosestPoint(const AABB& aabb, const Point& point);

bool PointInOBB(const Point& point, const OBB& obb);
Point ClosestPoint(const OBB& obb, const Point& point);

bool PointOnPlane(const Point& point, const Plane& plane);
Point ClosestPoint(const Plane& plane, const Point& point);

bool PointOnLine(const Point& point, const Line& line);
Point ClosestPoint(const Line& line, const Point& point);

bool PointOnRay(const Point& point, const Ray& ray);
Point ClosestPoint(const Ray& ray, const Point& point);

bool SphereShpere(const Sphere& sphere1, const Sphere& sphere2);
bool SphereAABB(const Sphere& sphere, const AABB& aabb);
bool SphereOBB(const Sphere& sphere, const OBB& obb);
bool SpherePlane(const Sphere& sphere, const Plane& plane);
bool AABBAABB(const AABB& aabb1, const AABB& aabb2);
bool AABBOBB(const AABB& aabb, const OBB& obb);
bool AABBPlane(const AABB& aabb, const Plane& plane);
bool OBBOBB(const OBB& obb1, const OBB& obb2);
bool OBBPlane(const OBB& obb, const Plane& plane);
bool PlanePlane(const Plane& plane1, const Plane& plane2);

bool OverlapOnAxis(const AABB& aabb, const OBB& obb, const Vec3& axis);
bool OverlapOnAxis(const OBB& obb1, const OBB& obb2, const Vec3& axis);

Interval GetInterval(const AABB& aabb, const Vec3& axis);
Interval GetInterval(const OBB& obb, const Vec3& axis);

bool Raycast(const Sphere& sphere, const Ray& ray, RaycastResult* outResult);
bool Raycast(const AABB& aabb, const Ray& ray, RaycastResult* outResult);
bool Raycast(const OBB& obb, const Ray& ray, RaycastResult* outResult);
bool Raycast(const Plane& plane, const Ray& ray, RaycastResult* outResult);
bool Raycast(const Triangle& triangle, const Ray& ray, RaycastResult* outResult);
void ResetRaycastResult(RaycastResult* outResult);

bool Linecast(const Sphere& sphere, const Line& line);
bool Linecast(const AABB& aabb, const Line& line);
bool Linecast(const OBB& obb, const Line& line);
bool Linecast(const Plane& plane, const Line& line);
bool Linecast(const Triangle& triangle, const Line& line);

bool PointInTriangle(const Point& point, const Triangle& triangle);
Plane FromTriangle(const Triangle& triangle);
Point ClosestPoint(const Triangle& triangle, const Point& point);
Interval GetInterval(const Triangle& triangle, const Vec3& axis);
bool OverlapOnAxis(const AABB& aabb, const Triangle& triangle, const Vec3& axis);
bool OverlapOnAxis(const OBB& obb, const Triangle& triangle, const Vec3& axis);
bool OverlapOnAxis(const Triangle& triangle1, const Triangle& triangle2, const Vec3& axis);
Vec3 SATCrossEdge(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d);
Vec3 Barycentric(const Point& point, const Triangle& triangle);
bool TriangleSphere(const Triangle& triangle, const Sphere& sphere);
bool TriangleAABB(const Triangle& triangle, const AABB& aabb);
bool TriangleOBB(const Triangle& triangle, const OBB& obb);
bool TrianglePlane(const Triangle& triangle, const Plane& plane);
bool TriangleTriangle(const Triangle& triangle1, const Triangle& triangle2);
bool TriangleTriangleRobust(const Triangle& triangle1, const Triangle& triangle2);

void AccelarateMesh(Mesh& mesh);
void SplitBVHNode(BVHNode* node, const Mesh& model, int depth);
void FreeBVHNode(BVHNode* node);
float MeshRay(const Mesh& mesh, const Ray& ray);
bool LineTest(const Mesh& mesh, const Line& line);
bool MeshSphere(const Mesh& mesh, const Sphere& sphere);
bool MeshAABB(const Mesh& mesh, const AABB& aabb);
bool MeshOBB(const Mesh& mesh, const OBB& obb);
bool MeshPlane(const Mesh& mesh, const Plane& plane);
bool MeshTriangle(const Mesh& mesh, const Triangle& triangle);

Mat4 GetWorldMatrix(const Model& model);
OBB GetOBB(const Model& model);
float ModelRay(const Model& model, const Ray& ray);
bool LineTest(const Model& model, const Line& line);
bool ModelSphere(const Model& model, const Sphere& sphere);
bool ModelAABB(const Model& model, const AABB& aabb);
bool ModelOBB(const Model& model, const OBB& obb);
bool ModelPlane(const Model& model, const Plane& plane);
bool ModelTriangle(const Model& model, const Triangle& triangle);

Point Intersection(Plane plane1, Plane plane2, Plane plane3);
void GetCorners(const Frustum& frustum, Vec3* outCorners);
bool Intersects(const Frustum& frustum, const Point& point);
bool Intersects(const Frustum& frustum, const Sphere& sphere);
float Classify(const AABB& aabb, const Plane& plane);
float Classify(const OBB& obb, const Plane& plane);
bool Intersects(const Frustum& frustum, const AABB& aabb);
bool Intersects(const Frustum& frustum, const OBB& obb);

Vec3 Uproject(const Vec3& viewportPoint, const Vec2& viewportOrigin,
	const Vec2& viewportSize, const Mat4& view, const Mat4& projection);
Ray GetPickRay(const Vec2& viewportPoint, const Vec2& viewportOrigin,
	const Vec2& viewportSize, const Mat4& view, const Mat4& projection);