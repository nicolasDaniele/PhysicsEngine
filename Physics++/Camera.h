#pragma once

#include "Matrices.h"
#include "Geometry3D.h"

class Camera
{
public:
	Camera();
	inline virtual ~Camera() { }

	Mat4 GetWorldMatrix();
	Mat4 GetViewMatrix();
	Mat4 GetProjectionMatrix();
	void SetProjection(const Mat4& projection);
	void SetWorld(const Mat4& view);
	float GetAstpect();
	bool IsOrthographic();
	bool IsPerspective();
	bool IsOrthoNormal();
	void OrthoNormalize();
	void Resize(int width, int height);
	void Perspective(float fov, float aspect, float zNear, float zFar);
	void Orthographic(float width, float height, float zNear, float zFar);
	Frustum GetFrustum();

protected:
	float m_nFOV;
	float m_nAspect;
	float m_nNear;
	float m_nFar;
	float m_nWidth;
	float m_nHeight;
	Mat4 m_matWorld;
	Mat4 m_matProj;
	int  m_nProjectionMode;
};

class OrbitCamera : public Camera
{
protected:
	Vec3 target;
	Vec2 panSpeed;
	float zoomDistance;
	Vec2 zoomDistanceLimit;
	float zoomSpeed;
	Vec2 rotationSpeed;
	Vec2 yRotationLimit;
	Vec2 currentRotation;

	float ClampAngle(float angle, float min, float max);

public:
	OrbitCamera();
	inline virtual ~OrbitCamera() { }

	void Rotate(const Vec2& deltaRotation, float deltaTime);
	void Zoom(float deltaZoom, float deltaTime);
	void Pan(const Vec2& deltaPan, float deltaTime);
	void Update(float deltaTime);
};