#include "Camera.h"
#include <cmath>
#include <cfloat>

Camera::Camera()
{
	m_nFOV = 60.0f;
	m_nAspect = 1.3f;
	m_nNear = 0.01f;
	m_nFar = 1000.0f;
	m_nWidth = 1.0f;
	m_nHeight = 1.0f;

	m_matWorld = Mat4();
	m_matProj = Projection(m_nFOV, m_nAspect, m_nNear, m_nFar);
	m_nProjectionMode = 0;
}

Mat4 Camera::GetWorldMatrix()
{
	return m_matWorld;
}

Mat4 Camera::GetViewMatrix()
{
	if (!IsOrthoNormal())
	{
		OrthoNormalize();
	}

	Mat4 inverse = Transpose(m_matWorld);
	inverse._41 = inverse._14 = 0.0f;
	inverse._42 = inverse._24 = 0.0f;
	inverse._43 = inverse._34 = 0.0f;

	Vec3 right = Vec3(m_matWorld._11, m_matWorld._12, m_matWorld._13);
	Vec3 up = Vec3(m_matWorld._21, m_matWorld._22, m_matWorld._23);
	Vec3 forward = Vec3(m_matWorld._31, m_matWorld._32, m_matWorld._33);
	Vec3 position = Vec3(m_matWorld._41, m_matWorld._42, m_matWorld._43);

	inverse._41 = -Dot(right, position);
	inverse._42 = -Dot(up, position);
	inverse._43 = -Dot(forward, position);

	return inverse;
}

Mat4 Camera::GetProjectionMatrix()
{
	return m_matProj;
}

float Camera::GetAstpect()
{
	return m_nAspect;
}

bool Camera::IsOrthographic()
{
	return m_nProjectionMode == 1;
}

bool Camera::IsPerspective()
{
	return m_nProjectionMode == 0;
}

bool Camera::IsOrthoNormal()
{
	Vec3 right = Vec3(m_matWorld._11, m_matWorld._12, m_matWorld._13);
	Vec3 up = Vec3(m_matWorld._21, m_matWorld._22, m_matWorld._23);
	Vec3 forward = Vec3(m_matWorld._31, m_matWorld._32, m_matWorld._33);

	if (!CMP(Dot(right, right), 1.0f) || 
		!CMP(Dot(up, up), 1.0f) ||
		!CMP(Dot(forward, forward), 1.0f))
	{
		return false;
	}

	if (!CMP(Dot(right, right), 0.0f) ||
		!CMP(Dot(up, up), 0.0f) ||
		!CMP(Dot(forward, forward), 0.0f))
	{
		return false;
	}

	return true;
}

void Camera::OrthoNormalize()
{
	Vec3 right = Vec3(m_matWorld._11, m_matWorld._12, m_matWorld._13);
	Vec3 up = Vec3(m_matWorld._21, m_matWorld._22, m_matWorld._23);
	Vec3 forward = Vec3(m_matWorld._31, m_matWorld._32, m_matWorld._33);

	Vec3 f = Normalized(forward);
	Vec3 r = Normalized(Cross(up, f));
	Vec3 u = Cross(f, r);

	m_matWorld = Mat4(
		r.x, r.y, r.z, 0.0f,
		u.x, u.y, u.z, 0.0f,
		f.x, f.y, f.z, 0.0f,
		m_matWorld._41, m_matWorld._42, m_matWorld._43, 1.0f
		);
}

void Camera::Resize(int width, int height)
{
	m_nAspect = (float)width / (float)height;

	if (m_nProjectionMode == 0)
	{
		m_matProj = Projection(m_nFOV, m_nAspect, m_nNear, m_nFar);
	}
	else if (m_nProjectionMode == 1)
	{
		m_nWidth = (float)width;
		m_nHeight = (float)height;
		float halfW = m_nWidth * 0.5f;
		float halfH = m_nHeight * 0.5f;
		m_matProj = Ortho(-halfW, halfW, halfH, -halfH, m_nNear, m_nFar);
	}
}

void Camera::Perspective(float fov, float aspect, float zNear, float zFar)
{
	m_nFOV = fov;
	m_nAspect = aspect;
	m_nNear = zNear;
	m_nFar = zFar;

	m_matProj = Projection(m_nFOV, m_nAspect, m_nNear, m_nFar);
	m_nProjectionMode = 0;
}

void Camera::Orthographic(float width, float height, float zNear, float zFar)
{
	m_nWidth = width;
	m_nHeight = height;
	m_nNear = zNear;
	m_nFar = zFar;

	float halfW = m_nWidth * 0.5f;
	float halfH = m_nHeight * 0.5f;
	m_matProj = Ortho(-halfW, halfW, halfH, -halfH, m_nNear, m_nFar);

	m_nProjectionMode = 1;
}

void Camera::SetProjection(const Mat4& projection)
{
	m_matProj = projection;
	m_nProjectionMode = 2;
}

void Camera::SetWorld(const Mat4& world)
{
	m_matWorld = world;
}

Frustum Camera::GetFrustum()
{
	Frustum result;

	Mat4 vp = GetViewMatrix() * GetProjectionMatrix();
	Vec3 col1(vp._11, vp._21, vp._31);
	Vec3 col2(vp._12, vp._22, vp._32);
	Vec3 col3(vp._13, vp._23, vp._33); 
	Vec3 col4(vp._14, vp._24, vp._34);

	result.left.normal   = col4 + col1;
	result.right.normal  = col4 - col1;
	result.bottom.normal = col4 + col2;
	result.top.normal    = col4 - col2;
	result.near.normal   = col3;
	result.far.normal    = col4 - col3;

	result.left.distance   = vp._44 + vp._41;
	result.right.distance  = vp._44 - vp._41;
	result.bottom.distance = vp._44 + vp._42;
	result.top.distance    = vp._44 - vp._42;
	result.near.distance   = vp._43;
	result.far.distance    = vp._44 - vp._43;

	for (int i = 0; i < 6; ++i)
	{
		float mag = 1.0f / Magnitude(result.planes[i].normal);
		result.planes[i].normal = result.planes[i].normal * mag;
		result.planes[i].distance *= mag;
	}

	return result;
}

OrbitCamera::OrbitCamera()
{
	target = Vec3(0, 0, 0);
	zoomDistance = 10.0f;
	zoomSpeed = 200.0f;
	rotationSpeed = Vec2(250.0f, 120.0f);
	yRotationLimit = Vec2(-20.f, 80.0f);
	zoomDistanceLimit = Vec2(3.0f, 15.0f);
	currentRotation = Vec2(0, 0);
	panSpeed = Vec2(180.0f, 180.0f);
}

void OrbitCamera::Rotate(const Vec2& deltaRotation, float deltaTime)
{
	currentRotation.x += deltaRotation.x * rotationSpeed.x * zoomDistance * deltaTime;
	currentRotation.y += deltaRotation.y * rotationSpeed.y * zoomDistance * deltaTime;

	currentRotation.x = ClampAngle(currentRotation.x, -360, 360);
	currentRotation.y = ClampAngle(currentRotation.y, yRotationLimit.x, yRotationLimit.y);
}

void OrbitCamera::Zoom(float deltaZoom, float deltaTime)
{
	zoomDistance = zoomDistance + deltaZoom * zoomSpeed * deltaTime;

	if (zoomDistance < zoomDistanceLimit.x)
	{
		zoomDistance = zoomDistanceLimit.x;
	}
	if (zoomDistance > zoomDistanceLimit.y)
	{
		zoomDistance = zoomDistanceLimit.y;
	}
}

void OrbitCamera::Pan(const Vec2& deltaPan, float deltaTime)
{
	Vec3 right(m_matWorld._11, m_matWorld._12, m_matWorld._13);
	float xPanMag = deltaPan.x * panSpeed.x * deltaTime;
	target = target - (right * xPanMag);

	float yPanMag = deltaPan.y * panSpeed.y * deltaTime;
	target = target + (Vec3(0, 1, 0) * yPanMag);
}

void OrbitCamera::Update(float deltaTime)
{
	Vec3 rotation = Vec3(currentRotation.y, currentRotation.x, 0);
	Mat3 orient = Rotation3x3(rotation.x, rotation.y, rotation.z);
	Vec3 direction = MultiplyVector(Vec3(0.0f, 0.0f, -zoomDistance), orient);
	Vec3 position = direction + target;

	m_matWorld = Inverse(LookAt(position, target, Vec3(0, 1, 0)));
}

float OrbitCamera::ClampAngle(float angle, float min, float max)
{
	while (angle < -360)
	{
		angle += 360;
	}

	while (angle > 360)
	{
		angle -= 360;
	}

	if (angle < min)
	{
		angle = min;
	}

	if (angle > max)
	{
		angle = max;
	}

	return angle;
}