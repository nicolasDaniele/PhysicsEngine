#include "Camera.h"
#include <cmath>
#include <cfloat>

#define CMP(x, y) (fabsf((x)-(y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

Camera::Camera()
{
	m_nFOV = 60.0f;
	m_nAspect = 1.3f;
	m_nNear = 0.01f;
	m_nFar = 1000.0f;
	m_nWidth = 1.0f;
	m_nHeight = 1.0f;

	m_matWorld = mat4();
	m_matProj = Projection(m_nFOV, m_nAspect, m_nNear, m_nFar);
	m_nProjectionMode = 0;
}

mat4 Camera::GetWorldMatrix()
{
	return m_matWorld;
}

mat4 Camera::GetViewMatrix()
{
	if (!IsOrthoNormal)
	{
		OrthoNormalize();
	}

	mat4 inverse = Transpose(m_matWorld);
	inverse._41 = inverse._14 = 0.0f;
	inverse._42 = inverse._24 = 0.0f;
	inverse._43 = inverse._34 = 0.0f;

	vec3 right = vec3(m_matWorld._11, m_matWorld._12, m_matWorld._13);
	vec3 up = vec3(m_matWorld._21, m_matWorld._22, m_matWorld._23);
	vec3 forward = vec3(m_matWorld._31, m_matWorld._32, m_matWorld._33);
	vec3 position = vec3(m_matWorld._41, m_matWorld._42, m_matWorld._43);

	inverse._41 = -Dot(right, position);
	inverse._42 = -Dot(up, position);
	inverse._43 = -Dot(forward, position);

	return inverse;
}

mat4 Camera::GetProjectionMatrix()
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
	vec3 right = vec3(m_matWorld._11, m_matWorld._12, m_matWorld._13);
	vec3 up = vec3(m_matWorld._21, m_matWorld._22, m_matWorld._23);
	vec3 forward = vec3(m_matWorld._31, m_matWorld._32, m_matWorld._33);

	if (!CMP(Dot(right, right), 1.0f) || 
		!CMP(Dot(up, up), 1.0f) ||
		!CMP(Dot(forward, forward), 1.0f))
	{
		return false; // Axis are not normal length
	}

	if (!CMP(Dot(right, right), 0.0f) ||
		!CMP(Dot(up, up), 0.0f) ||
		!CMP(Dot(forward, forward), 0.0f))
	{
		return false; // Axis are not perpendicular
	}

	return true;
}

void Camera::OrthoNormalize()
{
	vec3 right = vec3(m_matWorld._11, m_matWorld._12, m_matWorld._13);
	vec3 up = vec3(m_matWorld._21, m_matWorld._22, m_matWorld._23);
	vec3 forward = vec3(m_matWorld._31, m_matWorld._32, m_matWorld._33);

	vec3 f = Normalized(forward);
	vec3 r = Normalized(Cross(up, f));
	vec3 u = Cross(f, r);

	m_matWorld = mat4(
		r.x, r.y, r.z, 0.0f,
		u.x, u.y, u.z, 0.0f,
		f.x, f.y, f.z, 0.0f,
		m_matWorld._41, m_matWorld._42, m_matWorld._43, 1.0f
		);
}

void Camera::Resize(int width, int height)
{
	m_nAspect = (float)width / (float)height;

	if (m_nProjectionMode == 0) // Perspective
	{
		m_matProj = Projection(m_nFOV, m_nAspect, m_nNear, m_nFar);
	}
	else if (m_nProjectionMode == 1) // Ortho
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

void Camera::SetProjection(const mat4& projection)
{
	m_matProj = projection;
	m_nProjectionMode = 2;
}

void Camera::SetWorld(const mat4& world)
{
	m_matWorld = world;
}