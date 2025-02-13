#include "Particle.h"

Particle::Particle()
{
	friction = 0.95f;
	gravity = Vec3(0.0f, -9.82f, 0.0f);
	mass = 1.0f;
	bounce = 0.7f;
}

void Particle::Update(float deltaTime)
{
	oldPosition = position;
	Vec3 acceleration = forces * (1.0f / mass);
	Vec3 velocity = position - oldPosition;
	
	oldPosition = position;
	float deltaSquare = deltaTime * deltaTime;
	position = position + (velocity * friction + forces * deltaSquare);
}

void Particle::Render()
{
	// Render code
}

void Particle::ApplyForces()
{
	forces = gravity;
}

void Particle::SolveConstraints(const std::vector<OBB>& constraints)
{
	for (int i = 0; i < constraints.size(); ++i)
	{
		Line traveled(oldPosition, position);

		if (Linecast(constraints[i], traveled))
		{
			Vec3 velocity = position - oldPosition;
			Vec3 direction = Normalized(velocity);
			Ray ray(oldPosition, direction);
			RaycastResult result;

			if (Raycast(constraints[i], ray, &result))
			{
				position = result.point + result.normal * 0.003f;

				Vec3 vn = result.normal * Dot(result.normal, velocity);
				Vec3 vt = velocity - vn;
				oldPosition = position - (vt - vn * bounce);

				break;
			}
		}
	}
}

void Particle::SetPosition(const Vec3& _position)
{
	position = _position;
}

void Particle::SetBounce(const float _bounce)
{
	bounce = _bounce;
}

Vec3 Particle::GetPosition() const
{
	return position;
}

float Particle::GetBounce() const
{
	return bounce;
}
