#pragma once

#include "Rigidbody.h"

class Particle : public Rigidbody
{
public:
	Particle();

	void Update(float deltaTime);
	void Render();
	void ApplyForces();
	void SolveConstraints(const std::vector<OBB>& constraints);

	void SetPosition(const Vec3& _position);
	void SetBounce(const float _bounce);
	Vec3 GetPosition() const;
	float GetBounce() const;


private:
	Vec3 position, oldPosition;
	Vec3 forces, gravity;
	float mass, bounce, friction;
};