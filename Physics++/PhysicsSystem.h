#pragma once

#include "Rigidbody.h"

class PhysicsSystem
{
public:
	void Update(float deltaTime);
	void Render();
	void AddRigidbody(Rigidbody* body);
	void AddConstraint(const OBB& constraint);
	void ClearRigidbodies();
	void ClearConstraints();

protected:
	std::vector<Rigidbody*> bodies;
	std::vector<OBB> constraints;
}; 