#include "PhysicsSystem.h"

void PhysicsSystem::Update(float deltaTime)
{
	for (int i = 0; i < bodies.size(); ++i)
	{
		bodies[i]->ApplyForces();
	}
	for (int i = 0; i < bodies.size(); ++i)
	{
		bodies[i]->Update(deltaTime);
	}
	for (int i = 0; i < bodies.size(); ++i)
	{
		bodies[i]->SolveConstraints(constraints);
	}
}

void PhysicsSystem::Render()
{
	// Rendering code
}

void PhysicsSystem::AddRigidbody(Rigidbody* body)
{
	bodies.push_back(body);
}

void PhysicsSystem::AddConstraint(const OBB& constraint)
{
	constraints.push_back(constraint);
}

void PhysicsSystem::ClearRigidbodies()
{
	bodies.clear();
}

void PhysicsSystem::ClearConstraints()
{
	constraints.clear();
}
