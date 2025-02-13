#pragma once

#include <vector>
#include "Geometry3D.h"

class Rigidbody
{
public:
	Rigidbody() { }
	virtual ~Rigidbody() { }
	virtual void Update(float deltaTime){ }
	virtual void Render() { }
	virtual void ApplyForces() { }
	virtual void SolveConstraints(const std::vector<OBB>& constraints) { }
};