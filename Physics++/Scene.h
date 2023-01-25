#pragma once

#ifndef _H_SCENE_
#define _H_SCENE_

#include "Geometry3D.h"
#include <vector>

#endif 


class Scene
{
protected:
	std::vector<Model*> objects;

public:
	void AddModel(Model* model);
	void RemoveModel(Model* model);
	void UpdateModel(Model* model);
	std::vector<Model*> FindChildren(const Model* model);
};