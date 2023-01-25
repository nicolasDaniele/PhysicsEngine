#pragma once

#ifndef _H_SCENE_
#define _H_SCENE_

#include "Geometry3D.h"
#include <vector>

#endif 


typedef struct OctreeNode
{
	AABB bounds;
	OctreeNode* children;
	std::vector<Model*> models;

	inline OctreeNode() : children(0) { }
	inline ~OctreeNode()
	{
		if (children != 0)
		{
			delete[] children;
		}
	}
} OctreeNode;

class Scene
{
protected:
	std::vector<Model*> objects;

public:
	void AddModel(Model* model);
	void RemoveModel(Model* model);
	void UpdateModel(Model* model);
	std::vector<Model*> FindChildren(const Model* model);
	Model* Raycast(const Ray& ray);
	std::vector<Model*> Query(const Sphere& sphere);
	std::vector<Model*> Query(const AABB& aabb);
};