#include "Scene.h"
#include <algorithm>
#include <stack>

void Scene::AddModel(Model* model)
{
	if (std::find(objects.begin(), objects.end(), model) != objects.end())
	{
		return;
	}

	objects.push_back(model);
}

void Scene::RemoveModel(Model* model) 
{
	objects.erase(std::remove(objects.begin(), objects.end(), model), 
		objects.end());
}

void Scene::UpdateModel(Model* model)
{

}

std::vector<Model*> Scene::FindChildren(const Model* model)
{
	std::vector<Model*> result;

	for (int i = 0, size = objects.size(); i < size; ++i)
	{
		if (objects[i] == 0 || objects[i] == model)
		{
			continue;
		}

		Model* iterator = objects[i]->parent;
		if (iterator != 0)
		{
			if (iterator == model)
			{
				result.push_back(objects[i]);
				continue;
			}
			iterator = iterator->parent;
		}
	}

	return result;
}

Model* Scene::Raycast(const Ray& ray)
{
	Model* result = 0;
	float result_t = -1;

	for (int i = 0, size = objects.size(); i < size; ++i)
	{
		float t = ModelRay(*objects[i], ray);
		if (result == 0 && t >= 0)
		{
			result = objects[i];
			result_t = t;
		}
		else if (result != 0 && t < result_t)
		{
			result = objects[i];
			result_t = t;
		}
	}

	return result;
}

std::vector<Model*> Scene::Query(const Sphere& sphere) 
{
	std::vector<Model*> result;

	for (int i = 0, size = objects.size(); i < size; ++i)
	{
		OBB bounds = GetOBB(*objects[i]);
		
		if (SphereOBB(sphere, bounds))
		{
			result.push_back(objects[i]);
		}
	}

	return result;
}

std::vector<Model*> Scene::Query(const AABB& aabb)
{
	std::vector<Model*> result;

	for (int i = 0, size = objects.size(); i < size; ++i)
	{
		OBB bounds = GetOBB(*objects[i]);

		if (AABBOBB(aabb, bounds))
		{
			result.push_back(objects[i]);
		}
	}

	return result;
}