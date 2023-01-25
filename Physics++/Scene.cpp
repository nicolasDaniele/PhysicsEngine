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