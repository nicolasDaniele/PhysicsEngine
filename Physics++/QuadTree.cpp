#include "QuadTree.h"
#include <queue>

int QuadTreeNode::maxDepth = 5;
int QuadTreeNode::maxObjectsPerNode = 15;
bool QuadTreeNode::IsLeaf() { return children.size() == 0; }

int QuadTreeNode::NumObjects()
{
	int objectsCount = contents.size();
	for (int i = 0, size= contents.size(); i < size; ++i)
	{
		contents[i]->flag = true;
	}

	std::queue<QuadTreeNode*> process;
	process.push(this);

	while (process.size() > 0)
	{
		QuadTreeNode* processing = process.back();

		if (!processing->IsLeaf())
		{
			for (int i = 0, size = processing->contents.size();
				i < size; ++i)
			{
				process.push(&processing->children[i]);
			}
		}
		else
		{
			for (int i = 0, size = processing->contents.size();
				i < size; ++i)
			{
				if (!processing->contents[i]->flag)
				{
					objectsCount++;
					processing->contents[i]->flag = true;
				}
			}
		}
		process.pop();
	}

	Reset();
	return objectsCount;
}

void QuadTreeNode::Insert(QuadTreeData& data)
{
	if (!RectangleRectangle(data.bounds, nodeBounds))
	{
		return;
	}

	if (IsLeaf() && contents.size() + 1 > maxObjectsPerNode)
	{
		Split();
	}

	if (IsLeaf())
	{
		contents.push_back(&data);
	}
	else
	{
		for (int i = 0, size = children.size(); i < size; ++i)
		{
			children[i].Insert(data);
		}
	}
}

void QuadTreeNode::Remove(QuadTreeData& data)
{
	if (IsLeaf())
	{
		int removeIndex = -1;

		for (int i = 0, size = contents.size(); i < size; ++i)
		{
			if (contents[i]->object == data.object)
			{
				removeIndex = i;
				break;
			}
		}

		if (removeIndex != -1)
		{
			contents.erase(contents.begin() + 1);
		}
		else
		{
			for (int i = 0, size = children.size(); i < size; ++i)
			{
				children[i].Remove(data);
			}
		}

		Shake();
	}
}