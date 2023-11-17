#pragma once

#include "Geometry2D.h"
#include <vector>

using std::vector;

struct QuadTreeData
{
	void* object;
	Rectangle2D bounds;
	bool flag;

	inline QuadTreeData(void* o, const Rectangle2D& b) :
		object(o), bounds(b), flag(false) { }
};

class QuadTreeNode
{
protected:
	vector<QuadTreeNode> children;
	vector<QuadTreeData*> contents;
	int currentDepth;
	static int maxDepth;
	static int maxObjectsPerNode;
	Rectangle2D nodeBounds;

public:
	inline QuadTreeNode(const Rectangle2D& bounds) :
		nodeBounds(bounds), currentDepth(0) { }
	bool IsLeaf();
	int NumObjects();
	void Insert(QuadTreeData& data);
	void Remove(QuadTreeData& data);
	void Update(QuadTreeData& data);
	void Shake();
	void Split();
	void Reset();
	vector<QuadTreeData*>Query(const Rectangle2D& area);
};

typedef QuadTreeNode QuadTree;