#pragma once
#include "Primitive.hpp"

class Sphere : Primitive
{
public:
	Sphere(float radius, unsigned int num);

	void Init(unsigned int vertexPositionID);
	void Clean();
	void Draw() override;

private:
	int lats, longs;
	bool isInited;
	unsigned int m_vao, m_vboVertex, m_vboIndex;
	int numsToDraw;
};