#pragma once
#include <string>

class Component
{
public:
	virtual ~Component() = default;
	std::string unique_id;
};