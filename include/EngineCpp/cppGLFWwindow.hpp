#pragma once

#include <string>

class GLFWwindow;

class cppGLFWwindow
{
public:
	cppGLFWwindow(int width, int height, const std::string& title);
	~cppGLFWwindow();

	GLFWwindow* GetHandle();
private:
	GLFWwindow* m_window = nullptr;
};