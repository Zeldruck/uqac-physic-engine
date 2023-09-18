#pragma once

#include <string>

class GLFWwindow;

class cppGLFWwindow
{
public:
	cppGLFWwindow(int width, int height, const std::string& title);
	~cppGLFWwindow();

	cppGLFWwindow(const cppGLFWwindow&) = delete;
	cppGLFWwindow(cppGLFWwindow&& window) noexcept;

	cppGLFWwindow& operator=(const cppGLFWwindow&) = delete;
	cppGLFWwindow& operator=(cppGLFWwindow&& window) noexcept;

	GLFWwindow* GetHandle();
private:
	GLFWwindow* m_window = nullptr;
};