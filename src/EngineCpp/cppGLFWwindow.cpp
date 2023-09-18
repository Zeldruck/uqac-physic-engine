#include <iostream>
#include <string>
#include <GLFW/glfw3.h>
#include "EngineCpp/cppGLFWwindow.hpp"

cppGLFWwindow::cppGLFWwindow(int width, int height, const std::string& title)
{
	m_window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
	if (m_window == nullptr)
	{
		std::cout << "Error: Can't create GLFW window" << std::endl;
		glfwTerminate();
	}

	std::cout << "Window created" << std::endl;
	glfwMakeContextCurrent(m_window);
	std::cout << "Current context: " << m_window << std::endl;
}

cppGLFWwindow::~cppGLFWwindow()
{
	std::cout << "Current context nullptr" << std::endl;
	glfwMakeContextCurrent(nullptr);
	std::cout << "Destroy window" << std::endl;
	glfwDestroyWindow(m_window);
}

GLFWwindow* cppGLFWwindow::GetHandle()
{
	return m_window;
}