#include "EngineCpp/cppGLFW.hpp"
#include <GLFW/glfw3.h>
#include <iostream>

cppGLFW::cppGLFW()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    std::cout << "GLFW initialized" << std::endl;
}

cppGLFW::~cppGLFW()
{
    std::cout << "Terminating GLFW" << std::endl;
    glfwTerminate();
}