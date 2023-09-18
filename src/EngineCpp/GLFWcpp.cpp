#include "EngineCpp/GLFWcpp.hpp"
#include <GLFW/glfw3.h>

GLFWcpp::GLFWcpp()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
}

GLFWcpp::~GLFWcpp()
{
    glfwTerminate();
}