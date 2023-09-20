#include <iostream>
#include "Vector3.hpp"
#include <glad/glad.h>
#include "./EngineCpp/ImguiCpp.hpp"
#include "imgui.h"
#include <GLFW/glfw3.h>

#include "EngineCpp/cppGLFW.hpp"
#include "EngineCpp/cppGLFWwindow.hpp"

using namespace std;

void FramebufferSizeCallback(GLFWwindow* window, int width, int height);

int main(int argc, char** argv)
{
    cppGLFW glfw;
    cppGLFWwindow window(800, 600, "Uqac physic engine");

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Error: Failed to initialize GLAD" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwSetFramebufferSizeCallback(window.GetHandle(), FramebufferSizeCallback);

    // Imgui setup
    ImguiCpp imguiCpp(window);

    while (!glfwWindowShouldClose(window))
    {
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        imguiCpp.NewFrame();

        ImGui::Begin("Hello, world!");
        ImGui::Text("Basic IMGUI text.");
        ImGui::End();


        imguiCpp.Render();

        glfwSwapBuffers(window.GetHandle());
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

void FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}