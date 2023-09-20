#include <iostream>
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include "imgui.h"

#include "EngineCpp/cppGLFW.hpp"
#include "EngineCpp/cppGLFWwindow.hpp"
#include "EngineCpp/cppImgui.hpp"
#include "EulerIntegrator.hpp"
#include "Particle.hpp"

void FramebufferSizeCallback(GLFWwindow* window, int width, int height);

int main(int argc, char** argv)
{
    cppGLFW glfw;
    cppGLFWwindow window(800, 600, "Uqac physic engine");

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Error: Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glfwSetFramebufferSizeCallback(window.GetHandle(), FramebufferSizeCallback);

    EulerIntegrator integrator;
    Particle particle(Vector3<float>(0.0f, 0.0f, 0.0f), Vector3<float>(1.0f, 1.0f, 2.0f), Vector3<float>(1.0f, 2.0f, 3.0f), 0.000001f, "Particle");

    uint64_t lastFrameTime = glfwGetTime();

    // Imgui setup
    ImguiCpp imguiCpp(&window);
    while (!window.ShouldClose())
    {
        //uint64_t currentTime = glfwGetTime();
        //float deltaTime = (float)(currentTime - lastFrameTime) / glfwGetTimerFrequency();
        //lastFrameTime = currentTime;

        float deltaTime = 0.2f;

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        imguiCpp.NewFrame();

        ImGui::Begin("Hello, world!");
        ImGui::Text("Basic IMGUI text.");
        ImGui::Text("Particles");
        ImGui::Text("Particle name: %s", particle.name.c_str());
        ImGui::Text("Particle position: (%f, %f, %f)", particle.position.x, particle.position.y, particle.position.z);
        ImGui::Text("Particle velocity: (%f, %f, %f)", particle.velocity.x, particle.velocity.y, particle.velocity.z);
        ImGui::Text("Particle acceleration: (%f, %f, %f)", particle.acceleration.x, particle.acceleration.y, particle.acceleration.z);
        ImGui::Text("Particle mass: %f", particle.mass);
        ImGui::Text("Delta Time: %f", deltaTime);
        ImGui::End();


        imguiCpp.Render();

        glfwSwapBuffers(window.GetHandle());
        glfwPollEvents();

        integrator.Integrate(particle, deltaTime);
    }

    return 0;
}

void FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}