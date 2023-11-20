#include <iostream>
#include <algorithm>
#include <memory>
#include <chrono>

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include "imgui.h"

#include "EngineCpp/cppGLFW.hpp"
#include "EngineCpp/cppGLFWwindow.hpp"
#include "EngineCpp/cppImgui.hpp"
#include "EngineCpp/cppShader.hpp"

#include "Constants/PhysicConstants.hpp"
#include "Constants/MathConstants.hpp"

#include "PhysicsSystem.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"

#include"Contact/ParticleContactResolver.hpp"
#include "Contact/ParticleContact.hpp"
#include "Contact/ParticleContactGenerator.hpp"
#include "Contact/ParticleLink.hpp"
#include "Contact/ParticleCable.hpp"
#include "Contact/ParticleRod.hpp"

#include "Force/ForceRegistry.hpp"
#include "Force/ForceGenerator.hpp"
#include "Force/ForceGravity.hpp"
#include "Force/ForceDrag.hpp"
#include "Force/ForceSpring.hpp"
#include "Force/ForceAnchoredSpring.hpp"
#include "Force/ForceBuoyancy.hpp"

#include "Camera.hpp"

#include "Vector3.hpp"
#include "Transform.hpp"
#include "Quaternion.hpp"

#include "Primitives/Sphere.hpp"

void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
void MouseCallback(GLFWwindow* window, double xpos, double ypos);
void ProcessInput(GLFWwindow* window, float deltaTime);

void ContactsGenerator(std::vector<std::shared_ptr<ParticleContact>>& contactArray, std::vector<std::shared_ptr<ParticleContactGenerator>>& contacts);

void ImguiGamePanel(std::shared_ptr<Particle> particle, PhysicsSystem& physics, Vector3f& direction, float& power, bool& isParticleLaunched, bool& isGravityEnabled, float deltaTime);
void ImguiStatsPanel(float deltaTime);
void Vector3ClassTest();
double HiresTimeInSeconds();
void ImguiSceneSelection();
void ImguiRigidbodyData(std::shared_ptr<Rigidbody> rigidbody);

enum class Scene
{
    SCENE_1,
    SCENE_2
};

// Current scene
Scene currentScene = Scene::SCENE_1;

// settings
const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 20.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

int main(int argc, char** argv)
{
    using clock = std::chrono::high_resolution_clock;

    cppGLFW glfw;
    cppGLFWwindow window(SCR_WIDTH, SCR_HEIGHT, "Uqac physic engine");

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Error: Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glfwSetFramebufferSizeCallback(window.GetHandle(), FramebufferSizeCallback);
    glfwSetCursorPosCallback(window.GetHandle(), MouseCallback);

    glEnable(GL_DEPTH_TEST);

    ImguiCpp imguiCpp(&window);

#pragma region  Physics System
    std::shared_ptr<ForceRegistry> forceRegistry = std::make_shared<ForceRegistry>();
    PhysicsSystem physics(forceRegistry);
#pragma endregion

#pragma region Rigidbody
    std::shared_ptr<Rigidbody> rigidbodyBox = std::make_shared<Rigidbody>(Transform(Vector3f(0.f, 5.f, 0.f), Quaternionf(1.f, 0.f, 45.f * Deg2Rad, 0.f), Vector3f::One), Vector3f::Zero, Vector3f::Zero, 10.f, Vector3f::Zero, Vector3f::Zero, Vector3f::Zero, "Cube", RigidbodyType::BOX, std::vector<Particle>(), 0.0f, 0.9f);
    physics.AddRigidbody(rigidbodyBox);

    std::shared_ptr<Rigidbody> rigidbodyTriangle = std::make_shared<Rigidbody>(Transform(Vector3f(0.f, 5.f, 0.f), Quaternionf(1.f, 0.f, 0.f, 0.f), Vector3f::One), Vector3f::Zero, Vector3f::Zero, 10.f, Vector3f::Zero, Vector3f::Zero, Vector3f::Zero, "Triangle", RigidbodyType::TRIANGLE);
    physics.AddRigidbody(rigidbodyTriangle);

    std::shared_ptr<ForceGravity> forceGravity = std::make_shared<ForceGravity>();
    std::shared_ptr<ForceDrag> weakForceDrag = std::make_shared<ForceDrag>(5.0f, 0.0f);
    std::shared_ptr<ForceDrag> strongForceDrag = std::make_shared<ForceDrag>(20.0f, 0.0f);

    std::shared_ptr<ForceAnchoredSpring> forceAnchoredSpringCube = std::make_shared<ForceAnchoredSpring>(100.f, 5.0f, Vector3f::Zero, Vector3f(0.0f, 1.0f, 1.0f));
    std::shared_ptr<ForceAnchoredSpring> forceAnchoredSpringTriangle = std::make_shared<ForceAnchoredSpring>(10.f, 5.0f, Vector3f::Zero, Vector3f(0.5f, 0.5f, 0.5f));

    forceRegistry->Add(rigidbodyBox, forceGravity);
    forceRegistry->Add(rigidbodyBox, strongForceDrag);
    forceRegistry->Add(rigidbodyBox, forceAnchoredSpringCube);

    forceRegistry->Add(rigidbodyTriangle, forceGravity);
    forceRegistry->Add(rigidbodyTriangle, weakForceDrag);
    forceRegistry->Add(rigidbodyTriangle, forceAnchoredSpringTriangle);
#pragma endregion

#pragma region Timestep
    double t = 0.0;
    double deltaTime = 0.01;

    double currentTime = HiresTimeInSeconds();
    double accumulator = 0.0;
#pragma endregion

#pragma region Shader

    Shader ourShader("assets/shaders/test.vert", "assets/shaders/test.frag"); // To fix, so we can use real files for vert and frag shaders

#pragma endregion

#pragma region Model

    // Cube
    float cubeVertices[] = {
    -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
     0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
     0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
     0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
    -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
     0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
     0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
     0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
    -0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

    -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
    -0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
    -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

     0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
     0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
     0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
     0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
     0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
     0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
     0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
     0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
     0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
     0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
     0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
     0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
    -0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f
    };

    glm::vec3 cubePosition = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 cubeRotation = glm::vec3(0.0f, 0.0f, 0.0f);

    GLuint VBO1, VAO1;
    glGenVertexArrays(1, &VAO1);
    glGenBuffers(1, &VBO1);

    glBindVertexArray(VAO1);

    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);

    // vertices for a triangle pyramidal from :
    // https://stackoverflow.com/questions/22714866/to-draw-tetrahedron-using-opengl-in-android-with-different-color-faces-but-gray
    float triangleVertices[] = {

        // FRONT Face
        -1.0f, -1.0f,  1.0f,
        1.0f, -1.0f,  1.0f,
        1.0f,  1.0f,  1.0f,

        //Right face
        -1.0f, -1.0f,  1.0f,
        1.0f,  1.0f, 1.0f,
        -1.0f, -1.0f, -1.0f,

        // Left Face
        1.0, -1.0f, 1.0f,
        -1.0f, -1.0f, -1.0f,
        1.0f,  1.0f, 1.0f,

        // BOTTOM
        -1.0f, -1.0f, 1.0f,
        -1.0f, -1.0f, -1.0f,
        1.0f, -1.0f,  1.0f
    };

    glm::vec3 trianglePosition = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 triangleRotation = glm::vec3(0.0f, 0.0f, 0.0f);

    // Create a VAO and VBO for Model 2
    GLuint VAO2, VBO2;
    glGenVertexArrays(1, &VAO2);
    glGenBuffers(1, &VBO2);

    glBindVertexArray(VAO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, sizeof(triangleVertices), triangleVertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

#pragma endregion

    bool isGravityEnabled = true;
    while (!window.ShouldClose())
    {
        double newTime = HiresTimeInSeconds();
        double frameTime = newTime - currentTime;

        // Cap the frame time to avoid spiral of death
        if (frameTime > 0.015)
            frameTime = 0.015;

        currentTime = newTime;
        accumulator += frameTime;

        while (accumulator >= deltaTime) {
            // Integrates the physics
            physics.Update(deltaTime, isGravityEnabled);
            t += deltaTime;
            accumulator -= deltaTime;
        }

        // Use alpha with rneder to interpolate between the previous and current physics state
        const double alpha = accumulator / deltaTime;
        // state = currentState * alpha + previousState * (1.0 - alpha);

        ProcessInput(window.GetHandle(), deltaTime);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ourShader.Use();

        glm::mat4 projection = glm::perspective(glm::radians(45.f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.SetMat4("projection", projection);

        glm::mat4 view = camera.GetViewMatrix();
        ourShader.SetMat4("view", view);

        cubePosition = glm::vec3(rigidbodyBox->transform.position.x, rigidbodyBox->transform.position.y, rigidbodyBox->transform.position.z);
        cubeRotation = glm::vec3(rigidbodyBox->transform.rotation.GetX(), rigidbodyBox->transform.rotation.GetY(), rigidbodyBox->transform.rotation.GetZ());
        
        trianglePosition = glm::vec3(rigidbodyTriangle->transform.position.x, rigidbodyTriangle->transform.position.y, rigidbodyTriangle->transform.position.z);
        triangleRotation = glm::vec3(rigidbodyTriangle->transform.rotation.GetX(), rigidbodyTriangle->transform.rotation.GetY(), rigidbodyTriangle->transform.rotation.GetZ());

        glm::mat4 model = glm::mat4(1.0f);
        switch (currentScene)
        {
        case Scene::SCENE_1:
            glBindVertexArray(VAO1);
            model = glm::translate(model, cubePosition);
            // rotation on X Axis
            model = glm::rotate(model, cubeRotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
            // rotation on Y Axis
            model = glm::rotate(model, cubeRotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
            // rotation on Z Axis
            model = glm::rotate(model, cubeRotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
            glDrawArrays(GL_TRIANGLES, 0, 36);

            //ImguiRigidbodyData(rigidbodyBox);
            break;

        case Scene::SCENE_2:
            glBindVertexArray(VAO2);
            model = glm::translate(model, trianglePosition);
            // rotation on X Axis
            model = glm::rotate(model, triangleRotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
            // rotation on Y Axis
            model = glm::rotate(model, triangleRotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
            // rotation on Z Axis
            model = glm::rotate(model, triangleRotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
            glDrawArrays(GL_TRIANGLES, 0, 12);
            
            //ImguiRigidbodyData(rigidbodyTriangle);
            break;
        }
        ourShader.SetMat4("model", model);

        imguiCpp.NewFrame();

        ImguiStatsPanel(deltaTime);
        ImguiSceneSelection();
        ImguiRigidbodyData(rigidbodyBox);
        ImguiRigidbodyData(rigidbodyTriangle);

        imguiCpp.Render();

        glfwSwapBuffers(window.GetHandle());
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO1);
    glDeleteBuffers(1, &VBO1);
    glDeleteVertexArrays(1, &VAO2);
    glDeleteBuffers(1, &VBO2);

    return 0;
}

void FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void ImguiGamePanel(std::shared_ptr<Particle> particle, PhysicsSystem& physics, Vector3f& direction, float& power, bool& isParticleLaunched, bool& isGravityEnabled, float deltaTime)
{
    float directionAngle = 0.f;

    ImGui::Begin("Game panel");
    ImGui::Text("Launch a particle");

    ImGui::BeginGroup();
    ImGui::SliderFloat("Dir X", &direction.x, 0.f, 1.f);
    ImGui::SliderFloat("Dir Y", &direction.y, 0.f, 1.f);
    ImGui::EndGroup();

    ImGui::Spacing();
    ImGui::SliderFloat("Power", &power, 0.f, 8.f);
    ImGui::Spacing();

    ImGui::Spacing();
    ImGui::Checkbox("Gravity", &isGravityEnabled);
    ImGui::Spacing();

    ImGui::BeginGroup();
    if (!isParticleLaunched && ImGui::Button("Launch"))
    {
        std::cout << "Launch the particle" << std::endl;

        particle->velocity = direction.GetUnitNormalized() * power;

        physics.AddParticle(particle);

        isParticleLaunched = true;
    }
    if (isParticleLaunched && ImGui::Button("Reset"))
    {
        std::cout << "Reset the particle" << std::endl;

        physics.RemoveParticle(particle);
        particle->position = Vector3f(0, 0, 0);
        particle->velocity = Vector3f(0, 0, 0);

        isParticleLaunched = false;
    }
    ImGui::EndGroup();
    ImGui::End();
}

void ImguiStatsPanel(float deltaTime)
{
    ImGui::Begin("Stats Panel");
    ImGui::Text("Delta Time: %f", deltaTime);
    ImGui::Text("FPS: %.f", std::clamp(1000 / (deltaTime * 1000), 0.f, 60.f));
    ImGui::End();
}

void Vector3ClassTest()
{
    /* Vector3 class test */
    Vector3f testVec(1.f, 1.f, 0.f);
    std::cout << "TestVec: " << testVec << std::endl;
    std::cout << "TestVec length: " << testVec.GetLength() << std::endl;
    std::cout << "TestVec normalized: " << testVec.GetNormalized() << std::endl;
    std::cout << "TestVec unit normalized: " << testVec.GetUnitNormalized() << std::endl;
    std::cout << std::endl;

    Vector3f t(1, 2, 3);
    Vector3f t2(4, 5, 6);

    std::cout << "Cross product: " << t << ";" << t2 << " = " << Vector3f::CrossProduct(t, t2) << std::endl;
    std::cout << "Dot product: " << t << ";" << t2 << " = " << Vector3f::DotProduct(t, t2) << std::endl;
    std::cout << std::endl;
}

double HiresTimeInSeconds() {
    using namespace std::chrono;
    using clock = high_resolution_clock;

    auto currentTime = duration_cast<milliseconds>(clock::now().time_since_epoch()).count();
    return currentTime / 1000.0;
}

void ProcessInput(GLFWwindow* window, float deltaTime)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

void MouseCallback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

void ContactsGenerator(std::vector<std::shared_ptr<ParticleContact>>& contactArray, std::vector<std::shared_ptr<ParticleContactGenerator>>& contacts)
{
    if(contacts.size() == 0)
		return;

    for (int i = 0; i < contacts.size(); i++)
    {
        contacts.at(i)->AddContact(contactArray, 0);
    }
}

void ImguiSceneSelection() {
    // ImGui code for scene selection
    ImGui::Begin("Scene Selection");
    ImGui::Text("Current Scene: %d", (int)currentScene + 1);
    if (ImGui::Button("Scene 1"))
        currentScene = Scene::SCENE_1;
    if (ImGui::Button("Scene 2"))
        currentScene = Scene::SCENE_2;
    ImGui::End();
}

void ImguiRigidbodyData(std::shared_ptr<Rigidbody> rigidbody)
{
    ImGui::Begin("Rigidbody Data");
    ImGui::Text("%s", rigidbody->name.c_str());
    ImGui::Text("%s position: (%f, %f, %f)", rigidbody->name.c_str(), rigidbody->transform.position.x, rigidbody->transform.position.y, rigidbody->transform.position.z);
    ImGui::Text("%s velocity: (%f, %f, %f)", rigidbody->name.c_str(), rigidbody->velocity.x, rigidbody->velocity.y, rigidbody->velocity.z);
    ImGui::Text("%s acceleration: (%f, %f, %f)", rigidbody->name.c_str(), rigidbody->GetAcceleration().x, rigidbody->GetAcceleration().y, rigidbody->GetAcceleration().z);
    ImGui::Text("%s mass: %f", rigidbody->name.c_str(), rigidbody->mass);
    ImGui::Text("%s rotation (rad): (%f, %f, %f) %f", rigidbody->name.c_str(), rigidbody->transform.rotation.GetX(), rigidbody->transform.rotation.GetY(), rigidbody->transform.rotation.GetZ(), rigidbody->transform.rotation.GetS());
    ImGui::Text("%s angular velocity: (%f, %f, %f)", rigidbody->name.c_str(), rigidbody->angularVelocity.x, rigidbody->angularVelocity.y, rigidbody->angularVelocity.z);
    ImGui::Text("%s angular acceleration: (%f, %f, %f)", rigidbody->name.c_str(), rigidbody->GetAngularAcceleration().x, rigidbody->GetAngularAcceleration().y, rigidbody->GetAngularAcceleration().z);
    ImGui::Text("%s inverseMass: %f", rigidbody->name.c_str(), rigidbody->inverseMass);
    ImGui::Text("%s transformMatrix:\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f",
        rigidbody->name.c_str(),
        rigidbody->transformMatrix.Value(0, 0), rigidbody->transformMatrix.Value(0, 1), rigidbody->transformMatrix.Value(0, 2), rigidbody->transformMatrix.Value(0, 3),
        rigidbody->transformMatrix.Value(1, 0), rigidbody->transformMatrix.Value(1, 1), rigidbody->transformMatrix.Value(1, 2), rigidbody->transformMatrix.Value(1, 3),
        rigidbody->transformMatrix.Value(2, 0), rigidbody->transformMatrix.Value(2, 1), rigidbody->transformMatrix.Value(2, 2), rigidbody->transformMatrix.Value(2, 3),
        rigidbody->transformMatrix.Value(3, 0), rigidbody->transformMatrix.Value(3, 1), rigidbody->transformMatrix.Value(3, 2), rigidbody->transformMatrix.Value(3, 3));
    ImGui::End();
}