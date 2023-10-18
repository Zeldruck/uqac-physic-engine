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

#include "Force/ForceRegistry.hpp"
#include "Force/ForceGenerator.hpp"
#include "Force/ForceGravity.hpp"
#include "Force/ForceDrag.hpp"
#include "Force/ForceSpring.hpp"
#include "Force/ForceAnchoredSpring.hpp"
#include "Force/ForceBuoyancy.hpp"

#include "Camera.hpp"

#include "Vector3.hpp"

void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
void MouseCallback(GLFWwindow* window, double xpos, double ypos);
void ProcessInput(GLFWwindow* window, float deltaTime);

void ImguiGamePanel(std::shared_ptr<Particle> particle, PhysicsSystem& physics, Vector3f& direction, float& power, bool& isParticleLaunched, bool& isGravityEnabled, float deltaTime);
void ImguiStatsPanel(float deltaTime);
void Vector3ClassTest();
double HiresTimeInSeconds();

// settings
const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
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


    Vector3ClassTest();

    #pragma region Physics
    std::shared_ptr<ForceRegistry> forceRegistry = std::make_shared<ForceRegistry>();
    PhysicsSystem physics(forceRegistry);
    std::shared_ptr<Particle> particle(new Particle(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 2.0f), Vector3f(0.0f, 0.0f, 0.0f), 0.000001f, "Particle"));
    std::shared_ptr<Particle> particle3(new Particle(Vector3f(2.0f, 0.0f, 0.0f), Vector3f(1.0f, 0.0f, 1.0f), Vector3f(0.0f, 0.0f, 0.0f), 0.0001f, "Particle3Drag"));
    std::shared_ptr<Particle> particle2(new Particle(Vector3f(0.0f, 1.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), 1.f, "Particle2Spring2"));
    std::shared_ptr<Particle> particle4(new Particle(Vector3f(3.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), 1.f, "Particle4Spring1"));
    std::shared_ptr<Particle> particle5(new Particle(Vector3f(4.0f, 4.0f, 4.0f), Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), 1.f, "Particle5AnchoredSpring"));
    std::shared_ptr<Particle> particle6(new Particle(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), 0.0001f, "Particle6Buoyancy"));
    std::shared_ptr<Particle> particle7(new Particle(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), 0.0001f, "Particle7Gravity"));
    std::shared_ptr<Particle> particle8(new Particle(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, -10.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), 1.f, "Particle8Drag"));

    std::shared_ptr<ForceGravity> forceGravity = std::make_shared<ForceGravity>();
    
    float dragCoeff = 1.0f;
    float dragCoeff2 = 1.0f;
    std::shared_ptr<ForceDrag> forceDrag = std::make_shared<ForceDrag>(dragCoeff, dragCoeff2);
    std::shared_ptr<ForceDrag> forceDrag2 = std::make_shared<ForceDrag>(0.0f, 0.1f);

    float springConstant = 100.f;
    float restLength = (particle4->position - particle2->position).GetLength();
    std::shared_ptr<ForceSpring> forceSpring = std::make_shared<ForceSpring>(springConstant, restLength, particle2);
    
    float anchoredSpringConstant = 10.0f;
    std::shared_ptr<Particle> anchor(new Particle(Vector3f(.0f, 6.0f, .0f), Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), 1.f, "Anchor"));
    std::shared_ptr<ForceAnchoredSpring> forceAnchoredSpring = std::make_shared<ForceAnchoredSpring>(anchoredSpringConstant, restLength, anchor->position);
    
    std::shared_ptr<ForceBuoyancy> forceBuoyancy = std::make_shared<ForceBuoyancy>(1.f, 1.f, 1.f, 1.f);

    //physics.AddParticle(particle);
    physics.AddParticle(particle2);
    physics.AddParticle(particle3);
    physics.AddParticle(particle4);
    physics.AddParticle(particle5);
    physics.AddParticle(particle6);
    physics.AddParticle(particle7);
    physics.AddParticle(particle8);
    //forceRegistry->Add(particle, forceGravity);
    forceRegistry->Add(particle7, forceGravity);
    forceRegistry->Add(particle8, forceDrag);
    forceRegistry->Add(particle3, forceDrag2);
    forceRegistry->Add(particle4, forceSpring);
    forceRegistry->Add(particle5, forceAnchoredSpring);
    forceRegistry->Add(particle6, forceBuoyancy);

    forceRegistry->Add(particle4, forceGravity);
    forceRegistry->Add(particle5, forceGravity);
    //forceRegistry->Add(particle8, forceGravity);
    #pragma endregion

    // Game variables
    Vector3f direction(0.0f, 1.0f, 0.0f);
    float power = 4.f;
    bool isParticleLaunched = false;
    bool isGravityEnabled = true;
   
    #pragma region Timestep
    double t = 0.0;
    double deltaTime = 0.01;

    double currentTime = HiresTimeInSeconds();
    double accumulator = 0.0;
    #pragma endregion

    #pragma region Shader

    Shader ourShader("assets/shaders/test.vert", "assets/shaders/test.frag"); // To fix, so we can use real files for vert and frag shaders

    #pragma endregion

    #pragma region Triangle

    // Set up vertex data (and buffer(s)) and configure vertex attributes
    float vertices[] = {
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

    glm::vec3 cubePositions[] = {
    glm::vec3(0.0f,  0.0f,  0.0f),
    glm::vec3(2.0f,  5.0f, -15.0f),
    glm::vec3(-1.5f, -2.2f, -2.5f),
    glm::vec3(-3.8f, -2.0f, -12.3f),
    glm::vec3(2.4f, -0.4f, -3.5f),
    glm::vec3(-1.7f,  3.0f, -7.5f),
    glm::vec3(1.3f, -2.0f, -2.5f),
    glm::vec3(1.5f,  2.0f, -2.5f),
    glm::vec3(1.5f,  0.2f, -1.5f),
    glm::vec3(-1.3f,  1.0f, -1.5f)
    };

    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0);

    #pragma endregion
      
    // Imgui setup
    ImguiCpp imguiCpp(&window);

    // Game & window loop
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
        // activate shader
       
        // pass projection matrix to shader (note that in this case it could change every frame)
        glm::mat4 projection = glm::perspective(glm::radians(45.f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.SetMat4("projection", projection);

        // camera/view transformation
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.SetMat4("view", view);
        
        // render boxes
        glBindVertexArray(VAO);
        for (unsigned int i = 0; i < 10; i++)
        {
            // calculate the model matrix for each object and pass it to shader before drawing
            glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
            model = glm::translate(model, cubePositions[i]);
            float angle = 20.0f * i;
            model = glm::rotate(model, glm::radians(angle), glm::vec3(1.0f, 0.3f, 0.5f));
            ourShader.SetMat4("model", model);

            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        imguiCpp.NewFrame();
        cubePositions[0] = glm::vec3(particle7->position.x, particle7->position.y, particle7->position.z);
        cubePositions[1] = glm::vec3(particle8->position.x, particle8->position.y, particle8->position.z);
        cubePositions[2] = glm::vec3(particle3->position.x, particle3->position.y, particle3->position.z);
        cubePositions[3] = glm::vec3(particle4->position.x, particle4->position.y, particle4->position.z);
        cubePositions[4] = glm::vec3(particle5->position.x, particle5->position.y, particle5->position.z);
        cubePositions[5] = glm::vec3(particle6->position.x, particle6->position.y, particle6->position.z);
        ImguiStatsPanel(deltaTime);
        ImguiGamePanel(particle, physics, direction, power, isParticleLaunched, isGravityEnabled, alpha);

        /* Particle Panel Data*/
        ImGui::Begin("Particle Data");
        ImGui::Text("Particle name: %s", particle7->name.c_str());
        ImGui::Text("Particle position: (%f, %f, %f)", particle7->position.x, particle7->position.y, particle7->position.z);
        ImGui::Text("Particle velocity: (%f, %f, %f)", particle7->velocity.x, particle7->velocity.y, particle7->velocity.z);
        ImGui::Text("Particle acceleration: (%f, %f, %f)", particle7->acceleration.x, particle7->acceleration.y, particle7->acceleration.z);
        ImGui::Text("Particle mass: %f", particle7->mass);
        ImGui::End();
        ImGui::Begin("Particle Data");
        ImGui::Text("Particle name: %s", particle8->name.c_str());
        ImGui::Text("Particle position: (%f, %f, %f)", particle8->position.x, particle8->position.y, particle8->position.z);
        ImGui::Text("Particle velocity: (%f, %f, %f)", particle8->velocity.x, particle8->velocity.y, particle8->velocity.z);
        ImGui::Text("Particle acceleration: (%f, %f, %f)", particle8->acceleration.x, particle8->acceleration.y, particle8->acceleration.z);
        ImGui::Text("Particle mass: %f", particle8->mass);
        ImGui::BeginGroup();
        ImGui::SliderFloat("Drag coeff", &dragCoeff, 0.01f, 1.f);
        ImGui::SliderFloat("Drag coeff 2", &dragCoeff2, 0.01f, 1.f);
        forceDrag->SetDragCoefficients(dragCoeff, dragCoeff2);
        ImGui::EndGroup();
        ImGui::End();
        ImGui::Begin("Particle Data");
        ImGui::Text("Particle name: %s", particle3->name.c_str());
        ImGui::Text("Particle position: (%f, %f, %f)", particle3->position.x, particle3->position.y, particle3->position.z);
        ImGui::Text("Particle velocity: (%f, %f, %f)", particle3->velocity.x, particle3->velocity.y, particle3->velocity.z);
        ImGui::Text("Particle acceleration: (%f, %f, %f)", particle3->acceleration.x, particle3->acceleration.y, particle3->acceleration.z);
        ImGui::Text("Particle mass: %f", particle3->mass);
        ImGui::End();
        ImGui::Begin("Particle Data");
        ImGui::Text("Particle name: %s", particle4->name.c_str());
        ImGui::Text("Particle position: (%f, %f, %f)", particle4->position.x, particle4->position.y, particle4->position.z);
        ImGui::Text("Particle velocity: (%f, %f, %f)", particle4->velocity.x, particle4->velocity.y, particle4->velocity.z);
        ImGui::Text("Particle acceleration: (%f, %f, %f)", particle4->acceleration.x, particle4->acceleration.y, particle4->acceleration.z);
        ImGui::Text("Particle mass: %f", particle4->mass);
        ImGui::BeginGroup();
        ImGui::SliderFloat("Spring constant", &springConstant, 0.f, 1000.f);
        forceSpring->SetSpringConstant(springConstant);
        ImGui::EndGroup();
        ImGui::End();
        ImGui::Begin("Particle Data");
        ImGui::Text("Particle name: %s", particle5->name.c_str());
        ImGui::Text("Particle position: (%f, %f, %f)", particle5->position.x, particle5->position.y, particle5->position.z);
        ImGui::Text("Particle velocity: (%f, %f, %f)", particle5->velocity.x, particle5->velocity.y, particle5->velocity.z);
        ImGui::Text("Particle acceleration: (%f, %f, %f)", particle5->acceleration.x, particle5->acceleration.y, particle5->acceleration.z);
        ImGui::Text("Particle mass: %f", particle5->mass);
        ImGui::BeginGroup();
        ImGui::SliderFloat("Anchored Spring constant", &anchoredSpringConstant, 0.f, 1000.f);
        forceAnchoredSpring->SetSpringConstant(anchoredSpringConstant);
        ImGui::EndGroup();
        ImGui::End();
        ImGui::Begin("Particle Data");
        ImGui::Text("Particle name: %s", particle6->name.c_str());
        ImGui::Text("Particle position: (%f, %f, %f)", particle6->position.x, particle6->position.y, particle6->position.z);
        ImGui::Text("Particle velocity: (%f, %f, %f)", particle6->velocity.x, particle6->velocity.y, particle6->velocity.z);
        ImGui::Text("Particle acceleration: (%f, %f, %f)", particle6->acceleration.x, particle6->acceleration.y, particle6->acceleration.z);
        ImGui::Text("Particle mass: %f", particle6->mass);
        ImGui::End();

        imguiCpp.Render(); // Draw the imgui frame

        glfwSwapBuffers(window.GetHandle());
        glfwPollEvents();
    }

    // Deallocate all resources once they've outlived their purpose
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

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

    /* Particle Panel Data*/
    ImGui::Begin("Particle Data");
    ImGui::Text("Particle name: %s", particle->name.c_str());
    ImGui::Text("Particle position: (%f, %f, %f)", particle->position.x, particle->position.y, particle->position.z);
    ImGui::Text("Particle velocity: (%f, %f, %f)", particle->velocity.x, particle->velocity.y, particle->velocity.z);
    ImGui::Text("Particle acceleration: (%f, %f, %f)", particle->acceleration.x, particle->acceleration.y, particle->acceleration.z);
    ImGui::Text("Particle mass: %f", particle->mass);
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