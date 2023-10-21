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

void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
void MouseCallback(GLFWwindow* window, double xpos, double ypos);
void ProcessInput(GLFWwindow* window, float deltaTime);

void ContactsGenerator(std::shared_ptr<std::vector<ParticleContact>> contactArray, std::shared_ptr<std::vector<ParticleContactGenerator>> contacts);

void ImguiGamePanel(std::shared_ptr<Particle> particle, PhysicsSystem& physics, Vector3f& direction, float& power, bool& isParticleLaunched, bool& isGravityEnabled, float deltaTime);
void ImguiStatsPanel(float deltaTime);
void Vector3ClassTest();
double HiresTimeInSeconds();

// settings
const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 10.0f));
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
    std::shared_ptr<Particle> particle(new Particle(Vector3f(5.f, 20.f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), 1.f, "Particle"));
    std::shared_ptr<Particle> particle2(new Particle(Vector3f(-5.f, 20.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), 1.f, "Particle2"));
    std::shared_ptr<Particle> particle3(new Particle(Vector3f(5.0f, 10.f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), 1.f, "Particle3"));
    std::shared_ptr<Particle> particle4(new Particle(Vector3f(-5.0f, 10.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), 1.f, "Particle4"));

    std::shared_ptr<ForceGravity> forceGravity = std::make_shared<ForceGravity>();
    
    std::shared_ptr<ForceDrag> weakForceDrag = std::make_shared<ForceDrag>(5.0f, 0.0f);
    std::shared_ptr<ForceDrag> strongForceDrag = std::make_shared<ForceDrag>(10.0f, 0.0f);

    float springConstant = 100.0f;
    std::shared_ptr<ForceSpring> forceSpring12 = std::make_shared<ForceSpring>(
        springConstant,
        (particle->position - particle2->position).GetLength() - 5.f,
        particle);
    std::shared_ptr<ForceSpring> forceSpring23 = std::make_shared<ForceSpring>(
        springConstant,
        (particle2->position - particle3->position).GetLength(),
        particle2);
    std::shared_ptr<ForceSpring> forceSpring34 = std::make_shared<ForceSpring>(
        springConstant,
        (particle3->position - particle4->position).GetLength(),
        particle3);
    std::shared_ptr<ForceSpring> forceSpring41 = std::make_shared<ForceSpring>(
        springConstant,
        (particle4->position - particle->position).GetLength(),
        particle4);
    
    std::shared_ptr<ForceAnchoredSpring> forceAnchoredSpring = std::make_shared<ForceAnchoredSpring>(springConstant, 5.0f, Vector3f(.0f, .0f, .0f));
    
    std::shared_ptr<ForceBuoyancy> forceBuoyancy = std::make_shared<ForceBuoyancy>(1.f, 1.f, 1.f, 1.f);

    physics.AddParticle(particle);
    physics.AddParticle(particle2);
    physics.AddParticle(particle3);
    physics.AddParticle(particle4);

    //forceRegistry->Add(particle, forceSpring41);
    //forceRegistry->Add(particle2, forceSpring12);
    //forceRegistry->Add(particle3, forceSpring23);
    //forceRegistry->Add(particle4, forceSpring34);

    forceRegistry->Add(particle, forceAnchoredSpring);
    forceRegistry->Add(particle2, forceAnchoredSpring);
    forceRegistry->Add(particle3, forceAnchoredSpring);
    forceRegistry->Add(particle4, forceAnchoredSpring);

    forceRegistry->Add(particle, forceGravity);
    //forceRegistry->Add(particle3, forceGravity);

    forceRegistry->Add(particle, strongForceDrag);
    forceRegistry->Add(particle2, strongForceDrag);
    forceRegistry->Add(particle3, strongForceDrag);
    forceRegistry->Add(particle4, strongForceDrag);
    #pragma endregion

    // Game variables
    Vector3f direction(0.0f, 1.0f, 0.0f);
    float power = 4.f;
    bool isParticleLaunched = false;
    bool isGravityEnabled = true;
   
    #pragma region Contacts
    ParticleContactResolver contactResolver(50);

    std::shared_ptr<std::vector<ParticleContact>> contactArray = std::make_shared<std::vector<ParticleContact>>(std::vector<ParticleContact>());
    std::shared_ptr<std::vector<ParticleContactGenerator>> contacts = std::make_shared<std::vector<ParticleContactGenerator>>(std::vector<ParticleContactGenerator>());

    std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles12 = std::make_shared<std::vector<std::shared_ptr<Particle>>>(std::vector<std::shared_ptr<Particle>>{particle, particle2});
    std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles23 = std::make_shared<std::vector<std::shared_ptr<Particle>>>(std::vector<std::shared_ptr<Particle>>{particle2, particle3});
    std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles34 = std::make_shared<std::vector<std::shared_ptr<Particle>>>(std::vector<std::shared_ptr<Particle>>{particle3, particle4});
    std::shared_ptr<std::vector<std::shared_ptr<Particle>>> particles41 = std::make_shared<std::vector<std::shared_ptr<Particle>>>(std::vector<std::shared_ptr<Particle>>{particle4, particle});

    contacts->push_back(ParticleRod(std::shared_ptr<std::vector<std::shared_ptr<Particle>>>(particles12), 10.f));
    contacts->push_back(ParticleRod(std::shared_ptr<std::vector<std::shared_ptr<Particle>>>(particles23), 10.f));
    contacts->push_back(ParticleRod(std::shared_ptr<std::vector<std::shared_ptr<Particle>>>(particles34), 10.f));
    contacts->push_back(ParticleRod(std::shared_ptr<std::vector<std::shared_ptr<Particle>>>(particles41), 10.f));
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

        ContactsGenerator(contactArray, contacts);
        contactResolver.ResolveContacts(contactArray, 0, 0.01);
       
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

        cubePositions[0] = glm::vec3(particle->position.x, particle->position.y, particle->position.z);
        cubePositions[1] = glm::vec3(particle2->position.x, particle2->position.y, particle2->position.z);
        cubePositions[2] = glm::vec3(particle3->position.x, particle3->position.y, particle3->position.z);
        cubePositions[3] = glm::vec3(particle4->position.x, particle4->position.y, particle4->position.z);
        
        // render boxes
        glBindVertexArray(VAO);
        for (unsigned int i = 0; i <= cubePositions->length(); i++)
        {
            // calculate the model matrix for each object and pass it to shader before drawing
            glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
            model = glm::translate(model, cubePositions[i]);
            ourShader.SetMat4("model", model);

            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        imguiCpp.NewFrame();

        ImguiStatsPanel(deltaTime);
        ImguiGamePanel(particle, physics, direction, power, isParticleLaunched, isGravityEnabled, alpha);

        ImGui::Begin("Particle Data");
        ImGui::Text("Particle name: %s", particle->name.c_str());
        ImGui::Text("Particle position: (%f, %f, %f)", particle->position.x, particle->position.y, particle->position.z);
        ImGui::Text("Particle velocity: (%f, %f, %f)", particle->velocity.x, particle->velocity.y, particle->velocity.z);
        ImGui::Text("Particle acceleration: (%f, %f, %f)", particle->acceleration.x, particle->acceleration.y, particle->acceleration.z);
        ImGui::Text("Particle mass: %f", particle->mass);
        ImGui::Text("Particle name: %s", particle2->name.c_str());
        ImGui::Text("Particle position: (%f, %f, %f)", particle2->position.x, particle2->position.y, particle2->position.z);
        ImGui::Text("Particle velocity: (%f, %f, %f)", particle2->velocity.x, particle2->velocity.y, particle2->velocity.z);
        ImGui::Text("Particle acceleration: (%f, %f, %f)", particle2->acceleration.x, particle2->acceleration.y, particle2->acceleration.z);
        ImGui::Text("Particle mass: %f", particle2->mass);
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
    //ImGui::Begin("Particle Data");
    //ImGui::Text("Particle name: %s", particle->name.c_str());
    //ImGui::Text("Particle position: (%f, %f, %f)", particle->position.x, particle->position.y, particle->position.z);
    //ImGui::Text("Particle velocity: (%f, %f, %f)", particle->velocity.x, particle->velocity.y, particle->velocity.z);
    //ImGui::Text("Particle acceleration: (%f, %f, %f)", particle->acceleration.x, particle->acceleration.y, particle->acceleration.z);
    //ImGui::Text("Particle mass: %f", particle->mass);
    //ImGui::Text("Particle name: %s", particle2->name.c_str());
    //ImGui::Text("Particle position: (%f, %f, %f)", particle2->position.x, particle->position.y, particle->position.z);
    //ImGui::Text("Particle velocity: (%f, %f, %f)", particle2->velocity.x, particle->velocity.y, particle->velocity.z);
    //ImGui::Text("Particle acceleration: (%f, %f, %f)", particle->acceleration.x, particle->acceleration.y, particle->acceleration.z);
    //ImGui::Text("Particle mass: %f", particle->mass);
    //ImGui::End();
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

void ContactsGenerator(std::shared_ptr<std::vector<ParticleContact>> contactArray, std::shared_ptr<std::vector<ParticleContactGenerator>> contacts)
{
    for (int i = 0; i < contacts->size(); i++)
    {
        contacts->at(i).AddContact(contactArray, 0);
    }
}