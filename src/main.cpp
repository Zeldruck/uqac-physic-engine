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

void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
void MouseCallback(GLFWwindow* window, double xpos, double ypos);
void ProcessInput(GLFWwindow* window, float deltaTime);

void ContactsGenerator(std::vector<std::shared_ptr<ParticleContact>>& contactArray, std::vector<std::shared_ptr<ParticleContactGenerator>>& contacts);

void ImguiGamePanel(std::shared_ptr<PhysicsBody> particle, PhysicsSystem& physics, Vector3f& direction, float& power, bool& isParticleLaunched, bool& isGravityEnabled, float deltaTime);
void ImguiStatsPanel(float deltaTime);
void Vector3ClassTest();
double HiresTimeInSeconds();

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


    Vector3ClassTest();

    #pragma region Physics
    std::shared_ptr<ForceRegistry> forceRegistry = std::make_shared<ForceRegistry>();
    PhysicsSystem physics(forceRegistry);
    std::shared_ptr<PhysicsBody> particle = std::make_shared<Particle>(Vector3f(5.f, 20.f, 0.0f), Vector3f::Zero, Vector3f::Zero, 1.f, "Particle");
    std::shared_ptr<PhysicsBody> particle2 = std::make_shared<Particle>(Vector3f(-5.f, 20.0f, 0.0f), Vector3f::Zero, Vector3f::Zero, 1.f, "Particle2");
    std::shared_ptr<PhysicsBody> particle3 = std::make_shared<Particle>(Vector3f(5.0f, 10.f, 0.0f), Vector3f::Zero, Vector3f::Zero, 1.f, "Particle3");
    std::shared_ptr<PhysicsBody> particle4 = std::make_shared<Particle>(Vector3f(-5.0f, 10.0f, 0.0f), Vector3f::Zero, Vector3f::Zero, 1.f, "Particle4");

    std::shared_ptr<PhysicsBody> rigidbody = std::make_shared<Rigidbody>(Transform(), Vector3f::Zero, Vector3f::Zero, 1.f, Vector3f::Zero, Vector3f::Zero, Vector3f::Zero, "Rigidbody");
    
    std::shared_ptr<ForceGravity> forceGravity = std::make_shared<ForceGravity>();
    
    std::shared_ptr<ForceDrag> weakForceDrag = std::make_shared<ForceDrag>(5.0f, 0.0f);
    std::shared_ptr<ForceDrag> strongForceDrag = std::make_shared<ForceDrag>(1.0f, 0.0f);

    float springConstant = 100.f;
    std::shared_ptr<ForceSpring> forceSpring12 = std::make_shared<ForceSpring>(
        springConstant,
        (particle->GetPosition() - particle2->GetPosition()).GetLength() - 5.f,
        particle);
    std::shared_ptr<ForceSpring> forceSpring23 = std::make_shared<ForceSpring>(
        springConstant,
        (particle2->GetPosition() - particle3->GetPosition()).GetLength(),
        particle2);
    std::shared_ptr<ForceSpring> forceSpring34 = std::make_shared<ForceSpring>(
        springConstant,
        (particle3->GetPosition() - particle4->GetPosition()).GetLength(),
        particle3);
    std::shared_ptr<ForceSpring> forceSpring41 = std::make_shared<ForceSpring>(
        springConstant,
        (particle4->GetPosition() - particle->GetPosition()).GetLength(),
        particle4);
    
    std::shared_ptr<ForceAnchoredSpring> forceAnchoredSpring = std::make_shared<ForceAnchoredSpring>(springConstant, 5.0f, Vector3f(.0f, .0f, .0f));
    
    std::shared_ptr<ForceBuoyancy> forceBuoyancy = std::make_shared<ForceBuoyancy>(1.f, 1.f, 1.f, 1.f);

    physics.AddParticle(particle);
    physics.AddParticle(particle2);
    physics.AddParticle(particle3);
    physics.AddParticle(particle4);

    physics.AddParticle(rigidbody);

    //forceRegistry->Add(particle, forceSpring41);
    //forceRegistry->Add(particle2, forceSpring12);
    //forceRegistry->Add(particle3, forceSpring23);
    //forceRegistry->Add(particle4, forceSpring34);

    forceRegistry->Add(particle, forceAnchoredSpring);
    forceRegistry->Add(particle2, forceAnchoredSpring);
    forceRegistry->Add(particle3, forceAnchoredSpring);
    forceRegistry->Add(particle4, forceAnchoredSpring);

    forceRegistry->Add(particle, forceGravity);
    forceRegistry->Add(particle3, forceGravity);

    forceRegistry->Add(particle, weakForceDrag);
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

    std::vector<std::shared_ptr<ParticleContact>> contactArray;
    std::vector<std::shared_ptr<ParticleContactGenerator>> contacts;

    std::vector<std::shared_ptr<PhysicsBody>> particles12 = {particle, particle2};
    std::vector<std::shared_ptr<PhysicsBody>> particles23 = {particle2, particle3};
    std::vector<std::shared_ptr<PhysicsBody>> particles34 = {particle3, particle4};
    std::vector<std::shared_ptr<PhysicsBody>> particles41 = {particle4, particle};

    std::shared_ptr<ParticleCable> particleCable = std::make_shared<ParticleCable>(particles12, 15.f, 1.f);
    std::shared_ptr<ParticleCable> particleCable2 = std::make_shared<ParticleCable>(particles23, 15.f, 1.f);
    std::shared_ptr<ParticleCable> particleCable3 = std::make_shared<ParticleCable>(particles34, 15.f, 1.f);
    std::shared_ptr<ParticleCable> particleCable4 = std::make_shared<ParticleCable>(particles41, 15.f, 1.f);
    
    contacts.push_back(particleCable);
    contacts.push_back(particleCable2);
    contacts.push_back(particleCable3);
    contacts.push_back(particleCable4);

    //contacts->push_back(std::make_shared<ParticleCable>(std::shared_ptr<std::vector<std::shared_ptr<Particle>>>(particles12), 15.f, 1.f));
    //contacts->push_back(std::make_shared<ParticleCable>(std::shared_ptr<std::vector<std::shared_ptr<Particle>>>(particles23), 15.f, 1.f));
    //contacts->push_back(std::make_shared<ParticleCable>(std::shared_ptr<std::vector<std::shared_ptr<Particle>>>(particles34), 15.f, 1.f));
    //contacts->push_back(std::make_shared<ParticleCable>(std::shared_ptr<std::vector<std::shared_ptr<Particle>>>(particles41), 15.f, 1.f));
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
        contactResolver.ResolveContacts(contactArray, contactArray.size(), 0.01);
       
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

        cubePositions[0] = glm::vec3(particle->GetPosition().x, particle->GetPosition().y, particle->GetPosition().z);
        cubePositions[1] = glm::vec3(particle2->GetPosition().x, particle2->GetPosition().y, particle2->GetPosition().z);
        cubePositions[2] = glm::vec3(particle3->GetPosition().x, particle3->GetPosition().y, particle3->GetPosition().z);
        cubePositions[3] = glm::vec3(particle4->GetPosition().x, particle4->GetPosition().y, particle4->GetPosition().z);
        
        // render boxes
        glBindVertexArray(VAO);
        for (unsigned int i = 0; i <= cubePositions->length(); i++)
        {
            glm::mat4 model = glm::mat4(1.0f);
            model = glm::translate(model, cubePositions[i]);
            ourShader.SetMat4("model", model);

            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        imguiCpp.NewFrame();

        ImguiStatsPanel(deltaTime);
        ImguiGamePanel(particle, physics, direction, power, isParticleLaunched, isGravityEnabled, alpha);

        ImGui::Begin("Particle Data");
        ImGui::Text("Particle name: %s", particle->name.c_str());
        ImGui::Text("Particle position: (%f, %f, %f)", particle->GetPosition().x, particle->GetPosition().y, particle->GetPosition().z);
        ImGui::Text("Particle velocity: (%f, %f, %f)", particle->velocity.x, particle->velocity.y, particle->velocity.z);
        ImGui::Text("Particle acceleration: (%f, %f, %f)", particle->GetAcceleration().x, particle->GetAcceleration().y, particle->GetAcceleration().z);
        ImGui::Text("Particle mass: %f", particle->mass);
        ImGui::Text("Particle name: %s", particle2->name.c_str());
        ImGui::Text("Particle position: (%f, %f, %f)", particle2->GetPosition().x, particle2->GetPosition().y, particle2->GetPosition().z);
        ImGui::Text("Particle velocity: (%f, %f, %f)", particle2->velocity.x, particle2->velocity.y, particle2->velocity.z);
        ImGui::Text("Particle acceleration: (%f, %f, %f)", particle2->GetAcceleration().x, particle2->GetAcceleration().y, particle2->GetAcceleration().z);
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

void ImguiGamePanel(std::shared_ptr<PhysicsBody> particle, PhysicsSystem& physics, Vector3f& direction, float& power, bool& isParticleLaunched, bool& isGravityEnabled, float deltaTime)
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
        particle->GetPosition() = Vector3f(0, 0, 0);
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

void ContactsGenerator(std::vector<std::shared_ptr<ParticleContact>>& contactArray, std::vector<std::shared_ptr<ParticleContactGenerator>>& contacts)
{
    if(contacts.size() == 0)
		return;

    for (int i = 0; i < contacts.size(); i++)
    {
        contacts.at(i)->AddContact(contactArray, 0);
    }
}