#include <iostream>
#include <chrono>
#include <memory>
#include <vector>
#include <algorithm>

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include <imgui.h>

#include "Camera.hpp"

#include "EngineCpp/cppGLFW.hpp"
#include "EngineCpp/cppGLFWwindow.hpp"
#include "EngineCpp/cppImgui.hpp"
#include "EngineCpp/cppShader.hpp"

#include "PhysicsSystem.hpp"
#include "Particle.hpp"
#include "Rigidbody.hpp"

#include "Force/ForceRegistry.hpp"
#include "Force/ForceGenerator.hpp"
#include "Force/ForceGravity.hpp"
#include "Force/ForceDrag.hpp"
#include "Force/ForceSpring.hpp"
#include "Force/ForceAnchoredSpring.hpp"
#include "Force/ForceBuoyancy.hpp"

#include "Contact/ParticleContactResolver.hpp"
#include "Contact/ParticleContact.hpp"
#include "Contact/ParticleContactGenerator.hpp"
#include "Contact/ParticleLink.hpp"
#include "Contact/ParticleCable.hpp"
#include "Contact/ParticleRod.hpp"

#include "Collision/BVHNode.hpp"

#include "Collision/Primitives/Sphere.hpp"
#include "Collision/Primitives/Plane.hpp"
#include "Collision/Primitives/Box.hpp"
#include "Collision/Contact.hpp"
#include "Collision/ContactGenerator.hpp"
#include "Collision/ContactResolver.hpp"

using m_clock = std::chrono::high_resolution_clock;

#pragma region Settings
const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;
#pragma endregion

#pragma region Camera
Camera camera(glm::vec3(0.0f, 5.0f, 20.0f));

float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;
#pragma endregion

struct State
{
    double x = 0.0; // position
    double v = 0.0; // velocity

    State operator*(double a) const
    {
        return { x * a, v * a };
    }
    State operator+(const State& rhs) const
    {
        return { x + rhs.x, v + rhs.v };
    }
};

enum class Scene
{
    SCENE_1,
    SCENE_2,
    SCENE_3,
    SCENE_4,
    SCENE_5
};

#pragma region Functions
void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
void MouseCallback(GLFWwindow* window, double xposIn, double yposIn);
void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
void ProcessCameraInput(GLFWwindow* window, float deltaTime);
void ProcessSceneInput(GLFWwindow* window, Scene& currentScene);
double HiresTimeInSeconds();

void CreateCube(std::vector<glm::vec3>& cubeVertices, std::vector<glm::vec2>& cubeTexCoords, float size);
void CreateSphere(std::vector<glm::vec3>& sphereVertices, float radius, int slices, int stacks);

void LoadTexture(unsigned int& texture, const std::string& texturePath);

void ImGuiCameraPanel();
void ImGuiStatsPanel(float deltaTime);
void ImGuiSceneSelectionPanel(Scene& currentScene);
void ImGuiBroadPhasePanel(PotentialContact* potientialContact, unsigned int potentialContactCount);

void Scene1(cppGLFWwindow& window, ImguiCpp& imguiCpp, Scene& currentScene);
void ImGuiScene1Panel(const std::vector<std::shared_ptr<Particle>>& particles, const std::vector<glm::vec3> cubePositions);

void Scene2(cppGLFWwindow& window, ImguiCpp& imguiCpp, Scene& currentScene);
void ImGuiScene2Panel(const std::vector<std::shared_ptr<Rigidbody>>& rigidbodies, const std::vector<glm::vec3> cubePositions);

void Scene3(cppGLFWwindow& window, ImguiCpp& imguiCpp, Scene& currentScene);
void ImGuiScene3Panel(const std::vector<std::shared_ptr<Rigidbody>>& rigidbodies, const std::vector<glm::vec3> cubePositions);

void Scene4(cppGLFWwindow& window, ImguiCpp& imguiCpp, Scene& currentScene);
void ImGuiScene4Panel(const std::vector<std::shared_ptr<Rigidbody>>& rigidbodies, const std::vector<glm::vec3> cubePositions);

void Scene5(cppGLFWwindow& window, ImguiCpp& imguiCpp, Scene& currentScene);
void ImGuiScene5Panel(const std::vector<std::shared_ptr<Rigidbody>>& rigidbodies, const std::vector<glm::vec3> cubePositions);
#pragma endregion

int main()
{
    cppGLFW glfw;
    cppGLFWwindow window(SCR_WIDTH, SCR_HEIGHT, "Uqac physic engine v2");

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Error: Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glfwSetFramebufferSizeCallback(window.GetHandle(), FramebufferSizeCallback);
    glfwSetCursorPosCallback(window.GetHandle(), MouseCallback);
    glfwSetKeyCallback(window.GetHandle(), KeyCallback);

    glEnable(GL_DEPTH_TEST);

    ImguiCpp imguiCpp(&window);

#pragma region Loop
    Scene currentScene = Scene::SCENE_4;

    while (!window.ShouldClose())
    {
        switch (currentScene)
        {
            case Scene::SCENE_1:
                Scene1(window, imguiCpp, currentScene);
				break;
            case Scene::SCENE_2:
                Scene2(window, imguiCpp, currentScene);
				break;
            case Scene::SCENE_3:
                Scene3(window, imguiCpp, currentScene);
                break;
            case Scene::SCENE_4:
                Scene4(window, imguiCpp, currentScene);
                break;
            case Scene::SCENE_5:
                Scene5(window, imguiCpp, currentScene);
                break;
        }
    }
#pragma endregion Loop
	return 0;
}

void Scene1(cppGLFWwindow& window, ImguiCpp& imguiCpp, Scene& currentScene)
{
#pragma region PhysicsSystem
    std::shared_ptr<ForceRegistry> forceRegistry = std::make_shared<ForceRegistry>();
    PhysicsSystem physics(forceRegistry);
#pragma endregion

#pragma region Particles
    std::shared_ptr<Particle> particle1 = std::make_shared<Particle>("Particle 1 Gravity", Vector3f::Up * 10.0f);
    std::shared_ptr<Particle> particle2 = std::make_shared<Particle>("Particle 2 Gravity & Drag", Vector3f::Left * 10.0f, 1.0f);
    std::shared_ptr<Particle> particle3a = std::make_shared<Particle>("Particle 3A Spring", Vector3f::Right * 4.0f + Vector3f::Up * 10.0f, 1.0f);
    std::shared_ptr<Particle> particle3b = std::make_shared<Particle>("Particle 3B Spring", Vector3f::Left * 4.0f + Vector3f::Up * 10.0f, 1.0f);
    std::shared_ptr<Particle> particle4 = std::make_shared<Particle>("Particle 4 Anchored Spring", Vector3f::Up * 5.0f, 10.0f);
    std::shared_ptr<Particle> particle5 = std::make_shared<Particle>("Particle 5 Buoyancy", Vector3f::Down * 4.0f + Vector3f::Right * 10.0f, 1.0f);

    physics.AddParticle(particle1);
    physics.AddParticle(particle2);
    physics.AddParticle(particle3a);
    physics.AddParticle(particle3b);
    physics.AddParticle(particle4);
    physics.AddParticle(particle5);
#pragma endregion

#pragma region Forces
    std::shared_ptr<ForceGravity> gravity = std::make_shared<ForceGravity>();
	std::shared_ptr<ForceDrag> drag = std::make_shared<ForceDrag>(10.f, 0.0f);
	std::shared_ptr<ForceSpring> spring = std::make_shared<ForceSpring>(particle3b, 1.0f, 10.0f);
	std::shared_ptr<ForceAnchoredSpring> anchoredSpring = std::make_shared<ForceAnchoredSpring>(Vector3f::Zero, 100.0f, 0.0f);
	std::shared_ptr<ForceBuoyancy> buoyancy = std::make_shared<ForceBuoyancy>(2.0f, 1.0f, 0.0f, 10.0f);

    // Test Gravity
    forceRegistry->Add(particle1, gravity);

    // Test Gravity and Drag
    forceRegistry->Add(particle2, gravity);
	forceRegistry->Add(particle2, drag);

    // Test Spring
    forceRegistry->Add(particle3a, spring);

    // Test Anchored Spring
	forceRegistry->Add(particle4, anchoredSpring);

    // Test Buoyancy
    forceRegistry->Add(particle5, buoyancy);
#pragma endregion

#pragma region Shader
    Shader ourShader("assets/shaders/test.vert", "assets/shaders/test.frag");

    std::string texturePath = "assets/textures/test.jpg";
    unsigned int texture1;
    LoadTexture(texture1, texturePath);
#pragma endregion 

#pragma region Model
    // For Sphere
    std::vector<glm::vec3> sphereVertices;
    CreateSphere(sphereVertices, 1.f, 50, 50);

    std::vector<glm::vec3> spherePositions;
    spherePositions.push_back(glm::vec3(particle1->position.x, particle1->position.y, particle1->position.z));
    spherePositions.push_back(glm::vec3(particle2->position.x, particle2->position.y, particle2->position.z));
    spherePositions.push_back(glm::vec3(particle3a->position.x, particle3a->position.y, particle3a->position.z));
    spherePositions.push_back(glm::vec3(particle3b->position.x, particle3b->position.y, particle3b->position.z));
    spherePositions.push_back(glm::vec3(particle4->position.x, particle4->position.y, particle4->position.z));
    spherePositions.push_back(glm::vec3(particle5->position.x, particle5->position.y, particle5->position.z));

    GLuint VAO1, VBO1;
    glGenVertexArrays(1, &VAO1);
    glBindVertexArray(VAO1);

    glGenBuffers(1, &VBO1);
    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
    glBufferData(GL_ARRAY_BUFFER, sphereVertices.size() * sizeof(glm::vec3), sphereVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // For Cube
    std::vector<glm::vec3> cubeVertices;
    std::vector<glm::vec2> cubeTexCoords;

    CreateCube(cubeVertices, cubeTexCoords, 1.0f);

    std::vector<glm::vec3> cubePositions;
    cubePositions.push_back(glm::vec3(anchoredSpring->GetAnchor().x, anchoredSpring->GetAnchor().y, anchoredSpring->GetAnchor().z));

    GLuint VAO2, VBO2;
    glGenVertexArrays(1, &VAO2);
    glBindVertexArray(VAO2);

    // Vertex Positions
    glGenBuffers(1, &VBO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, cubeVertices.size() * sizeof(glm::vec3), cubeVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Texture Coords
    glGenBuffers(1, &VBO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, cubeTexCoords.size() * sizeof(glm::vec2), cubeTexCoords.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
#pragma endregion

#pragma region Timestep
    double t = 0.0;
    double dt = 0.01;
    double currentTime = HiresTimeInSeconds();
    double accumulator = 0.0;

    State previous;
    State current;
#pragma endregion

    glm::mat4 model = glm::mat4(1.0f);
#pragma region Loop

    while (!window.ShouldClose() && currentScene == Scene::SCENE_1)
    {
        double newTime = HiresTimeInSeconds();
        double frameTime = newTime - currentTime;
        // max frame time to avoid spiral of death(on slow devices)
        if (frameTime > 0.015)
            frameTime = 0.015;
        currentTime = newTime;

        accumulator += frameTime;

        while (accumulator >= dt)
        {
            previous = current;
            physics.Update(dt, true, false);
            accumulator -= dt;
            t += dt;
        }

        const double alpha = accumulator / dt;
        State state = current * alpha + previous * (1.0 - alpha);

        ProcessCameraInput(window.GetHandle(), dt);
        ProcessSceneInput(window.GetHandle(), currentScene);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ourShader.Use();

        glm::mat4 projection = glm::perspective(glm::radians(45.f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.SetMat4("projection", projection);
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.SetMat4("view", view);

        // Update Spheres Positions
        spherePositions[0] = glm::vec3(particle1->position.x, particle1->position.y, particle1->position.z);
        spherePositions[1] = glm::vec3(particle2->position.x, particle2->position.y, particle2->position.z);
        spherePositions[2] = glm::vec3(particle3a->position.x, particle3a->position.y, particle3a->position.z);
        spherePositions[3] = glm::vec3(particle3b->position.x, particle3b->position.y, particle3b->position.z);
        spherePositions[4] = glm::vec3(particle4->position.x, particle4->position.y, particle4->position.z);
        spherePositions[5] = glm::vec3(particle5->position.x, particle5->position.y, particle5->position.z);

        // Render Spheres
        for (int i = 0; i < spherePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Initialize model matrix for each object
            model = glm::translate(model, spherePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO1);
            glDrawArrays(GL_LINE_STRIP, 0, sphereVertices.size());
        }

        // Render Cubes
        for (int i = 0; i < cubePositions.size(); ++i)
        {
			model = glm::mat4(1.0f); // Reset model matrix for the next object
			model = glm::translate(model, cubePositions[i]);
			ourShader.SetMat4("model", model);
			glBindVertexArray(VAO2);
			glDrawArrays(GL_TRIANGLES, 0, cubeVertices.size());
		}

        imguiCpp.NewFrame();
        // Add imgui panels here
        ImGuiCameraPanel();
        ImGuiStatsPanel(dt);
        ImGuiSceneSelectionPanel(currentScene);
        ImGuiScene1Panel(physics.GetParticles(), cubePositions);
        imguiCpp.Render();

        glfwSwapBuffers(window.GetHandle());
        glfwPollEvents();
    }
#pragma endregion
    glDeleteVertexArrays(1, &VAO1);
    glDeleteBuffers(1, &VBO1);
    glDeleteVertexArrays(1, &VAO2);
    glDeleteBuffers(1, &VBO2);
}

void ImGuiScene1Panel(const std::vector<std::shared_ptr<Particle>>& particles, const std::vector<glm::vec3> cubesPositions)
{
    ImGui::Begin("Center Cube Anchored");
    ImGui::Text("Position: %f, %f, %f", cubesPositions[0].x, cubesPositions[0].y, cubesPositions[0].z);
    ImGui::End();
    ImGui::Begin("Particles");
    for (auto& particle : particles)
    {
        ImGui::Text("%s", particle->name.c_str());
        ImGui::Text("Position: %f, %f, %f", particle->position.x, particle->position.y, particle->position.z);
        ImGui::Text("Velocity: %f, %f, %f", particle->velocity.x, particle->velocity.y, particle->velocity.z);
        ImGui::Text("Acceleration: %f, %f, %f", particle->GetAcceleration().x, particle->GetAcceleration().y, particle->GetAcceleration().z);
        ImGui::Text("Mass: %f", particle->mass);
        ImGui::Separator();
    }
    ImGui::End();
}

void Scene2(cppGLFWwindow& window, ImguiCpp& imguiCpp, Scene& currentScene)
{
#pragma region PhysicsSystem
    std::shared_ptr<ForceRegistry> forceRegistry = std::make_shared<ForceRegistry>();
    PhysicsSystem physics(forceRegistry);
#pragma endregion

#pragma region Rigidbodies
    std::shared_ptr<Rigidbody> rigidbody1 = std::make_shared<Rigidbody>("Rigidbody 1 Gravity", Vector3f::Up * 10.0f);
	std::shared_ptr<Rigidbody> rigidbody2 = std::make_shared<Rigidbody>("Rigidbody 2 Gravity & Drag", Vector3f::Left * 10.0f, 1.0f);
	std::shared_ptr<Rigidbody> rigidbody3a = std::make_shared<Rigidbody>("Rigidbody 3A Spring", Vector3f::Right * 4.0f + Vector3f::Up * 10.0f, 1.0f);
	std::shared_ptr<Rigidbody> rigidbody3b = std::make_shared<Rigidbody>("Rigidbody 3B Spring", Vector3f::Left * 4.0f + Vector3f::Up * 10.0f, 1.0f);
    std::shared_ptr<Rigidbody> rigidbody3c = std::make_shared<Rigidbody>("Rigidbody 3C Spring At Point", Vector3f::Right * 4.0f + Vector3f::Up * 15.0f, 1.0f);
    std::shared_ptr<Rigidbody> rigidbody3d = std::make_shared<Rigidbody>("Rigidbody 3D Spring At Point", Vector3f::Left * 4.0f + Vector3f::Up * 15.0f, 1.0f);
	std::shared_ptr<Rigidbody> rigidbody4 = std::make_shared<Rigidbody>("Rigidbody 4 Anchored Spring", Vector3f::Up * 5.0f, 10.0f);
	std::shared_ptr<Rigidbody> rigidbody4b = std::make_shared<Rigidbody>("Rigidbody 5 Anchored Spring At Point", Vector3f::Up * 5.0f, 10.0f);
    std::shared_ptr<Rigidbody> rigidbody5 = std::make_shared<Rigidbody>("Rigidbody 6 Buoyancy", Vector3f::Down * 4.0f + Vector3f::Right * 10.0f, 1.0f);

	physics.AddRigidbody(rigidbody1);
	physics.AddRigidbody(rigidbody2);
	physics.AddRigidbody(rigidbody3a);
	physics.AddRigidbody(rigidbody3b);
	physics.AddRigidbody(rigidbody3c);
	physics.AddRigidbody(rigidbody3d);
	physics.AddRigidbody(rigidbody4);
	physics.AddRigidbody(rigidbody4b);
    physics.AddRigidbody(rigidbody5);
#pragma endregion

#pragma region Forces
    std::shared_ptr<ForceGravity> gravity = std::make_shared<ForceGravity>();
    std::shared_ptr<ForceDrag> drag = std::make_shared<ForceDrag>(10.f, 0.0f);
    std::shared_ptr<ForceSpring> spring = std::make_shared<ForceSpring>(rigidbody3b, 1.0f, 10.0f);
    std::shared_ptr<ForceSpring> springAtPoint = std::make_shared<ForceSpring>(rigidbody3d, Vector3f::Left, Vector3f::Right, 1.0f, 10.0f);
    std::shared_ptr<ForceAnchoredSpring> anchoredSpring = std::make_shared<ForceAnchoredSpring>(Vector3f::Zero, 100.0f, 0.0f);
    std::shared_ptr<ForceAnchoredSpring> anchoredSpringAtPoint = std::make_shared<ForceAnchoredSpring>(Vector3f::Zero, Vector3f(0.5f, 0.5f, 0.5f), 100.0f, 0.0f);
    std::shared_ptr<ForceBuoyancy> buoyancy = std::make_shared<ForceBuoyancy>(2.0f, 1.0f, 0.0f, 10.0f);

    // Test Gravity
    forceRegistry->Add(rigidbody1, gravity);

    // Test Gravity and Drag
    forceRegistry->Add(rigidbody2, gravity);
    forceRegistry->Add(rigidbody2, drag);

    // Test Spring
    forceRegistry->Add(rigidbody3a, spring);

    // Test Spring with a connection point
    //forceRegistry->Add(rigidbody3c, gravity);
    //forceRegistry->Add(rigidbody3c, drag);
    forceRegistry->Add(rigidbody3c, springAtPoint);

    // Test Anchored Spring
    forceRegistry->Add(rigidbody4, anchoredSpring);

    // Test Anchored Spring with a connection point
    forceRegistry->Add(rigidbody4b, anchoredSpringAtPoint);

    // Test Buoyancy
    forceRegistry->Add(rigidbody5, buoyancy);
#pragma endregion

#pragma region Shader
    Shader ourShader("assets/shaders/test.vert", "assets/shaders/test.frag");

    std::string texturePath = "assets/textures/test.jpg";
    unsigned int texture1;
    LoadTexture(texture1, texturePath);
#pragma endregion 

#pragma region Model
    // For Sphere
    std::vector<glm::vec3> sphereVertices;
    CreateSphere(sphereVertices, 1.f, 50, 50);

    std::vector<glm::vec3> spherePositions;
    spherePositions.push_back(glm::vec3(rigidbody1->position.x, rigidbody1->position.y, rigidbody1->position.z));
    spherePositions.push_back(glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z));
    spherePositions.push_back(glm::vec3(rigidbody3a->position.x, rigidbody3a->position.y, rigidbody3a->position.z));
    spherePositions.push_back(glm::vec3(rigidbody3b->position.x, rigidbody3b->position.y, rigidbody3b->position.z));
    spherePositions.push_back(glm::vec3(rigidbody3c->position.x, rigidbody3c->position.y, rigidbody3c->position.z));
    spherePositions.push_back(glm::vec3(rigidbody3d->position.x, rigidbody3d->position.y, rigidbody3d->position.z));
    spherePositions.push_back(glm::vec3(rigidbody4->position.x, rigidbody4->position.y, rigidbody4->position.z));
    spherePositions.push_back(glm::vec3(rigidbody4b->position.x, rigidbody4b->position.y, rigidbody4b->position.z));
    spherePositions.push_back(glm::vec3(rigidbody5->position.x, rigidbody5->position.y, rigidbody5->position.z));

    GLuint VAO1, VBO1;
    glGenVertexArrays(1, &VAO1);
    glBindVertexArray(VAO1);

    glGenBuffers(1, &VBO1);
    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
    glBufferData(GL_ARRAY_BUFFER, sphereVertices.size() * sizeof(glm::vec3), sphereVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // For Cube
    std::vector<glm::vec3> cubeVertices;
    std::vector<glm::vec2> cubeTexCoords;
    CreateCube(cubeVertices, cubeTexCoords, 1.0f);

    std::vector<glm::vec3> cubePositions;
    cubePositions.push_back(glm::vec3(anchoredSpring->GetAnchor().x, anchoredSpring->GetAnchor().y, anchoredSpring->GetAnchor().z));
    cubePositions.push_back(glm::vec3(rigidbody3a->GetPointInWorldSpace(Vector3f::Zero).x, rigidbody3a->GetPointInWorldSpace(Vector3f::Zero).y, rigidbody3a->GetPointInWorldSpace(Vector3f::Zero).z));
    cubePositions.push_back(glm::vec3(rigidbody3b->GetPointInWorldSpace(Vector3f::Zero).x, rigidbody3b->GetPointInWorldSpace(Vector3f::Zero).y, rigidbody3b->GetPointInWorldSpace(Vector3f::Zero).z));
    cubePositions.push_back(glm::vec3(rigidbody3c->GetPointInWorldSpace(Vector3f::Left).x, rigidbody3c->GetPointInWorldSpace(Vector3f::Left).y, rigidbody3c->GetPointInWorldSpace(Vector3f::Left).z));
    cubePositions.push_back(glm::vec3(rigidbody3d->GetPointInWorldSpace(Vector3f::Right).x, rigidbody3d->GetPointInWorldSpace(Vector3f::Right).y, rigidbody3d->GetPointInWorldSpace(Vector3f::Right).z));
    cubePositions.push_back(glm::vec3(rigidbody4b->GetPointInWorldSpace(Vector3f(0.5f, 0.5f, 0.5f)).x, rigidbody4b->GetPointInWorldSpace(Vector3f(0.5f, 0.5f, 0.5f)).y, rigidbody4b->GetPointInWorldSpace(Vector3f(0.5f, 0.5f, 0.5f)).z));
    
    GLuint VAO2, VBO2;
    glGenVertexArrays(1, &VAO2);
    glBindVertexArray(VAO2);

    // Vertex Positions
    glGenBuffers(1, &VBO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, cubeVertices.size() * sizeof(glm::vec3), cubeVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Texture Coords
    glGenBuffers(1, &VBO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, cubeTexCoords.size() * sizeof(glm::vec2), cubeTexCoords.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
#pragma endregion

#pragma region Timestep
    double t = 0.0;
    double dt = 0.01;
    double currentTime = HiresTimeInSeconds();
    double accumulator = 0.0;

    State previous;
    State current;
#pragma endregion

    glm::mat4 model = glm::mat4(1.0f);
#pragma region Loop

    while (!window.ShouldClose() && currentScene == Scene::SCENE_2)
    {
        double newTime = HiresTimeInSeconds();
        double frameTime = newTime - currentTime;
        // max frame time to avoid spiral of death(on slow devices)
        if (frameTime > 0.015)
            frameTime = 0.015;
        currentTime = newTime;

        accumulator += frameTime;

        while (accumulator >= dt)
        {
            previous = current;
            physics.Update(dt, true, false);
            accumulator -= dt;
            t += dt;
        }

        const double alpha = accumulator / dt;
        State state = current * alpha + previous * (1.0 - alpha);

        ProcessCameraInput(window.GetHandle(), dt);
        ProcessSceneInput(window.GetHandle(), currentScene);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ourShader.Use();

        glm::mat4 projection = glm::perspective(glm::radians(45.f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.SetMat4("projection", projection);
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.SetMat4("view", view);

        // Update Spheres Positions
        spherePositions[0] = glm::vec3(rigidbody1->position.x, rigidbody1->position.y, rigidbody1->position.z);
        spherePositions[1] = glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z);
        spherePositions[2] = glm::vec3(rigidbody3a->position.x, rigidbody3a->position.y, rigidbody3a->position.z);
        spherePositions[3] = glm::vec3(rigidbody3b->position.x, rigidbody3b->position.y, rigidbody3b->position.z);
        spherePositions[4] = glm::vec3(rigidbody3c->position.x, rigidbody3c->position.y, rigidbody3c->position.z);
        spherePositions[5] = glm::vec3(rigidbody3d->position.x, rigidbody3d->position.y, rigidbody3d->position.z);
        spherePositions[6] = glm::vec3(rigidbody4->position.x, rigidbody4->position.y, rigidbody4->position.z);
        spherePositions[7] = glm::vec3(rigidbody4b->position.x, rigidbody4b->position.y, rigidbody4b->position.z);
        spherePositions[8] = glm::vec3(rigidbody5->position.x, rigidbody5->position.y, rigidbody5->position.z);

        // Render Spheres
        for (int i = 0; i < spherePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Initialize model matrix for each object
            model = glm::translate(model, spherePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO1);
            glDrawArrays(GL_LINE_STRIP, 0, sphereVertices.size());
        }

        // Update Cubes Positions
        cubePositions[0] = glm::vec3(anchoredSpring->GetAnchor().x, anchoredSpring->GetAnchor().y, anchoredSpring->GetAnchor().z);
        cubePositions[1] = glm::vec3(rigidbody3a->GetPointInWorldSpace(Vector3f::Zero).x, rigidbody3a->GetPointInWorldSpace(Vector3f::Zero).y, rigidbody3a->GetPointInWorldSpace(Vector3f::Zero).z);
        cubePositions[2] = glm::vec3(rigidbody3b->GetPointInWorldSpace(Vector3f::Zero).x, rigidbody3b->GetPointInWorldSpace(Vector3f::Zero).y, rigidbody3b->GetPointInWorldSpace(Vector3f::Zero).z);
        cubePositions[3] = glm::vec3(rigidbody3c->GetPointInWorldSpace(Vector3f::Left).x, rigidbody3c->GetPointInWorldSpace(Vector3f::Left).y, rigidbody3c->GetPointInWorldSpace(Vector3f::Left).z);
        cubePositions[4] = glm::vec3(rigidbody3d->GetPointInWorldSpace(Vector3f::Right).x, rigidbody3d->GetPointInWorldSpace(Vector3f::Right).y, rigidbody3d->GetPointInWorldSpace(Vector3f::Right).z);
        cubePositions[5] = glm::vec3(rigidbody4b->GetPointInWorldSpace(Vector3f(0.5f, 0.5f, 0.5f)).x, rigidbody4b->GetPointInWorldSpace(Vector3f(0.5f, 0.5f, 0.5f)).y, rigidbody4b->GetPointInWorldSpace(Vector3f(0.5f, 0.5f, 0.5f)).z);

        // Render Cubes
        for (int i = 0; i < cubePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Reset model matrix for the next object
            model = glm::translate(model, cubePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO2);
            glDrawArrays(GL_TRIANGLES, 0, cubeVertices.size());
        }

        imguiCpp.NewFrame();
        // Add imgui panels here
        ImGuiCameraPanel();
        ImGuiStatsPanel(dt);
        ImGuiSceneSelectionPanel(currentScene);
        ImGuiScene2Panel(physics.GetRigidbodies(), cubePositions);
        imguiCpp.Render();

        glfwSwapBuffers(window.GetHandle());
        glfwPollEvents();
    }
#pragma endregion
    glDeleteVertexArrays(1, &VAO1);
    glDeleteBuffers(1, &VBO1);
    glDeleteVertexArrays(1, &VAO2);
    glDeleteBuffers(1, &VBO2);
}

void ImGuiScene2Panel(const std::vector<std::shared_ptr<Rigidbody>>& rigidbodies, const std::vector<glm::vec3> cubesPositions)
{
    ImGui::Begin("Center Cube Anchored");
    ImGui::Text("Position: %f, %f, %f", cubesPositions[0].x, cubesPositions[0].y, cubesPositions[0].z);
    ImGui::End();
    ImGui::Begin("Rigidbodies");
    for (auto& rigidbody : rigidbodies)
    {
        ImGui::Text("%s", rigidbody->name.c_str());
        ImGui::Text("Position: %f, %f, %f", rigidbody->position.x, rigidbody->position.y, rigidbody->position.z);
        ImGui::Text("Rotation: %f, %f, %f", rigidbody->rotation.GetX(), rigidbody->rotation.GetY(), rigidbody->rotation.GetZ());
        ImGui::Text("Scale: %f, %f, %f", rigidbody->scale.x, rigidbody->scale.y, rigidbody->scale.z);
        ImGui::Text("Velocity: %f, %f, %f", rigidbody->velocity.x, rigidbody->velocity.y, rigidbody->velocity.z);
        ImGui::Text("Acceleration: %f, %f, %f", rigidbody->GetAcceleration().x, rigidbody->GetAcceleration().y, rigidbody->GetAcceleration().z);
        ImGui::Text("AngularVelocity: %f, %f, %f", rigidbody->angularVelocity.x, rigidbody->angularVelocity.y, rigidbody->angularVelocity.z);
        ImGui::Text("AngularAcceleration: %f, %f, %f", rigidbody->GetAngularAcceleration().x, rigidbody->GetAngularAcceleration().y, rigidbody->GetAngularAcceleration().z);
        ImGui::Text("Mass: %f", rigidbody->mass);
        ImGui::Text("Inverse Mass: %f", rigidbody->inverseMass);
        ImGui::Text("%s transformMatrix:\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f",
            rigidbody->name.c_str(),
            rigidbody->transformMatrix.Value(0, 0), rigidbody->transformMatrix.Value(0, 1), rigidbody->transformMatrix.Value(0, 2), rigidbody->transformMatrix.Value(0, 3),
            rigidbody->transformMatrix.Value(1, 0), rigidbody->transformMatrix.Value(1, 1), rigidbody->transformMatrix.Value(1, 2), rigidbody->transformMatrix.Value(1, 3),
            rigidbody->transformMatrix.Value(2, 0), rigidbody->transformMatrix.Value(2, 1), rigidbody->transformMatrix.Value(2, 2), rigidbody->transformMatrix.Value(2, 3),
            rigidbody->transformMatrix.Value(3, 0), rigidbody->transformMatrix.Value(3, 1), rigidbody->transformMatrix.Value(3, 2), rigidbody->transformMatrix.Value(3, 3));
        ImGui::Separator();
    }
    ImGui::End();
}

void Scene3(cppGLFWwindow& window, ImguiCpp& imguiCpp, Scene& currentScene)
{
#pragma region PhysicsSystem
    std::shared_ptr<ForceRegistry> forceRegistry = std::make_shared<ForceRegistry>();
    PhysicsSystem physics(forceRegistry);
#pragma endregion

#pragma region Rigidbodies
    std::shared_ptr<Rigidbody> rigidbody1 = std::make_shared<Rigidbody>("Rigidbody 1 Sphere", Vector3f::Up * 10.0f);
    std::shared_ptr<Rigidbody> rigidbody2 = std::make_shared<Rigidbody>("Rigidbody 2 Cube", RigidbodyType::CUBE, Vector3f::Zero, 10.0f);

    physics.AddRigidbody(rigidbody1);
    physics.AddRigidbody(rigidbody2);
#pragma endregion

#pragma region Forces
    std::shared_ptr<ForceGravity> gravity = std::make_shared<ForceGravity>();
    forceRegistry->Add(rigidbody1, gravity);
#pragma endregion

#pragma region BroadPhase

    std::shared_ptr<BoundingSphere> boundingSphere1 = std::make_shared<BoundingSphere>(rigidbody1);
    rigidbody1->m_boundingSphere = boundingSphere1;

    std::shared_ptr<BoundingSphere> boundingSphere2 = std::make_shared<BoundingSphere>(rigidbody2);
    rigidbody2->m_boundingSphere = boundingSphere2;

    std::shared_ptr<BVHNode> bvhRoot = std::make_shared<BVHNode>(rigidbody1);
    bvhRoot->Insert(rigidbody2, boundingSphere2);

    physics.AddRootBVHNode(bvhRoot);
    PotentialContact* potentialContacts = new PotentialContact;
#pragma endregion

#pragma region Shader
    Shader ourShader("assets/shaders/test.vert", "assets/shaders/test.frag");

    std::string texturePath = "assets/textures/test.jpg";
    unsigned int texture1;
    LoadTexture(texture1, texturePath);
#pragma endregion 

#pragma region Model
    // For Sphere
    std::vector<glm::vec3> sphereVertices;
    CreateSphere(sphereVertices, 1.f, 50, 50);

    std::vector<glm::vec3> spherePositions;
    spherePositions.push_back(glm::vec3(rigidbody1->position.x, rigidbody1->position.y, rigidbody1->position.z));
    spherePositions.push_back(glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z));

    GLuint VAO1, VBO1;
    glGenVertexArrays(1, &VAO1);
    glBindVertexArray(VAO1);

    glGenBuffers(1, &VBO1);
    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
    glBufferData(GL_ARRAY_BUFFER, sphereVertices.size() * sizeof(glm::vec3), sphereVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // For Cube
    std::vector<glm::vec3> cubeVertices;
    std::vector<glm::vec2> cubeTexCoords;
    CreateCube(cubeVertices, cubeTexCoords, 1.0f);

    std::vector<glm::vec3> cubePositions;
    cubePositions.push_back(glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z));

    GLuint VAO2, VBO2;
    glGenVertexArrays(1, &VAO2);
    glBindVertexArray(VAO2);

    // Vertex Positions
    glGenBuffers(1, &VBO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, cubeVertices.size() * sizeof(glm::vec3), cubeVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Texture Coords
    glGenBuffers(1, &VBO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, cubeTexCoords.size() * sizeof(glm::vec2), cubeTexCoords.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);

    // For Bounding Sphere
    GLuint VAO3, VBO3;
    glGenVertexArrays(1, &VAO3);
    glGenBuffers(1, &VBO3);
    glBindVertexArray(VAO3);
    glBindBuffer(GL_ARRAY_BUFFER, VBO3);
#pragma endregion

#pragma region Timestep
    double t = 0.0;
    double dt = 0.01;
    double currentTime = HiresTimeInSeconds();
    double accumulator = 0.0;

    State previous;
    State current;
#pragma endregion

    glm::mat4 model = glm::mat4(1.0f);
#pragma region Loop

    while (!window.ShouldClose() && currentScene == Scene::SCENE_3)
    {
        double newTime = HiresTimeInSeconds();
        double frameTime = newTime - currentTime;
        // max frame time to avoid spiral of death(on slow devices)
        if (frameTime > 0.015)
            frameTime = 0.015;
        currentTime = newTime;

        accumulator += frameTime;

        while (accumulator >= dt)
        {
            previous = current;
            physics.Update(dt, true, true);
            accumulator -= dt;
            t += dt;
        }

        const double alpha = accumulator / dt;
        State state = current * alpha + previous * (1.0 - alpha);

        ProcessCameraInput(window.GetHandle(), dt);
        ProcessSceneInput(window.GetHandle(), currentScene);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ourShader.Use();

        // Render Bounding 
        std::vector<glm::vec3> boundingSphereVertices;
        CreateSphere(boundingSphereVertices, bvhRoot->m_volume->GetRadius() * 2.0f + 0.5f, 30, 30);

        std::vector<glm::vec3> boundingSpherePositions;
        boundingSpherePositions.push_back(glm::vec3(bvhRoot->m_volume->GetCenter().x, bvhRoot->m_volume->GetCenter().y, bvhRoot->m_volume->GetCenter().z));

        glBindVertexArray(VAO3);
        glBufferData(GL_ARRAY_BUFFER, boundingSphereVertices.size() * sizeof(glm::vec3), boundingSphereVertices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glm::mat4 projection = glm::perspective(glm::radians(45.f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.SetMat4("projection", projection);
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.SetMat4("view", view);

        // Update Spheres Positions
        spherePositions[0] = glm::vec3(rigidbody1->position.x, rigidbody1->position.y, rigidbody1->position.z);
        spherePositions[1] = glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z);
        
        // Render Spheres 
        for (int i = 0; i < spherePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Initialize model matrix for each object
            model = glm::translate(model, spherePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO1);
            glDrawArrays(GL_LINE_STRIP, 0, sphereVertices.size());
        }

        // Render Bounding Spheres
        for (int i = 0; i < boundingSpherePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Initialize model matrix for each object
            model = glm::translate(model, boundingSpherePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO3);
            glDrawArrays(GL_LINE_STRIP, 0, boundingSphereVertices.size());
        }
 
        // Render Cubes
        for (int i = 0; i < cubePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Reset model matrix for the next object
            model = glm::translate(model, cubePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO2);
            glDrawArrays(GL_TRIANGLES, 0, cubeVertices.size());
        }

        imguiCpp.NewFrame();
        // Add imgui panels here
        ImGuiCameraPanel();
        ImGuiStatsPanel(dt);
        ImGuiSceneSelectionPanel(currentScene);
        ImGuiScene3Panel(physics.GetRigidbodies(), cubePositions);
        ImGuiBroadPhasePanel(physics.GetPotentialContactArray(), physics.GetPotentialContactCount());
        imguiCpp.Render();

        glfwSwapBuffers(window.GetHandle());
        glfwPollEvents();
    }
#pragma endregion
    glDeleteVertexArrays(1, &VAO1);
    glDeleteBuffers(1, &VBO1);
    glDeleteVertexArrays(1, &VAO2);
    glDeleteBuffers(1, &VBO2);
    glDeleteVertexArrays(1, &VAO3);
    glDeleteBuffers(1, &VBO3);

    delete(potentialContacts);
}

void ImGuiScene3Panel(const std::vector<std::shared_ptr<Rigidbody>>& rigidbodies, const std::vector<glm::vec3> cubesPositions)
{
    ImGui::Begin("Center Cube Anchored");
    ImGui::Text("Position: %f, %f, %f", cubesPositions[0].x, cubesPositions[0].y, cubesPositions[0].z);
    ImGui::End();
    ImGui::Begin("Rigidbodies");
    for (auto& rigidbody : rigidbodies)
    {
        ImGui::Text("%s", rigidbody->name.c_str());
        ImGui::Text("Position: %f, %f, %f", rigidbody->position.x, rigidbody->position.y, rigidbody->position.z);
        ImGui::Text("Rotation: %f, %f, %f", rigidbody->rotation.GetX(), rigidbody->rotation.GetY(), rigidbody->rotation.GetZ());
        ImGui::Text("Scale: %f, %f, %f", rigidbody->scale.x, rigidbody->scale.y, rigidbody->scale.z);
        ImGui::Text("Velocity: %f, %f, %f", rigidbody->velocity.x, rigidbody->velocity.y, rigidbody->velocity.z);
        ImGui::Text("Acceleration: %f, %f, %f", rigidbody->GetAcceleration().x, rigidbody->GetAcceleration().y, rigidbody->GetAcceleration().z);
        ImGui::Text("AngularVelocity: %f, %f, %f", rigidbody->angularVelocity.x, rigidbody->angularVelocity.y, rigidbody->angularVelocity.z);
        ImGui::Text("AngularAcceleration: %f, %f, %f", rigidbody->GetAngularAcceleration().x, rigidbody->GetAngularAcceleration().y, rigidbody->GetAngularAcceleration().z);
        ImGui::Text("Mass: %f", rigidbody->mass);
        ImGui::Text("Inverse Mass: %f", rigidbody->inverseMass);
        ImGui::Text("%s transformMatrix:\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f",
            rigidbody->name.c_str(),
            rigidbody->transformMatrix.Value(0, 0), rigidbody->transformMatrix.Value(0, 1), rigidbody->transformMatrix.Value(0, 2), rigidbody->transformMatrix.Value(0, 3),
            rigidbody->transformMatrix.Value(1, 0), rigidbody->transformMatrix.Value(1, 1), rigidbody->transformMatrix.Value(1, 2), rigidbody->transformMatrix.Value(1, 3),
            rigidbody->transformMatrix.Value(2, 0), rigidbody->transformMatrix.Value(2, 1), rigidbody->transformMatrix.Value(2, 2), rigidbody->transformMatrix.Value(2, 3),
            rigidbody->transformMatrix.Value(3, 0), rigidbody->transformMatrix.Value(3, 1), rigidbody->transformMatrix.Value(3, 2), rigidbody->transformMatrix.Value(3, 3));
        ImGui::Separator();
    }
    ImGui::End();
}

void Scene4(cppGLFWwindow& window, ImguiCpp& imguiCpp, Scene& currentScene)
{
#pragma region PhysicsSystem
    std::shared_ptr<ForceRegistry> forceRegistry = std::make_shared<ForceRegistry>();
    PhysicsSystem physics(forceRegistry);
#pragma endregion

#pragma region Rigidbodies
    std::shared_ptr<Rigidbody> rigidbody1 = std::make_shared<Rigidbody>("Rigidbody 1 Sphere", Vector3f::Up * 10.0f);
    std::shared_ptr<Rigidbody> rigidbody2 = std::make_shared<Rigidbody>("Rigidbody 2 Cube", RigidbodyType::CUBE, Vector3f::Zero, 10.0f);

    physics.AddRigidbody(rigidbody1);
    physics.AddRigidbody(rigidbody2);
#pragma endregion

#pragma region Forces
    std::shared_ptr<ForceGravity> gravity = std::make_shared<ForceGravity>();
    forceRegistry->Add(rigidbody1, gravity);
#pragma endregion

#pragma region BroadPhase

    std::shared_ptr<BoundingSphere> boundingSphere1 = std::make_shared<BoundingSphere>(rigidbody1);
    rigidbody1->m_boundingSphere = boundingSphere1;

    std::shared_ptr<BoundingSphere> boundingSphere2 = std::make_shared<BoundingSphere>(rigidbody2);
    rigidbody2->m_boundingSphere = boundingSphere2;

    std::shared_ptr<BVHNode> bvhRoot = std::make_shared<BVHNode>(rigidbody1);
    bvhRoot->Insert(rigidbody2, boundingSphere2);

    physics.AddRootBVHNode(bvhRoot);
    PotentialContact* potentialContacts = new PotentialContact;
#pragma endregion

#pragma region Shader
    Shader ourShader("assets/shaders/test.vert", "assets/shaders/test.frag");

    std::string texturePath = "assets/textures/test.jpg";
    unsigned int texture1;
    LoadTexture(texture1, texturePath);
#pragma endregion 

#pragma region Model
    // For Sphere
    std::vector<glm::vec3> sphereVertices;
    CreateSphere(sphereVertices, 1.f, 50, 50);

    std::vector<glm::vec3> spherePositions;
    spherePositions.push_back(glm::vec3(rigidbody1->position.x, rigidbody1->position.y, rigidbody1->position.z));
    spherePositions.push_back(glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z));

    GLuint VAO1, VBO1;
    glGenVertexArrays(1, &VAO1);
    glBindVertexArray(VAO1);

    glGenBuffers(1, &VBO1);
    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
    glBufferData(GL_ARRAY_BUFFER, sphereVertices.size() * sizeof(glm::vec3), sphereVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // For Cube
    std::vector<glm::vec3> cubeVertices;
    std::vector<glm::vec2> cubeTexCoords;
    CreateCube(cubeVertices, cubeTexCoords, 1.0f);

    std::vector<glm::vec3> cubePositions;
    cubePositions.push_back(glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z));

    GLuint VAO2, VBO2;
    glGenVertexArrays(1, &VAO2);
    glBindVertexArray(VAO2);

    // Vertex Positions
    glGenBuffers(1, &VBO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, cubeVertices.size() * sizeof(glm::vec3), cubeVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Texture Coords
    glGenBuffers(1, &VBO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, cubeTexCoords.size() * sizeof(glm::vec2), cubeTexCoords.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);

    // For Bounding Sphere
    GLuint VAO3, VBO3;
    glGenVertexArrays(1, &VAO3);
    glGenBuffers(1, &VBO3);
    glBindVertexArray(VAO3);
    glBindBuffer(GL_ARRAY_BUFFER, VBO3);
#pragma endregion

#pragma region Timestep
    double t = 0.0;
    double dt = 0.01;
    double currentTime = HiresTimeInSeconds();
    double accumulator = 0.0;

    State previous;
    State current;
#pragma endregion

    glm::mat4 model = glm::mat4(1.0f);
#pragma region Loop

    while (!window.ShouldClose() && currentScene == Scene::SCENE_4)
    {
        double newTime = HiresTimeInSeconds();
        double frameTime = newTime - currentTime;
        // max frame time to avoid spiral of death(on slow devices)
        if (frameTime > 0.015)
            frameTime = 0.015;
        currentTime = newTime;

        accumulator += frameTime;

        while (accumulator >= dt)
        {
            previous = current;
            physics.Update(dt, true, true);
            accumulator -= dt;
            t += dt;
        }

        const double alpha = accumulator / dt;
        State state = current * alpha + previous * (1.0 - alpha);

        ProcessCameraInput(window.GetHandle(), dt);
        ProcessSceneInput(window.GetHandle(), currentScene);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ourShader.Use();

        // Render Bounding 
        std::vector<glm::vec3> boundingSphereVertices;
        CreateSphere(boundingSphereVertices, bvhRoot->m_volume->GetRadius() * 2.0f + 0.5f, 30, 30);

        std::vector<glm::vec3> boundingSpherePositions;
        boundingSpherePositions.push_back(glm::vec3(bvhRoot->m_volume->GetCenter().x, bvhRoot->m_volume->GetCenter().y, bvhRoot->m_volume->GetCenter().z));

        glBindVertexArray(VAO3);
        glBufferData(GL_ARRAY_BUFFER, boundingSphereVertices.size() * sizeof(glm::vec3), boundingSphereVertices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glm::mat4 projection = glm::perspective(glm::radians(45.f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.SetMat4("projection", projection);
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.SetMat4("view", view);

        // Update Spheres Positions
        spherePositions[0] = glm::vec3(rigidbody1->position.x, rigidbody1->position.y, rigidbody1->position.z);
        spherePositions[1] = glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z);

        // Render Spheres 
        for (int i = 0; i < spherePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Initialize model matrix for each object
            model = glm::translate(model, spherePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO1);
            glDrawArrays(GL_LINE_STRIP, 0, sphereVertices.size());
        }

        // Render Bounding Spheres
        for (int i = 0; i < boundingSpherePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Initialize model matrix for each object
            model = glm::translate(model, boundingSpherePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO3);
            glDrawArrays(GL_LINE_STRIP, 0, boundingSphereVertices.size());
        }

        // Render Cubes
        for (int i = 0; i < cubePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Reset model matrix for the next object
            model = glm::translate(model, cubePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO2);
            glDrawArrays(GL_TRIANGLES, 0, cubeVertices.size());
        }

        imguiCpp.NewFrame();
        // Add imgui panels here
        ImGuiCameraPanel();
        ImGuiStatsPanel(dt);
        ImGuiSceneSelectionPanel(currentScene);
        ImGuiScene3Panel(physics.GetRigidbodies(), cubePositions);
        ImGuiBroadPhasePanel(physics.GetPotentialContactArray(), physics.GetPotentialContactCount());
        imguiCpp.Render();

        glfwSwapBuffers(window.GetHandle());
        glfwPollEvents();
    }
#pragma endregion
    glDeleteVertexArrays(1, &VAO1);
    glDeleteBuffers(1, &VBO1);
    glDeleteVertexArrays(1, &VAO2);
    glDeleteBuffers(1, &VBO2);
    glDeleteVertexArrays(1, &VAO3);
    glDeleteBuffers(1, &VBO3);

    delete(potentialContacts);
}

void ImGuiScene4Panel(const std::vector<std::shared_ptr<Rigidbody>>& rigidbodies, const std::vector<glm::vec3> cubesPositions)
{
    ImGui::Begin("Center Cube Anchored");
    ImGui::Text("Position: %f, %f, %f", cubesPositions[0].x, cubesPositions[0].y, cubesPositions[0].z);
    ImGui::End();
    ImGui::Begin("Rigidbodies");
    for (auto& rigidbody : rigidbodies)
    {
        ImGui::Text("%s", rigidbody->name.c_str());
        ImGui::Text("Position: %f, %f, %f", rigidbody->position.x, rigidbody->position.y, rigidbody->position.z);
        ImGui::Text("Rotation: %f, %f, %f", rigidbody->rotation.GetX(), rigidbody->rotation.GetY(), rigidbody->rotation.GetZ());
        ImGui::Text("Scale: %f, %f, %f", rigidbody->scale.x, rigidbody->scale.y, rigidbody->scale.z);
        ImGui::Text("Velocity: %f, %f, %f", rigidbody->velocity.x, rigidbody->velocity.y, rigidbody->velocity.z);
        ImGui::Text("Acceleration: %f, %f, %f", rigidbody->GetAcceleration().x, rigidbody->GetAcceleration().y, rigidbody->GetAcceleration().z);
        ImGui::Text("AngularVelocity: %f, %f, %f", rigidbody->angularVelocity.x, rigidbody->angularVelocity.y, rigidbody->angularVelocity.z);
        ImGui::Text("AngularAcceleration: %f, %f, %f", rigidbody->GetAngularAcceleration().x, rigidbody->GetAngularAcceleration().y, rigidbody->GetAngularAcceleration().z);
        ImGui::Text("Mass: %f", rigidbody->mass);
        ImGui::Text("Inverse Mass: %f", rigidbody->inverseMass);
        ImGui::Text("%s transformMatrix:\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f",
            rigidbody->name.c_str(),
            rigidbody->transformMatrix.Value(0, 0), rigidbody->transformMatrix.Value(0, 1), rigidbody->transformMatrix.Value(0, 2), rigidbody->transformMatrix.Value(0, 3),
            rigidbody->transformMatrix.Value(1, 0), rigidbody->transformMatrix.Value(1, 1), rigidbody->transformMatrix.Value(1, 2), rigidbody->transformMatrix.Value(1, 3),
            rigidbody->transformMatrix.Value(2, 0), rigidbody->transformMatrix.Value(2, 1), rigidbody->transformMatrix.Value(2, 2), rigidbody->transformMatrix.Value(2, 3),
            rigidbody->transformMatrix.Value(3, 0), rigidbody->transformMatrix.Value(3, 1), rigidbody->transformMatrix.Value(3, 2), rigidbody->transformMatrix.Value(3, 3));
        ImGui::Separator();
    }
    ImGui::End();
}

void Scene5(cppGLFWwindow& window, ImguiCpp& imguiCpp, Scene& currentScene)
{
#pragma region PhysicsSystem
    std::shared_ptr<ForceRegistry> forceRegistry = std::make_shared<ForceRegistry>();
    PhysicsSystem physics(forceRegistry);
#pragma endregion

#pragma region Rigidbodies
    std::shared_ptr<Rigidbody> rigidbody1 = std::make_shared<Rigidbody>("Rigidbody 1 Sphere", Vector3f::Up * 10.0f);
    std::shared_ptr<Rigidbody> rigidbody2 = std::make_shared<Rigidbody>("Rigidbody 2 Cube", RigidbodyType::CUBE, Vector3f::Zero, 10.0f);

    physics.AddRigidbody(rigidbody1);
    physics.AddRigidbody(rigidbody2);
#pragma endregion

#pragma region Forces
    std::shared_ptr<ForceGravity> gravity = std::make_shared<ForceGravity>();
    forceRegistry->Add(rigidbody1, gravity);
#pragma endregion

#pragma region BroadPhase

    std::shared_ptr<BoundingSphere> boundingSphere1 = std::make_shared<BoundingSphere>(rigidbody1);
    rigidbody1->m_boundingSphere = boundingSphere1;

    std::shared_ptr<BoundingSphere> boundingSphere2 = std::make_shared<BoundingSphere>(rigidbody2);
    rigidbody2->m_boundingSphere = boundingSphere2;

    std::shared_ptr<BVHNode> bvhRoot = std::make_shared<BVHNode>(rigidbody1);
    bvhRoot->Insert(rigidbody2, boundingSphere2);

    physics.AddRootBVHNode(bvhRoot);
    PotentialContact* potentialContacts = new PotentialContact;
#pragma endregion

#pragma region Shader
    Shader ourShader("assets/shaders/test.vert", "assets/shaders/test.frag");

    std::string texturePath = "assets/textures/test.jpg";
    unsigned int texture1;
    LoadTexture(texture1, texturePath);
#pragma endregion 

#pragma region Model
    // For Sphere
    std::vector<glm::vec3> sphereVertices;
    CreateSphere(sphereVertices, 1.f, 50, 50);

    std::vector<glm::vec3> spherePositions;
    spherePositions.push_back(glm::vec3(rigidbody1->position.x, rigidbody1->position.y, rigidbody1->position.z));
    spherePositions.push_back(glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z));

    GLuint VAO1, VBO1;
    glGenVertexArrays(1, &VAO1);
    glBindVertexArray(VAO1);

    glGenBuffers(1, &VBO1);
    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
    glBufferData(GL_ARRAY_BUFFER, sphereVertices.size() * sizeof(glm::vec3), sphereVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // For Cube
    std::vector<glm::vec3> cubeVertices;
    std::vector<glm::vec2> cubeTexCoords;
    CreateCube(cubeVertices, cubeTexCoords, 1.0f);

    std::vector<glm::vec3> cubePositions;
    cubePositions.push_back(glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z));

    GLuint VAO2, VBO2;
    glGenVertexArrays(1, &VAO2);
    glBindVertexArray(VAO2);

    // Vertex Positions
    glGenBuffers(1, &VBO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, cubeVertices.size() * sizeof(glm::vec3), cubeVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Texture Coords
    glGenBuffers(1, &VBO2);
    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
    glBufferData(GL_ARRAY_BUFFER, cubeTexCoords.size() * sizeof(glm::vec2), cubeTexCoords.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);

    // For Bounding Sphere
    GLuint VAO3, VBO3;
    glGenVertexArrays(1, &VAO3);
    glGenBuffers(1, &VBO3);
    glBindVertexArray(VAO3);
    glBindBuffer(GL_ARRAY_BUFFER, VBO3);
#pragma endregion

#pragma region Timestep
    double t = 0.0;
    double dt = 0.01;
    double currentTime = HiresTimeInSeconds();
    double accumulator = 0.0;

    State previous;
    State current;
#pragma endregion

    glm::mat4 model = glm::mat4(1.0f);
#pragma region Loop

    while (!window.ShouldClose() && currentScene == Scene::SCENE_5)
    {
        double newTime = HiresTimeInSeconds();
        double frameTime = newTime - currentTime;
        // max frame time to avoid spiral of death(on slow devices)
        if (frameTime > 0.015)
            frameTime = 0.015;
        currentTime = newTime;

        accumulator += frameTime;

        while (accumulator >= dt)
        {
            previous = current;
            physics.Update(dt, true, true);
            accumulator -= dt;
            t += dt;
        }

        const double alpha = accumulator / dt;
        State state = current * alpha + previous * (1.0 - alpha);

        ProcessCameraInput(window.GetHandle(), dt);
        ProcessSceneInput(window.GetHandle(), currentScene);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ourShader.Use();

        // Render Bounding 
        std::vector<glm::vec3> boundingSphereVertices;
        CreateSphere(boundingSphereVertices, bvhRoot->m_volume->GetRadius() * 2.0f + 0.5f, 30, 30);

        std::vector<glm::vec3> boundingSpherePositions;
        boundingSpherePositions.push_back(glm::vec3(bvhRoot->m_volume->GetCenter().x, bvhRoot->m_volume->GetCenter().y, bvhRoot->m_volume->GetCenter().z));

        glBindVertexArray(VAO3);
        glBufferData(GL_ARRAY_BUFFER, boundingSphereVertices.size() * sizeof(glm::vec3), boundingSphereVertices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glm::mat4 projection = glm::perspective(glm::radians(45.f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.SetMat4("projection", projection);
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.SetMat4("view", view);

        // Update Spheres Positions
        spherePositions[0] = glm::vec3(rigidbody1->position.x, rigidbody1->position.y, rigidbody1->position.z);
        spherePositions[1] = glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z);

        // Render Spheres 
        for (int i = 0; i < spherePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Initialize model matrix for each object
            model = glm::translate(model, spherePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO1);
            glDrawArrays(GL_LINE_STRIP, 0, sphereVertices.size());
        }

        // Render Bounding Spheres
        for (int i = 0; i < boundingSpherePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Initialize model matrix for each object
            model = glm::translate(model, boundingSpherePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO3);
            glDrawArrays(GL_LINE_STRIP, 0, boundingSphereVertices.size());
        }

        // Render Cubes
        for (int i = 0; i < cubePositions.size(); ++i)
        {
            model = glm::mat4(1.0f); // Reset model matrix for the next object
            model = glm::translate(model, cubePositions[i]);
            ourShader.SetMat4("model", model);
            glBindVertexArray(VAO2);
            glDrawArrays(GL_TRIANGLES, 0, cubeVertices.size());
        }

        imguiCpp.NewFrame();
        // Add imgui panels here
        ImGuiCameraPanel();
        ImGuiStatsPanel(dt);
        ImGuiSceneSelectionPanel(currentScene);
        ImGuiScene3Panel(physics.GetRigidbodies(), cubePositions);
        ImGuiBroadPhasePanel(physics.GetPotentialContactArray(), physics.GetPotentialContactCount());
        imguiCpp.Render();

        glfwSwapBuffers(window.GetHandle());
        glfwPollEvents();
    }
#pragma endregion
    glDeleteVertexArrays(1, &VAO1);
    glDeleteBuffers(1, &VBO1);
    glDeleteVertexArrays(1, &VAO2);
    glDeleteBuffers(1, &VBO2);
    glDeleteVertexArrays(1, &VAO3);
    glDeleteBuffers(1, &VBO3);

    delete(potentialContacts);
}

void ImGuiScene5Panel(const std::vector<std::shared_ptr<Rigidbody>>& rigidbodies, const std::vector<glm::vec3> cubesPositions)
{
    ImGui::Begin("Center Cube Anchored");
    ImGui::Text("Position: %f, %f, %f", cubesPositions[0].x, cubesPositions[0].y, cubesPositions[0].z);
    ImGui::End();
    ImGui::Begin("Rigidbodies");
    for (auto& rigidbody : rigidbodies)
    {
        ImGui::Text("%s", rigidbody->name.c_str());
        ImGui::Text("Position: %f, %f, %f", rigidbody->position.x, rigidbody->position.y, rigidbody->position.z);
        ImGui::Text("Rotation: %f, %f, %f", rigidbody->rotation.GetX(), rigidbody->rotation.GetY(), rigidbody->rotation.GetZ());
        ImGui::Text("Scale: %f, %f, %f", rigidbody->scale.x, rigidbody->scale.y, rigidbody->scale.z);
        ImGui::Text("Velocity: %f, %f, %f", rigidbody->velocity.x, rigidbody->velocity.y, rigidbody->velocity.z);
        ImGui::Text("Acceleration: %f, %f, %f", rigidbody->GetAcceleration().x, rigidbody->GetAcceleration().y, rigidbody->GetAcceleration().z);
        ImGui::Text("AngularVelocity: %f, %f, %f", rigidbody->angularVelocity.x, rigidbody->angularVelocity.y, rigidbody->angularVelocity.z);
        ImGui::Text("AngularAcceleration: %f, %f, %f", rigidbody->GetAngularAcceleration().x, rigidbody->GetAngularAcceleration().y, rigidbody->GetAngularAcceleration().z);
        ImGui::Text("Mass: %f", rigidbody->mass);
        ImGui::Text("Inverse Mass: %f", rigidbody->inverseMass);
        ImGui::Text("%s transformMatrix:\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f",
            rigidbody->name.c_str(),
            rigidbody->transformMatrix.Value(0, 0), rigidbody->transformMatrix.Value(0, 1), rigidbody->transformMatrix.Value(0, 2), rigidbody->transformMatrix.Value(0, 3),
            rigidbody->transformMatrix.Value(1, 0), rigidbody->transformMatrix.Value(1, 1), rigidbody->transformMatrix.Value(1, 2), rigidbody->transformMatrix.Value(1, 3),
            rigidbody->transformMatrix.Value(2, 0), rigidbody->transformMatrix.Value(2, 1), rigidbody->transformMatrix.Value(2, 2), rigidbody->transformMatrix.Value(2, 3),
            rigidbody->transformMatrix.Value(3, 0), rigidbody->transformMatrix.Value(3, 1), rigidbody->transformMatrix.Value(3, 2), rigidbody->transformMatrix.Value(3, 3));
        ImGui::Separator();
    }
    ImGui::End();
}

void FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
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

void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	// Close window on escape key
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	// Toggle cursor on F1 key
    if (key == GLFW_KEY_F1 && action == GLFW_PRESS)
    {
		if (glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_NORMAL)
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		else
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	}

    // Toggle wireframe on F2 key
    if (key == GLFW_KEY_F2 && action == GLFW_PRESS)
    {
		if (glIsEnabled(GL_CULL_FACE))
			glDisable(GL_CULL_FACE);
		else
			glEnable(GL_CULL_FACE);
	}

    // Toggle depth test on F3 key
    if (key == GLFW_KEY_F3 && action == GLFW_PRESS)
    {
		if (glIsEnabled(GL_DEPTH_TEST))
			glDisable(GL_DEPTH_TEST);
		else
			glEnable(GL_DEPTH_TEST);
	}
}

void ProcessCameraInput(GLFWwindow* window, float deltaTime)
{
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(Camera_Movement::FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(Camera_Movement::BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(Camera_Movement::LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(Camera_Movement::RIGHT, deltaTime);
}

void ProcessSceneInput(GLFWwindow* window, Scene& currentScene)
{
	if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
		currentScene = Scene::SCENE_1;
	if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
		currentScene = Scene::SCENE_2;
	if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
		currentScene = Scene::SCENE_3;
    if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS)
        currentScene = Scene::SCENE_4;
    if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS)
        currentScene = Scene::SCENE_5;
}

double HiresTimeInSeconds()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(m_clock::now().time_since_epoch()).count() / 1000.0;
}

void CreateSphere(std::vector<glm::vec3>& vertices, float radius, int slices, int stacks) 
{
    for (int i = 0; i <= stacks; ++i)
    {
        float stackAngle = glm::pi<float>() * static_cast<float>(i) / static_cast<float>(stacks);
        float stackRadius = glm::sin(stackAngle);
        float stackHeight = glm::cos(stackAngle);

        for (int j = 0; j <= slices; ++j)
        {
            float sliceAngle = 2.0f * glm::pi<float>() * static_cast<float>(j) / static_cast<float>(slices);
            float x = stackRadius * glm::cos(sliceAngle);
            float y = stackRadius * glm::sin(sliceAngle);
            float z = stackHeight;

            vertices.emplace_back(x * radius, y * radius, z * radius);
        }
    }
}

void CreateCube(std::vector<glm::vec3>& cubeVertices, std::vector<glm::vec2>& cubeTexCoords, float _size)
{
    float size = _size / 2;
    // Front face
	cubeVertices.emplace_back(-size, -size, -size);
    cubeTexCoords.emplace_back(0.0f, 0.0f);
	cubeVertices.emplace_back(size, -size, -size);
    cubeTexCoords.emplace_back(1.0f, 0.0f);
	cubeVertices.emplace_back(size, size, -size);
    cubeTexCoords.emplace_back(1.0f, 1.0f);
	cubeVertices.emplace_back(size, size, -size);
    cubeTexCoords.emplace_back(1.0f, 1.0f);
	cubeVertices.emplace_back(-size, size, -size);
    cubeTexCoords.emplace_back(0.0f, 1.0f);
	cubeVertices.emplace_back(-size, -size, -size);
    cubeTexCoords.emplace_back(0.0f, 0.0f);

    // Back face
	cubeVertices.emplace_back(-size, -size, size);
    cubeTexCoords.emplace_back(1.0f, 0.0f);
	cubeVertices.emplace_back(size, -size, size);
    cubeTexCoords.emplace_back(0.0f, 0.0f);
	cubeVertices.emplace_back(size, size, size);
    cubeTexCoords.emplace_back(0.0f, 1.0f);
	cubeVertices.emplace_back(size, size, size);
    cubeTexCoords.emplace_back(0.0f, 1.0f);
	cubeVertices.emplace_back(-size, size, size);
    cubeTexCoords.emplace_back(1.0f, 1.0f);
	cubeVertices.emplace_back(-size, -size, size);
    cubeTexCoords.emplace_back(1.0f, 0.0f);

    // Left face
	cubeVertices.emplace_back(-size, size, size);
    cubeTexCoords.emplace_back(0.0f, 1.0f);
	cubeVertices.emplace_back(-size, size, -size);
    cubeTexCoords.emplace_back(1.0f, 1.0f);
	cubeVertices.emplace_back(-size, -size, -size);
    cubeTexCoords.emplace_back(1.0f, 0.0f);
	cubeVertices.emplace_back(-size, -size, -size);
    cubeTexCoords.emplace_back(1.0f, 0.0f);
	cubeVertices.emplace_back(-size, -size, size);
    cubeTexCoords.emplace_back(0.0f, 0.0f);
	cubeVertices.emplace_back(-size, size, size);
    cubeTexCoords.emplace_back(0.0f, 1.0f);

    // Right face
	cubeVertices.emplace_back(size, size, size);
    cubeTexCoords.emplace_back(1.0f, 1.0f);
	cubeVertices.emplace_back(size, size, -size);
    cubeTexCoords.emplace_back(0.0f, 1.0f);
	cubeVertices.emplace_back(size, -size, -size);
    cubeTexCoords.emplace_back(0.0f, 0.0f);
	cubeVertices.emplace_back(size, -size, -size);
    cubeTexCoords.emplace_back(0.0f, 0.0f);
	cubeVertices.emplace_back(size, -size, size);
    cubeTexCoords.emplace_back(1.0f, 0.0f);
	cubeVertices.emplace_back(size, size, size);
    cubeTexCoords.emplace_back(1.0f, 1.0f);

    // Bottom face
	cubeVertices.emplace_back(-size, -size, -size);
    cubeTexCoords.emplace_back(1.0f, 1.0f);
	cubeVertices.emplace_back(size, -size, -size);
    cubeTexCoords.emplace_back(0.0f, 1.0f);
	cubeVertices.emplace_back(size, -size, size);
    cubeTexCoords.emplace_back(0.0f, 0.0f);
	cubeVertices.emplace_back(size, -size, size);
    cubeTexCoords.emplace_back(0.0f, 0.0f);
	cubeVertices.emplace_back(-size, -size, size);
    cubeTexCoords.emplace_back(1.0f, 0.0f);
	cubeVertices.emplace_back(-size, -size, -size);
    cubeTexCoords.emplace_back(1.0f, 1.0f);

    // Top face
	cubeVertices.emplace_back(-size, size, -size);
    cubeTexCoords.emplace_back(0.0f, 1.0f);
	cubeVertices.emplace_back(size, size, -size);
    cubeTexCoords.emplace_back(1.0f, 1.0f);
	cubeVertices.emplace_back(size, size, size);
    cubeTexCoords.emplace_back(1.0f, 0.0f);
	cubeVertices.emplace_back(size, size, size);
    cubeTexCoords.emplace_back(1.0f, 0.0f);
	cubeVertices.emplace_back(-size, size, size);
    cubeTexCoords.emplace_back(0.0f, 0.0f);
	cubeVertices.emplace_back(-size, size, -size);
    cubeTexCoords.emplace_back(0.0f, 1.0f);
}

void LoadTexture(unsigned int& texture, const std::string& texturePath)
{
    // load and create a texture 
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);

	// set the texture wrapping parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	// set texture filtering parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	
    // load image, create texture and generate mipmaps
	int width, height, nrChannels;
	stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.
	
	// The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
	unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &nrChannels, 0);
	if (data)
	{
	    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	    glGenerateMipmap(GL_TEXTURE_2D);
	}
	else
	{
	    std::cout << "Failed to load texture" << std::endl;
	}
	stbi_image_free(data);
}

void ImGuiCameraPanel()
{
    ImGui::Begin("Camera");
    ImGui::Text("Camera position: (%f, %f, %f)", camera.Position.x, camera.Position.y, camera.Position.z);
    ImGui::Text("Camera rotation: (%f, %f)", camera.Yaw, camera.Pitch);
    ImGui::End();
}

void ImGuiStatsPanel(float deltaTime)
{
    ImGui::Begin("Stats Panel");
    ImGui::Text("Delta Time: %f", deltaTime);
    ImGui::Text("FPS: %.f", std::clamp(1000 / (deltaTime * 1000), 0.f, 60.f));
    ImGui::End();
}

void ImGuiSceneSelectionPanel(Scene& currentScene)
{
	ImGui::Begin("Scene Selection");
    ImGui::Text("Active Scene: %d", (int)currentScene + 1);
	if (ImGui::Button("Keyboard 1 : Scene Particles"))
		currentScene = Scene::SCENE_1;
	if (ImGui::Button("Keybaord 2 : Scene Rigidbody"))
		currentScene = Scene::SCENE_2;
	if (ImGui::Button("Keyboard 3 : Scene Collisions Broad Phase"))
		currentScene = Scene::SCENE_3;
    if (ImGui::Button("Keyboard 4 : Scene Collisions Narrow Phase"))
        currentScene = Scene::SCENE_4;
    if (ImGui::Button("Keyboard 5 : Scene Collisions Full"))
        currentScene = Scene::SCENE_5;
	ImGui::End();
}

void ImGuiBroadPhasePanel(PotentialContact* potentialContact, unsigned int potentialContactsCount)
{
    ImGui::Begin("Broad Phase");
    ImGui::Text("Potential Contacts: %d", potentialContactsCount);
    if (potentialContactsCount > 0)
    {
        ImGui::Text("Potential contacts: %s", potentialContact->rigidbodies[0]->name.c_str());
        ImGui::Text("Potential contacts: %s", potentialContact->rigidbodies[1]->name.c_str());
    }
    ImGui::End();
}


//int main(int argc, char** argv)
//{
//
//#pragma region Rigidbody
//    std::shared_ptr<Rigidbody> rigidbodyBox = std::make_shared<Rigidbody>(Transform(Vector3f(0.f, 0.f, 0.f), Quaternionf(1.f, 0.f, 45.f * Deg2Rad, 0.f), Vector3f::One), Vector3f::Zero, Vector3f::Zero, 10.f, Vector3f::Zero, Vector3f::Zero, Vector3f::Zero, "Cube", RigidbodyType::BOX);
//    physics.AddRigidbody(rigidbodyBox);
//
//    std::shared_ptr<Rigidbody> rigidbodyBox2 = std::make_shared<Rigidbody>(Transform(Vector3f(0.f, 5.f, 0.f), Quaternionf(1.f, 0.f, 45.f * Deg2Rad, 0.f), Vector3f::One), Vector3f::Zero, Vector3f::Zero, 10.f, Vector3f::Zero, Vector3f::Zero, Vector3f::Zero, "Cube2", RigidbodyType::BOX);
//    physics.AddRigidbody(rigidbodyBox2);
//
//    std::shared_ptr<Rigidbody> rigidbodyBox3 = std::make_shared<Rigidbody>(Transform(Vector3f(0.f, 8.f, 0.f), Quaternionf(1.f, 0.f, 45.f * Deg2Rad, 0.f), Vector3f::One), Vector3f::Zero, Vector3f::Zero, 10.f, Vector3f::Zero, Vector3f::Zero, Vector3f::Zero, "Cube3", RigidbodyType::BOX);
//    physics.AddRigidbody(rigidbodyBox3);
//
//    std::shared_ptr<Rigidbody> rigidbodyTriangle = std::make_shared<Rigidbody>(Transform(Vector3f(5.f, 5.f, 0.f), Quaternionf(1.f, 0.f, 0.f, 0.f), Vector3f::One), Vector3f::Zero, Vector3f::Zero, 10.f, Vector3f::Zero, Vector3f::Zero, Vector3f::Zero, "Triangle", RigidbodyType::TRIANGLE);
//    physics.AddRigidbody(rigidbodyTriangle);
//
//
//    std::shared_ptr<ForceGravity> forceGravity = std::make_shared<ForceGravity>();
//    std::shared_ptr<ForceDrag> weakForceDrag = std::make_shared<ForceDrag>(5.0f, 0.0f);
//    std::shared_ptr<ForceDrag> strongForceDrag = std::make_shared<ForceDrag>(20.0f, 0.0f);
//
//    std::shared_ptr<ForceAnchoredSpring> forceAnchoredSpringCube = std::make_shared<ForceAnchoredSpring>(10.f, 5.0f, Vector3f::Zero, Vector3f(1.0f, 1.0f, 0.0f));
//    std::shared_ptr<ForceAnchoredSpring> forceAnchoredSpringTriangle = std::make_shared<ForceAnchoredSpring>(10.f, 5.0f, Vector3f::Zero, Vector3f(0.0f, 0.5f, 0.0f));
//
//    forceRegistry->Add(rigidbodyBox2, forceGravity);
//    //forceRegistry->Add(rigidbodyBox3, forceGravity);
//    //forceRegistry->Add(rigidbodyBox, forceGravity);
//    //forceRegistry->Add(rigidbodyBox, strongForceDrag);
//    //forceRegistry->Add(rigidbodyBox, forceAnchoredSpringCube);
//    //
//    //forceRegistry->Add(rigidbodyTriangle, forceGravity);
//    //forceRegistry->Add(rigidbodyTriangle, weakForceDrag);
//    //forceRegistry->Add(rigidbodyTriangle, forceAnchoredSpringTriangle);
//#pragma endregion
//
//#pragma region Narrow Phase
//
//    ContactGenerator contactGenerator = ContactGenerator(50);
//    ContactResolver contactResolver = ContactResolver(50);
//
//    Plane plane = Plane(nullptr, Matrix4f(), Vector3f(0, 1, 0), 0.f);
//
//    Sphere sphere = Sphere(rigidbodyBox2, Matrix4f(), 1.f);
//    Sphere sphere2 = Sphere(rigidbodyBox, Matrix4f(), 1.f);
//
//    Box box = Box(rigidbodyBox, Matrix4f(), Vector3f(0.5f, 0.5f, 0.5f));
//
//#pragma endregion
//
//#pragma region BVH
//
//    //BVHNode<BoundingBox> nodeBox = BVHNode<BoundingBox>(rigidbodyBox, std::make_shared<BoundingBox>(rigidbodyBox->position, rigidbodyBox->scale.x));
//    //Vector3f nodePosition = nodeBox.m_volume->GetCenter();
//
//    //BVHNode<BoundingBox> nodeBox2 = BVHNode<BoundingBox>(rigidbodyBox2, std::make_shared<BoundingBox>(rigidbodyBox2->position, rigidbodyBox2->scale.x));
//    //Vector3f nodePosition2 = nodeBox2.m_volume->GetCenter();
//
//    //nodeBox.Insert(rigidbodyBox2, std::make_shared<BoundingBox>(rigidbodyBox2->position, rigidbodyBox2->scale.x));
//
//    //BVHNode<BoundingSphere> nodeSphere = BVHNode<BoundingSphere>(rigidbodyBox, std::make_shared<BoundingSphere>(rigidbodyBox->position, rigidbodyBox->scale.x));
//    //Vector3f nodePosition = nodeSphere.m_volume->GetCenter();
//    //BVHNode<BoundingSphere> nodeSphere2 = BVHNode<BoundingSphere>(rigidbodyBox2, std::make_shared<BoundingSphere>(rigidbodyBox2->position, rigidbodyBox2->scale.x));
//    //Vector3f nodePosition2 = nodeSphere2.m_volume->GetCenter();
//
//    std::shared_ptr<BVHNode> nodeSphere = std::make_shared<BVHNode>(rigidbodyBox, std::make_shared<BoundingSphere>(rigidbodyBox->position, rigidbodyBox->scale.x));
//    //BVHNode nodeSphere(rigidbodyBox/*, std::make_shared<BoundingSphere>(rigidbodyBox->position, rigidbodyBox->scale.x)*/);
//    Vector3f nodePosition = nodeSphere->m_volume->GetCenter();
//
//    std::cout << "IsLeaf : " << nodeSphere->IsLeaf() << std::endl;
//    std::cout << "Overlaps: " << nodeSphere->Overlaps(nodeSphere) << std::endl;
//
//    std::shared_ptr<BVHNode> nodeSphere2 = std::make_shared<BVHNode>(rigidbodyBox2, std::make_shared<BoundingSphere>(rigidbodyBox2->position, rigidbodyBox2->scale.x));
//    Vector3f nodePosition2 = nodeSphere2->m_volume->GetCenter();
//
//    std::shared_ptr<BVHNode> nodeSphere3 = std::make_shared<BVHNode>(rigidbodyBox3, std::make_shared<BoundingSphere>(rigidbodyBox3->position, rigidbodyBox3->scale.x));
//    Vector3f nodePosition3 = nodeSphere3->m_volume->GetCenter();
//
//    nodeSphere->Insert(nodeSphere2->m_rigidbody, nodeSphere2->m_volume);
//    //nodeSphere->Insert(nodeSphere3->m_rigidbody, nodeSphere3->m_volume);
//
//    physics.AddRootBVHNode(nodeSphere);
//#pragma endregion
//
//#pragma region Timestep
//    double t = 0.0;
//    double deltaTime = 0.01;
//
//    double currentTime = HiresTimeInSeconds();
//    double accumulator = 0.0;
//#pragma endregion
//
//#pragma region Shader
//
//    Shader ourShader("assets/shaders/test.vert", "assets/shaders/test.frag");
//
//#pragma endregion
//
//#pragma region Model
//
//    // Cube
//    float cubeVertices[] = {
//    -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
//     0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
//     0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
//     0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
//    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
//    -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
//
//    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
//     0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
//     0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
//     0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
//    -0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
//    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
//
//    -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
//    -0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
//    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
//    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
//    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
//    -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
//
//     0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
//     0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
//     0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
//     0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
//     0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
//     0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
//
//    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
//     0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
//     0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
//     0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
//    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
//    -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
//
//    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
//     0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
//     0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
//     0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
//    -0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
//    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f
//    };
//
//    glm::vec3 cubePosition = glm::vec3(0.0f, 0.0f, 0.0f);
//    glm::vec3 cubeRotation = glm::vec3(0.0f, 0.0f, 0.0f);
//    std::vector<glm::vec3> cubePositions;
//    cubePositions.push_back(glm::vec3(rigidbodyBox->position.x, rigidbodyBox->position.y, rigidbodyBox->position.z));
//    cubePositions.push_back(glm::vec3(rigidbodyBox2->position.x, rigidbodyBox2->position.y, rigidbodyBox2->position.z));
//    cubePositions.push_back(glm::vec3(rigidbodyBox3->position.x, rigidbodyBox3->position.y, rigidbodyBox3->position.z));
//    std::vector<glm::vec3> cubeRotations;
//    cubeRotations.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
//    cubeRotations.push_back(glm::vec3(0.0f, 45.0f, 0.0f));
//
//
//    GLuint VBO1, VAO1;
//    glGenVertexArrays(1, &VAO1);
//    glGenBuffers(1, &VBO1);
//
//    glBindVertexArray(VAO1);
//
//    glBindBuffer(GL_ARRAY_BUFFER, VBO1);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);
//
//    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
//    glEnableVertexAttribArray(0);
//
//    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
//    glEnableVertexAttribArray(1);
//
//    std::string texturePath = "assets/textures/test.jpg";
//
//    // load and create a texture 
//    // -------------------------
//    unsigned int texture1;
//    // texture 1
//    // ---------
//    glGenTextures(1, &texture1);
//    glBindTexture(GL_TEXTURE_2D, texture1);
//    // set the texture wrapping parameters
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
//    // set texture filtering parameters
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//    // load image, create texture and generate mipmaps
//    int width, height, nrChannels;
//    stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.
//    // The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
//    unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &nrChannels, 0);
//    if (data)
//    {
//        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
//        glGenerateMipmap(GL_TEXTURE_2D);
//    }
//    else
//    {
//        std::cout << "Failed to load texture" << std::endl;
//    }
//    stbi_image_free(data);
//
//    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
//    glBindBuffer(GL_ARRAY_BUFFER, 0);
//
//    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
//    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
//    glBindVertexArray(0);
//
//    // vertices for a triangle pyramidal from :
//    // https://stackoverflow.com/questions/22714866/to-draw-tetrahedron-using-opengl-in-android-with-different-color-faces-but-gray
//    float triangleVertices[] = {
//
//        // FRONT Face
//        -1.0f, -1.0f,  1.0f,
//        1.0f, -1.0f,  1.0f,
//        1.0f,  1.0f,  1.0f,
//
//        //Right face
//        -1.0f, -1.0f,  1.0f,
//        1.0f,  1.0f, 1.0f,
//        -1.0f, -1.0f, -1.0f,
//
//        // Left Face
//        1.0, -1.0f, 1.0f,
//        -1.0f, -1.0f, -1.0f,
//        1.0f,  1.0f, 1.0f,
//
//        // BOTTOM
//        -1.0f, -1.0f, 1.0f,
//        -1.0f, -1.0f, -1.0f,
//        1.0f, -1.0f,  1.0f
//    };
//
//    glm::vec3 trianglePosition = glm::vec3(rigidbodyTriangle->position.x, rigidbodyTriangle->position.y, rigidbodyTriangle->position.z);
//    glm::vec3 triangleRotation = glm::vec3(0.0f, 0.0f, 0.0f);
//
//    // Create a VAO and VBO for Model 2
//    GLuint VAO2, VBO2;
//    glGenVertexArrays(1, &VAO2);
//    glGenBuffers(1, &VBO2);
//
//    glBindVertexArray(VAO2);
//    glBindBuffer(GL_ARRAY_BUFFER, VBO2);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(triangleVertices), triangleVertices, GL_STATIC_DRAW);
//    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
//    glEnableVertexAttribArray(0);
//
//    glBindBuffer(GL_ARRAY_BUFFER, 0);
//    glBindVertexArray(0);
//
//    // Create a UV Sphere
//    std::vector<glm::vec3> sphereVertices;
//    CreateSphere(sphereVertices, 1.0f, 30, 30);
//    std::vector<glm::vec3> spherePositions;
//    spherePositions.push_back(glm::vec3(nodePosition.x, nodePosition.y, nodePosition.z));
//    spherePositions.push_back(glm::vec3(nodePosition2.x, nodePosition2.y, nodePosition2.z));
//    //spherePositions.push_back(glm::vec3(nodePosition3.x, nodePosition3.y, nodePosition3.z));
//    glm::vec3 sphereRotation = glm::vec3(0.0f, 0.0f, 0.0f);
//    GLuint VAO3, VBO3;
//    glGenVertexArrays(1, &VAO3);
//    glGenBuffers(1, &VBO3);
//
//    glBindVertexArray(VAO3);
//    glBindBuffer(GL_ARRAY_BUFFER, VBO3);
//    glBufferData(GL_ARRAY_BUFFER, sphereVertices.size() * sizeof(glm::vec3), sphereVertices.data(), GL_STATIC_DRAW);
//
//    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
//    glEnableVertexAttribArray(0);
//
//    //glBindBuffer(GL_ARRAY_BUFFER, 0);
//    //glBindVertexArray(0);
//#pragma endregion
//
//    bool isGravityEnabled = true;
//    while (!window.ShouldClose())
//    {
//        double newTime = HiresTimeInSeconds();
//        double frameTime = newTime - currentTime;
//
//        // Cap the frame time to avoid spiral of death
//        if (frameTime > 0.015)
//            frameTime = 0.015;
//
//        currentTime = newTime;
//        accumulator += frameTime;
//
//        while (accumulator >= deltaTime) {
//            // Integrates the physics
//            physics.Update(deltaTime, isGravityEnabled);
//            t += deltaTime;
//            accumulator -= deltaTime;
//        }
//
//        // Use alpha with rneder to interpolate between the previous and current physics state
//        const double alpha = accumulator / deltaTime;
//        // state = currentState * alpha + previousState * (1.0 - alpha);
//
//        ProcessInput(window.GetHandle(), deltaTime);
//
//        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//        ourShader.Use();
//
//        glm::mat4 model = glm::mat4(1.0f);
//        glm::mat4 model2 = glm::mat4(1.0f);
//        glm::mat4 projection = glm::perspective(glm::radians(45.f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
//        ourShader.SetMat4("projection", projection);
//
//        glm::mat4 view = camera.GetViewMatrix();
//        ourShader.SetMat4("view", view);
//
//        glActiveTexture(GL_TEXTURE0);
//        glBindTexture(GL_TEXTURE_2D, texture1);
//
//        cubePositions[0] = glm::vec3(rigidbodyBox->position.x, rigidbodyBox->position.y, rigidbodyBox->position.z);
//        cubePositions[1] = glm::vec3(rigidbodyBox2->position.x, rigidbodyBox2->position.y, rigidbodyBox2->position.z);
//        cubePositions[2] = glm::vec3(rigidbodyBox3->position.x, rigidbodyBox3->position.y, rigidbodyBox3->position.z);
//        cubeRotation = glm::vec3(rigidbodyBox->rotation.GetX(), rigidbodyBox->rotation.GetY(), rigidbodyBox->rotation.GetZ());
//
//        trianglePosition = glm::vec3(rigidbodyTriangle->position.x, rigidbodyTriangle->position.y, rigidbodyTriangle->position.z);
//        triangleRotation = glm::vec3(rigidbodyTriangle->rotation.GetX(), rigidbodyTriangle->rotation.GetY(), rigidbodyTriangle->rotation.GetZ());
//        
//        // Rotate sphere on Y axis
//        spherePositions[0] = glm::vec3(nodePosition.x, nodePosition.y, nodePosition.z);
//        spherePositions[1] = glm::vec3(rigidbodyBox2->position.x, rigidbodyBox2->position.y, rigidbodyBox2->position.z);
//        //spherePositions[2] = glm::vec3(rigidbodyBox3->position.x, rigidbodyBox3->position.y, rigidbodyBox3->position.z);
//        sphereRotation = glm::vec3(sphereRotation.x, sphereRotation.y + 0.01f, sphereRotation.z);
//        
//        glm::mat4 sModel = glm::mat4(1.0f);
//        glm::vec3 spherePos = glm::vec3(0.f, 0.f, 0.f);
//        glm::vec3 sphereRot;
//
//        glBindVertexArray(VAO3);
//
//        sModel = glm::mat4(1.0f);
//        //spherePos = glm::vec3(sphere.rigidbody->position.x, sphere.rigidbody->position.y, sphere.rigidbody->position.z);
//        //sphereRot = glm::vec3(sphere.rigidbody->rotation.GetX(), sphere.rigidbody->rotation.GetY(), sphere.rigidbody->rotation.GetZ());
//        sModel = glm::translate(sModel, spherePos);
//        //sModel = glm::rotate(sModel, sphereRot.y, glm::vec3(0.0f, 1.0f, 0.0f));
//        ourShader.SetMat4("model", sModel);
//
//        glDrawArrays(GL_LINE_STRIP, 0, sphereVertices.size());
//
//        imguiCpp.NewFrame();
//        imguiCpp.Render();
//
//        glfwSwapBuffers(window.GetHandle());
//        glfwPollEvents();
//    }
//
//    glDeleteVertexArrays(1, &VAO1);
//    glDeleteBuffers(1, &VBO1);
//    glDeleteVertexArrays(1, &VAO2);
//    glDeleteBuffers(1, &VBO2);
//    glDeleteVertexArrays(1, &VAO3);
//    glDeleteBuffers(1, &VBO3);
//
//    return 0;
//}
//
//void FramebufferSizeCallback(GLFWwindow* window, int width, int height)
//{
//    glViewport(0, 0, width, height);
//}
//
//void ImguiGamePanel(std::shared_ptr<Particle> particle, PhysicsSystem& physics, Vector3f& direction, float& power, bool& isParticleLaunched, bool& isGravityEnabled, float deltaTime)
//{
//    float directionAngle = 0.f;
//
//    ImGui::Begin("Game panel");
//    ImGui::Text("Launch a particle");
//
//    ImGui::BeginGroup();
//    ImGui::SliderFloat("Dir X", &direction.x, 0.f, 1.f);
//    ImGui::SliderFloat("Dir Y", &direction.y, 0.f, 1.f);
//    ImGui::EndGroup();
//
//    ImGui::Spacing();
//    ImGui::SliderFloat("Power", &power, 0.f, 8.f);
//    ImGui::Spacing();
//
//    ImGui::Spacing();
//    ImGui::Checkbox("Gravity", &isGravityEnabled);
//    ImGui::Spacing();
//
//    ImGui::BeginGroup();
//    if (!isParticleLaunched && ImGui::Button("Launch"))
//    {
//        std::cout << "Launch the particle" << std::endl;
//
//        particle->velocity = direction.GetUnitNormalized() * power;
//
//        physics.AddParticle(particle);
//
//        isParticleLaunched = true;
//    }
//    if (isParticleLaunched && ImGui::Button("Reset"))
//    {
//        std::cout << "Reset the particle" << std::endl;
//
//        physics.RemoveParticle(particle);
//        particle->position = Vector3f(0, 0, 0);
//        particle->velocity = Vector3f(0, 0, 0);
//
//        isParticleLaunched = false;
//    }
//    ImGui::EndGroup();
//    ImGui::End();
//}
//
//void ImguiStatsPanel(float deltaTime)
//{
//    ImGui::Begin("Stats Panel");
//    ImGui::Text("Delta Time: %f", deltaTime);
//    ImGui::Text("FPS: %.f", std::clamp(1000 / (deltaTime * 1000), 0.f, 60.f));
//    ImGui::End();
//}
//
//void Vector3ClassTest()
//{
//    /* Vector3 class test */
//    Vector3f testVec(1.f, 1.f, 0.f);
//    std::cout << "TestVec: " << testVec << std::endl;
//    std::cout << "TestVec length: " << testVec.GetLength() << std::endl;
//    std::cout << "TestVec normalized: " << testVec.GetNormalized() << std::endl;
//    std::cout << "TestVec unit normalized: " << testVec.GetUnitNormalized() << std::endl;
//    std::cout << std::endl;
//
//    Vector3f t(1, 2, 3);
//    Vector3f t2(4, 5, 6);
//
//    std::cout << "Cross product: " << t << ";" << t2 << " = " << Vector3f::CrossProduct(t, t2) << std::endl;
//    std::cout << "Dot product: " << t << ";" << t2 << " = " << Vector3f::DotProduct(t, t2) << std::endl;
//    std::cout << std::endl;
//}
//
//double HiresTimeInSeconds() {
//    using namespace std::chrono;
//    using clock = high_resolution_clock;
//
//    auto currentTime = duration_cast<milliseconds>(clock::now().time_since_epoch()).count();
//    return currentTime / 1000.0;
//}
//
//void ProcessInput(GLFWwindow* window, float deltaTime)
//{
//    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
//        glfwSetWindowShouldClose(window, true);
//
//    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
//        camera.ProcessKeyboard(FORWARD, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
//        camera.ProcessKeyboard(BACKWARD, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
//        camera.ProcessKeyboard(LEFT, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
//        camera.ProcessKeyboard(RIGHT, deltaTime);
//}
//
//void MouseCallback(GLFWwindow* window, double xposIn, double yposIn)
//{
//    float xpos = static_cast<float>(xposIn);
//    float ypos = static_cast<float>(yposIn);
//
//    if (firstMouse)
//    {
//        lastX = xpos;
//        lastY = ypos;
//        firstMouse = false;
//    }
//
//    float xoffset = xpos - lastX;
//    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
//
//    lastX = xpos;
//    lastY = ypos;
//
//    camera.ProcessMouseMovement(xoffset, yoffset);
//}
//
//void ContactsGenerator(std::vector<std::shared_ptr<ParticleContact>>& contactArray, std::vector<std::shared_ptr<ParticleContactGenerator>>& contacts)
//{
//    if(contacts.size() == 0)
//		return;
//
//    for (int i = 0; i < contacts.size(); i++)
//    {
//        contacts.at(i)->AddContact(contactArray, 0);
//    }
//}
//
//void ImguiSceneSelection() {
//    // ImGui code for scene selection
//    ImGui::Begin("Camera");
//    ImGui::Text("Camera position: (%f, %f, %f)", camera.Position.x, camera.Position.y, camera.Position.z);
//    ImGui::Text("Camera rotation: (%f, %f)", camera.Yaw, camera.Pitch);
//    ImGui::End();
//    ImGui::Begin("Scene Selection");
//    ImGui::Text("Current Scene: %d", (int)currentScene + 1);
//    if (ImGui::Button("Scene 1"))
//        currentScene = Scene::SCENE_1;
//    if (ImGui::Button("Scene 2"))
//        currentScene = Scene::SCENE_2;
//    if (ImGui::Button("Scene 3"))
//        currentScene = Scene::SCENE_3;
//    ImGui::End();
//}
//
//void ImguiRigidbodyData(std::shared_ptr<Rigidbody> rigidbody)
//{
//    ImGui::Begin("Rigidbody Data");
//    ImGui::Text("%s", rigidbody->name.c_str());
//    ImGui::Text("%s position: (%f, %f, %f)", rigidbody->name.c_str(), rigidbody->position.x, rigidbody->position.y, rigidbody->position.z);
//    ImGui::Text("%s velocity: (%f, %f, %f)", rigidbody->name.c_str(), rigidbody->velocity.x, rigidbody->velocity.y, rigidbody->velocity.z);
//    ImGui::Text("%s acceleration: (%f, %f, %f)", rigidbody->name.c_str(), rigidbody->GetAcceleration().x, rigidbody->GetAcceleration().y, rigidbody->GetAcceleration().z);
//    ImGui::Text("%s mass: %f", rigidbody->name.c_str(), rigidbody->mass);
//    ImGui::Text("%s rotation (rad): (%f, %f, %f) %f", rigidbody->name.c_str(), rigidbody->rotation.GetX(), rigidbody->rotation.GetY(), rigidbody->rotation.GetZ(), rigidbody->rotation.GetS());
//    ImGui::Text("%s angular velocity: (%f, %f, %f)", rigidbody->name.c_str(), rigidbody->angularVelocity.x, rigidbody->angularVelocity.y, rigidbody->angularVelocity.z);
//    ImGui::Text("%s angular acceleration: (%f, %f, %f)", rigidbody->name.c_str(), rigidbody->GetAngularAcceleration().x, rigidbody->GetAngularAcceleration().y, rigidbody->GetAngularAcceleration().z);
//    ImGui::Text("%s inverseMass: %f", rigidbody->name.c_str(), rigidbody->inverseMass);
//    ImGui::Text("%s transformMatrix:\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f",
//        rigidbody->name.c_str(),
//        rigidbody->transformMatrix.Value(0, 0), rigidbody->transformMatrix.Value(0, 1), rigidbody->transformMatrix.Value(0, 2), rigidbody->transformMatrix.Value(0, 3),
//        rigidbody->transformMatrix.Value(1, 0), rigidbody->transformMatrix.Value(1, 1), rigidbody->transformMatrix.Value(1, 2), rigidbody->transformMatrix.Value(1, 3),
//        rigidbody->transformMatrix.Value(2, 0), rigidbody->transformMatrix.Value(2, 1), rigidbody->transformMatrix.Value(2, 2), rigidbody->transformMatrix.Value(2, 3),
//        rigidbody->transformMatrix.Value(3, 0), rigidbody->transformMatrix.Value(3, 1), rigidbody->transformMatrix.Value(3, 2), rigidbody->transformMatrix.Value(3, 3));
//    ImGui::End();
//}
//
//// Function to create a UV Sphere
//void CreateSphere(std::vector<glm::vec3>& vertices, float radius, int slices, int stacks) 
//{
//    // Create vertices
//    for (int i = 0; i <= stacks; ++i)
//    {
//        float stackAngle = glm::pi<float>() * static_cast<float>(i) / static_cast<float>(stacks);
//        float stackRadius = glm::sin(stackAngle);
//        float stackHeight = glm::cos(stackAngle);
//
//        for (int j = 0; j <= slices; ++j)
//        {
//            float sliceAngle = 2.0f * glm::pi<float>() * static_cast<float>(j) / static_cast<float>(slices);
//            float x = stackRadius * glm::cos(sliceAngle);
//            float y = stackRadius * glm::sin(sliceAngle);
//            float z = stackHeight;
//
//            vertices.emplace_back(x * radius, y * radius, z * radius);
//        }
//    }
//}
//
//void GameLoop()
//{
//
//}