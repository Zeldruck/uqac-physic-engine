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
#include "State.hpp"

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
Camera camera(glm::vec3(0.0f, 5.0f, 40.0f));

float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;
#pragma endregion

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
void ImGuiBroadPhasePanel(PotentialContact* potentialContact, unsigned int potentialContactsCount, PotentialContactPrimitive* potentialContactPrimitive, unsigned int potentialContactPrimitiveCount);
void ImGuiNarrowPhasePanel(std::vector<std::shared_ptr<Contact>> contacts, int contactCount);

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

    Scene currentScene = Scene::SCENE_5;
#pragma region Loop
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
            physics.Update(current, dt, true, false);
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
            physics.Update(current, dt, true, false);
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
    std::shared_ptr<Rigidbody> rigidbody3 = std::make_shared<Rigidbody>("Rigidbody 3 Sphere", Vector3f::Up * 5.0f);

    physics.AddRigidbody(rigidbody1);
    physics.AddRigidbody(rigidbody2);
    physics.AddRigidbody(rigidbody3);

    std::shared_ptr<Sphere> sphere = std::make_shared<Sphere>(rigidbody1, Matrix4f(), 1.f);
    std::shared_ptr<Box> box = std::make_shared<Box>(rigidbody2, Matrix4f(), Vector3f(0.5f, 0.5f, 0.5f));
    std::shared_ptr<Sphere> sphere2 = std::make_shared<Sphere>(rigidbody3, Matrix4f(), 1.f);
#pragma endregion

#pragma region Forces
    std::shared_ptr<ForceGravity> gravity = std::make_shared<ForceGravity>();
    std::shared_ptr<ForceDrag> drag = std::make_shared<ForceDrag>(10.0f, 0.0f);

    forceRegistry->Add(rigidbody1, gravity);
    forceRegistry->Add(rigidbody1, drag);
#pragma endregion

#pragma region BroadPhase

    std::shared_ptr<BoundingSphere> boundingSphere1 = std::make_shared<BoundingSphere>(rigidbody1);
    rigidbody1->m_boundingSphere = boundingSphere1;

    std::shared_ptr<BoundingSphere> boundingSphere2 = std::make_shared<BoundingSphere>(rigidbody2);
    rigidbody2->m_boundingSphere = boundingSphere2;

    std::shared_ptr<BoundingSphere> boundingSphere3 = std::make_shared<BoundingSphere>(rigidbody3);
    rigidbody3->m_boundingSphere = boundingSphere3;


    std::shared_ptr<BVHNode> bvhRoot = std::make_shared<BVHNode>(sphere);
    bvhRoot->Insert(box, boundingSphere2);
    bvhRoot->Insert(sphere2, boundingSphere3);

    physics.AddRootBVHNode(bvhRoot);
    PotentialContact* potentialContacts = new PotentialContact;
    PotentialContactPrimitive* potentialContactsPrimitive = new PotentialContactPrimitive;
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
    spherePositions.push_back(glm::vec3(rigidbody3->position.x, rigidbody3->position.y, rigidbody3->position.z));

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
            physics.Update(current, dt, true, true);
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
        //std::vector<glm::vec3> boundingSphereVertices;
        //CreateSphere(boundingSphereVertices, bvhRoot->m_volume->GetRadius() * 2.0f + 0.5f, 30, 30);

        //std::vector<glm::vec3> boundingSpherePositions;
        //boundingSpherePositions.push_back(glm::vec3(bvhRoot->m_volume->GetCenter().x, bvhRoot->m_volume->GetCenter().y, bvhRoot->m_volume->GetCenter().z));

        //glBindVertexArray(VAO3);
        //glBufferData(GL_ARRAY_BUFFER, boundingSphereVertices.size() * sizeof(glm::vec3), boundingSphereVertices.data(), GL_STATIC_DRAW);

        //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        //glEnableVertexAttribArray(0);

        glm::mat4 projection = glm::perspective(glm::radians(45.f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        ourShader.SetMat4("projection", projection);
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.SetMat4("view", view);

        // Update Spheres Positions
        spherePositions[0] = glm::vec3(rigidbody1->position.x, rigidbody1->position.y, rigidbody1->position.z);
        spherePositions[1] = glm::vec3(rigidbody2->position.x, rigidbody2->position.y, rigidbody2->position.z);
        spherePositions[2] = glm::vec3(rigidbody3->position.x, rigidbody3->position.y, rigidbody3->position.z);

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
        //for (int i = 0; i < boundingSpherePositions.size(); ++i)
        //{
        //    model = glm::mat4(1.0f); // Initialize model matrix for each object
        //    model = glm::translate(model, boundingSpherePositions[i]);
        //    ourShader.SetMat4("model", model);
        //    glBindVertexArray(VAO3);
        //    glDrawArrays(GL_LINE_STRIP, 0, boundingSphereVertices.size());
        //}
 
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
        ImGuiBroadPhasePanel(physics.GetPotentialContactArray(), physics.GetPotentialContactCount(), physics.GetPotentialContactPrimitiveArray(), physics.GetPotentialContactPrimitiveCount());
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
    delete(potentialContactsPrimitive);
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

#pragma region Narrow Phase

    ContactGenerator contactGenerator = ContactGenerator(50);
    ContactResolver contactResolver = ContactResolver(50);

    Plane plane = Plane(nullptr, Matrix4f(), Vector3f(0, 1, 0), 0.f);

    Sphere sphere = Sphere(rigidbody1, Matrix4f(), 1.f);
    Sphere sphere2 = Sphere(rigidbody2, Matrix4f(), 1.f);

    Box box = Box(rigidbody1, Matrix4f(), Vector3f(0.5f, 0.5f, 0.5f));
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
            physics.Update(current, dt, true);
            accumulator -= dt;
            t += dt;
        }

        const double alpha = accumulator / dt;
        State state = current * alpha + previous * (1.0 - alpha);

        //contactGenerator.DetectSandHS(sphere, plane);
        //contactGenerator.DetectBandP(box, plane);
        //contactGenerator.DetectSandB(sphere, box);
        contactGenerator.DetectSandHS(sphere, plane);
        contactResolver.ResolveContacts(contactGenerator.GetContacts(), dt, state);

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
            model = glm::scale(model, glm::vec3(10, 0.5f, 10));
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
        ImGuiScene4Panel(physics.GetRigidbodies(), cubePositions);
        //ImGuiNarrowPhasePanel(physics.GetContactsArray(), physics.GetContactCount());
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
    std::shared_ptr<Rigidbody> rigidbody1 = std::make_shared<Rigidbody>("Rigidbody 1 Sphere", Vector3f::Up * 13.0f);
    std::shared_ptr<Rigidbody> rigidbody2 = std::make_shared<Rigidbody>("Rigidbody 2 Plane", RigidbodyType::CUBE, Vector3f::Zero, Vector3f(2.0f, .0f, 2.0f), 1.0f);

    physics.AddRigidbody(rigidbody1);
    physics.AddRigidbody(rigidbody2);

    std::shared_ptr<Sphere> sphere = std::make_shared<Sphere>(rigidbody1, Matrix4f(), 1.f);
    std::shared_ptr<Plane> plane = std::make_shared<Plane>(rigidbody2, Matrix4f(), Vector3f(0, 1, 0), 0.f);
#pragma endregion

#pragma region Forces
    std::shared_ptr<ForceGravity> gravity = std::make_shared<ForceGravity>();
    std::shared_ptr<ForceDrag> drag = std::make_shared<ForceDrag>(1.f, 0.f);

    forceRegistry->Add(rigidbody1, gravity);
    forceRegistry->Add(rigidbody1, drag);
#pragma endregion

#pragma region BroadPhase

    std::shared_ptr<BoundingSphere> boundingSphere1 = std::make_shared<BoundingSphere>(rigidbody1);
    rigidbody1->m_boundingSphere = boundingSphere1;

    std::shared_ptr<BoundingSphere> boundingSphere2 = std::make_shared<BoundingSphere>(rigidbody2);
    rigidbody2->m_boundingSphere = boundingSphere2;

    std::shared_ptr<BVHNode> bvhRoot = std::make_shared<BVHNode>(sphere);
    bvhRoot->Insert(plane, boundingSphere2);

    physics.AddRootBVHNode(bvhRoot);
    PotentialContact* potentialContacts = new PotentialContact;
    PotentialContactPrimitive* potentialContactsPrimitive = new PotentialContactPrimitive;
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
            physics.Update(current, dt, true, true, true, true);
            accumulator -= dt;
            t += dt;
        }

        const double alpha = accumulator / dt;
        State state = current * alpha + previous * (1.0 - alpha);
        physics.m_contactGenerator->DetectSandHS(*sphere, *plane);
        physics.m_contactResolver->ResolveContacts(physics.m_contactGenerator->GetContacts(), dt, state);
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
            model = glm::scale(model, glm::vec3(10.f, 0.5f, 10.f));
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
        ImGuiScene5Panel(physics.GetRigidbodies(), cubePositions);
        ImGuiBroadPhasePanel(physics.GetPotentialContactArray(), physics.GetPotentialContactCount(), physics.GetPotentialContactPrimitiveArray(), physics.GetPotentialContactPrimitiveCount());
        // contacts get clear during physics update
        //ImGuiNarrowPhasePanel(physics.GetContactsArray(), physics.GetContactCount());
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
    delete(potentialContactsPrimitive);
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

void ImGuiBroadPhasePanel(PotentialContact* potentialContact, unsigned int potentialContactsCount, PotentialContactPrimitive* potentialContactPrimitive, unsigned int potentialContactPrimitiveCount)
{
    ImGui::Begin("Broad Phase");
    //ImGui::Text("Potential Contacts: %d", potentialContactsCount);
    //if (potentialContactsCount > 0)
    //{
    //    for (unsigned int i = 0; i < potentialContactsCount; i++)
    //    {
    //        ImGui::Text("Potential contacts: %s", potentialContact->rigidbodies[0]->name.c_str());
    //        ImGui::Text("Potential contacts: %s", potentialContact->rigidbodies[1]->name.c_str());
    //        ImGui::Separator();
    //    }
    //}
    //ImGui::Separator();
    //ImGui::Separator();
    ImGui::Text("Potential Contacts Primitive: %d", potentialContactPrimitiveCount);
    if (potentialContactPrimitiveCount > 0)
    {
        for (unsigned int i = 0; i < potentialContactPrimitiveCount; i++)
        {
            ImGui::Text("Potential contacts primitive: %s", potentialContactPrimitive->primitives[0]->rigidbody->name.c_str());
            ImGui::Text("Potential contacts primitive: %s", potentialContactPrimitive->primitives[1]->rigidbody->name.c_str());
            ImGui::Separator();
        }
    }
    ImGui::End();
}

void ImGuiNarrowPhasePanel(std::vector<std::shared_ptr<Contact>> contacts, int contactCount)
{
    ImGui::Begin("Narrow Phase");
    ImGui::Text("Contacts: %d", contactCount);
    if (contactCount > 0)
    {
        for (auto& contact : contacts)
        {
            ImGui::Text("Contact");
            for (auto& rigibody : contact->rigidbodies)
            {
                ImGui::Text("%s", rigibody->name.c_str());
            }
            ImGui::Separator();
        }
	}
	ImGui::End();
}