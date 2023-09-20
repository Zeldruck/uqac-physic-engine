#include "EngineCpp/cppImgui.hpp"
#include "EngineCpp/cppGLFWwindow.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"



ImguiCpp::ImguiCpp(cppGLFWwindow* window)
{
	// Setup imgui
	IMGUI_CHECKVERSION();
	m_context = ImGui::CreateContext();

	ImGui::StyleColorsDark();

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window->GetHandle(), true);
	ImGui_ImplOpenGL3_Init("#version 130");
}

ImguiCpp::~ImguiCpp()
{
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext(m_context);
}

void ImguiCpp::Render()
{
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImguiCpp::NewFrame()
{
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

ImGuiContext* ImguiCpp::GetContext()
{
	return m_context;
}
