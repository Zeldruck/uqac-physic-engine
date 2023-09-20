#pragma once

struct ImGuiContext;
struct GLFWwindow;

class ImguiCpp
{
public:
	ImguiCpp(GLFWwindow* window);
	ImguiCpp(const ImguiCpp&) = delete;
	ImguiCpp(ImguiCpp&&) = delete;
	~ImguiCpp();

	void Render();

	void NewFrame();

	//void ProcessEvent();

public:
	ImGuiContext* GetContext();

	ImguiCpp& operator=(const ImguiCpp&) = delete;
	ImguiCpp& operator=(ImguiCpp&&) = delete;

private:
	ImGuiContext* m_context;
};