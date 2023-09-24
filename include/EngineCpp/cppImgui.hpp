#pragma once

struct ImGuiContext;
class cppGLFWwindow;

class ImguiCpp
{
public:
	ImguiCpp(cppGLFWwindow* window);
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