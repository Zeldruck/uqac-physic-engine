#pragma once

#include <string>
#include <glm/gtc/matrix_transform.hpp>
#include "Vector3.hpp"

class Shader
{
public:
    unsigned int ID;

    Shader(const char* vertexPath, const char* fragmentPath, bool isFile = true);
    // Activate the shader
    void Use();


    void SetBool(const std::string& name, bool value) const;
    void SetInt(const std::string& name, int value) const;
    void SetFloat(const std::string& name, float value) const;
    void SetVec3(const std::string& name, Vector3f value) const;
    void SetMat4(const std::string& name, glm::mat4 value) const;

private:
    void CheckCompileErrors(unsigned int shader, std::string type);
};