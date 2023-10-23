#version 330 core
layout(location = 0) out vec4 FragColor;
in vec3 ourColor;

void main()
{
    FragColor = vec4(0.6, 0.5, 0.5, 1.0f);
}