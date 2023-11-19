#include "Primitives/Cube.hpp"
#include "Transform.hpp"

#include <GLFW/glfw3.h>
#include <glad/glad.h>

Cube::Cube(float radius)
{
    m_radius = radius;

    m_vertices = {
    -radius, -radius, -radius,
     radius, -radius, -radius,
     radius,  radius, -radius,
     radius,  radius, -radius,
    -radius,  radius, -radius,
    -radius, -radius, -radius,

    -radius, -radius,  radius,
     radius, -radius,  radius,
     radius,  radius,  radius,
     radius,  radius,  radius,
    -radius,  radius,  radius,
    -radius, -radius,  radius,

    -radius,  radius,  radius,
    -radius,  radius, -radius,
    -radius, -radius, -radius,
    -radius, -radius, -radius,
    -radius, -radius,  radius,
    -radius,  radius,  radius,

     radius,  radius,  radius,
     radius,  radius, -radius,
     radius, -radius, -radius,
     radius, -radius, -radius,
     radius, -radius,  radius,
     radius,  radius,  radius,

    -radius, -radius, -radius,
     radius, -radius, -radius,
     radius, -radius,  radius,
     radius, -radius,  radius,
    -radius, -radius,  radius,
    -radius, -radius, -radius,

    -radius,  radius, -radius,
     radius,  radius, -radius,
     radius,  radius,  radius,
     radius,  radius,  radius,
    -radius,  radius,  radius,
    -radius,  radius, -radius,
    };

    m_indices = {
     0.0f, 0.0f,
     1.0f, 0.0f,
     1.0f, 1.0f,
     1.0f, 1.0f,
     0.0f, 1.0f,
     0.0f, 0.0f,

     0.0f, 0.0f,
     1.0f, 0.0f,
     1.0f, 1.0f,
     1.0f, 1.0f,
     0.0f, 1.0f,
     0.0f, 0.0f,

     1.0f, 0.0f,
     1.0f, 1.0f,
     0.0f, 1.0f,
     0.0f, 1.0f,
     0.0f, 0.0f,
     1.0f, 0.0f,

     1.0f, 0.0f,
     1.0f, 1.0f,
     0.0f, 1.0f,
     0.0f, 1.0f,
     0.0f, 0.0f,
     1.0f, 0.0f,

     0.0f, 1.0f,
     1.0f, 1.0f,
     1.0f, 0.0f,
     1.0f, 0.0f,
     0.0f, 0.0f,
     0.0f, 1.0f,

     0.0f, 1.0f,
     1.0f, 1.0f,
     1.0f, 0.0f,
     1.0f, 0.0f,
     0.0f, 0.0f,
     0.0f, 1.0f
    };
}

void Cube::Draw()
{

}