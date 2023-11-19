#include "Primitives/Sphere.hpp"
#include "Constants/MathConstants.hpp"
#include <vector>
#include <GLFW/glfw3.h>
#include <glad/glad.h>

Sphere::Sphere(float radius, unsigned int num)
{
    isInited = false;
    m_vao = 0;
    m_vboVertex = 0;
    m_vboIndex = 0;
    numsToDraw = 0;

    lats = 40;
    longs = 40;
}

void Sphere::Init(unsigned int vertexPositionID)
{
    int i, j;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;
    int indicator = 0;

    for (i = 0; i <= lats; i++)
    {
        float lat0 = PI * (-0.5 + (float)(i - 1) / lats);
        float z0 = sin(lat0);
        float zr0 = cos(lat0);

        float lat1 = PI * (-0.5 + (float)i / lats);
        float z1 = sin(lat1);
        float zr1 = cos(lat1);

        for (j = 0; j <= longs; j++)
        {
            float lng = 2 * PI * (j - 1) / longs;
            float x = cos(lng);
            float y = sin(lng);

            vertices.push_back(x * zr0);
            vertices.push_back(y * zr0);
            vertices.push_back(z0);
            indices.push_back(indicator);
            indicator++;

            vertices.push_back(x * zr1);
            vertices.push_back(y * zr1);
            vertices.push_back(z1);
            indices.push_back(indicator);
            indicator++;
        }

        indices.push_back(GL_PRIMITIVE_RESTART_FIXED_INDEX);
    }

    glGenVertexArrays(1, &m_vao);
    glBindVertexArray(m_vao);

    glGenBuffers(1, &m_vboVertex);
    glBindBuffer(GL_ARRAY_BUFFER, m_vboVertex);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat), &vertices[0], GL_STATIC_DRAW);

    glVertexAttribPointer(vertexPositionID, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(vertexPositionID);

    glGenBuffers(1, &m_vboIndex);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vboIndex);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);

    numsToDraw = indices.size();

    isInited = true;
}

void Sphere::Clean()
{
    if (!isInited) return;

    if (m_vboVertex) glDeleteBuffers(1, &m_vboVertex);

    if (m_vboIndex) glDeleteBuffers(1, &m_vboIndex);

    if (m_vao) glDeleteVertexArrays(1, &m_vao);

    isInited = false;
    m_vao = 0;
    m_vboVertex = 0;
    m_vboIndex = 0;
}

void Sphere::Draw()
{
    if (!isInited) return;

    glBindVertexArray(m_vao);
    glEnable(GL_PRIMITIVE_RESTART);
    glPrimitiveRestartIndex(GL_PRIMITIVE_RESTART_FIXED_INDEX);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vboIndex);
    glDrawElements(GL_QUAD_STRIP, numsToDraw, GL_UNSIGNED_INT, NULL);
}
