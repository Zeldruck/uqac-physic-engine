#pragma once
#include <vector>
#include "Vector3.hpp"
#include "Quaternion.hpp"

struct State
{
    std::vector<Vector3f> m_particlePositions;

    std::vector<Vector3f> m_rigidbodyPositions;
    std::vector<Quaternionf> m_rigidbodyRotations;

    State operator*(double a) const
    {
        std::vector<Vector3f> tempParticlePositions;
        std::vector<Vector3f> tempRigidbodyPositions;
        std::vector<Quaternionf> tempRigidbodyRotations;

        for (auto& particlePosition : m_particlePositions)
            tempParticlePositions.push_back(particlePosition * a);
        for (auto& rigidbodyPositions : m_rigidbodyPositions)
            tempRigidbodyPositions.push_back(rigidbodyPositions * a);
        for (auto& rigidbodyRotation : m_rigidbodyRotations)
            tempRigidbodyRotations.push_back(rigidbodyRotation * a);

        return { tempParticlePositions, tempRigidbodyPositions, tempRigidbodyRotations };
    }

    State operator+(const State& rhs) const
    {
        std::vector<Vector3f> tempParticlePositions;
        std::vector<Vector3f> tempRigidbodyPositions;
        std::vector<Quaternionf> tempRigidbodyRotations;

        for (int i = 0; i < m_particlePositions.size() && i < rhs.m_particlePositions.size(); ++i)
            tempParticlePositions.push_back(m_particlePositions[i] + rhs.m_particlePositions[i]);
        for (int i = 0; i < m_rigidbodyPositions.size() && i < rhs.m_rigidbodyPositions.size(); ++i)
            tempRigidbodyPositions.push_back(m_rigidbodyPositions[i] + rhs.m_rigidbodyPositions[i]);
        for (int i = 0; i < m_rigidbodyRotations.size() && i < rhs.m_rigidbodyRotations.size(); ++i)
            tempRigidbodyRotations.push_back(m_rigidbodyRotations[i] + rhs.m_rigidbodyRotations[i]);

        return { tempParticlePositions, tempRigidbodyPositions, tempRigidbodyRotations };
    }
};