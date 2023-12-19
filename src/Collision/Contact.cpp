#include "Collision/Contact.hpp"
#include "Rigidbody.hpp"
#include <array>

Contact::Contact(std::vector<std::shared_ptr<Rigidbody>>& rigidbodies, Vector3f contactPoint, Vector3f contactNormal, float penetration)
{
	this->rigidbodies = rigidbodies;
	this->contactPoint = contactPoint;
	this->contactNormal = contactNormal;
	this->penetration = penetration;
}

void Contact::PreCalculation(float duration)
{
    CalculateContactBasis();

    relativeContactPosition[0] = contactPoint - rigidbodies[0]->position;
    if (rigidbodies[1])
        relativeContactPosition[1] = contactPoint - rigidbodies[1]->position;

    contactVelocity = CalculateLocalVelocity(0, duration);

    if (rigidbodies[1])
        contactVelocity -= CalculateLocalVelocity(1, duration);

    CalculateDeltaVelocity(duration);
}

void Contact::CalculateContactBasis()
{
    Vector3f contactTangent[2];

    if (std::abs(contactNormal.x) > std::abs(contactNormal.y))
    {
        const float s = 1.0f / std::sqrt(contactNormal.z * contactNormal.z + contactNormal.x * contactNormal.x);

        contactTangent[0].x = contactNormal.z * s;
        contactTangent[0].y = 0;
        contactTangent[0].z = -contactNormal.x * s;

        contactTangent[1].x = contactNormal.y * contactTangent[0].x;
        contactTangent[1].y = contactNormal.z * contactTangent[0].x - contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
    }
    else
    {
        const float s = 1.0f / std::sqrt(contactNormal.z * contactNormal.z + contactNormal.y * contactNormal.y);

        contactTangent[0].x = 0;
        contactTangent[0].y = -contactNormal.z * s;
        contactTangent[0].z = contactNormal.y * s;

        contactTangent[1].x = contactNormal.y * contactTangent[0].z - contactNormal.z * contactTangent[0].y;
        contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = contactNormal.x * contactTangent[0].y;
    }

    std::array<float, 3 * 3> values = {contactNormal.x, contactNormal.y, contactNormal.z,
                                        contactTangent[0].x, contactTangent[0].y, contactTangent[0].z,
                                        contactTangent[1].x, contactTangent[1].y, contactTangent[1].z};

    contactToWorld = Matrix3f(values);
}

void Contact::CalculateDeltaVelocity(float duration)
{
    float velocityAcceleration = 0;

    if (rigidbodies[0]->isAwake)
    {
        velocityAcceleration += rigidbodies[0]->GetAcceleration() * duration * contactNormal;
    }

    if (rigidbodies[1] && rigidbodies[1]->isAwake)
    {
        velocityAcceleration -= rigidbodies[1]->GetAcceleration() * duration * contactNormal;
    }

    float thisRestitution = 0.f;//restitution
    if (std::abs(contactVelocity.x) < 0.25f)
    {
        thisRestitution = 0.0f;
    }

    deltaVelocity = -contactVelocity.x - thisRestitution * (contactVelocity.x - velocityAcceleration);
}

Vector3f Contact::CalculateLocalVelocity(int index, float duration)
{
    Vector3 velocity = Vector3f::CrossProduct(rigidbodies[index]->rotation.GetRotation(), relativeContactPosition[index]);
    velocity += rigidbodies[index]->velocity;

    Vector3 contactVelocity = contactToWorld.TransformTranspose(velocity);

    Vector3 accelerationVelocity = rigidbodies[index]->GetAcceleration() * duration;
    accelerationVelocity = contactToWorld.TransformTranspose(accelerationVelocity);
    accelerationVelocity.x = 0;

    contactVelocity += accelerationVelocity;

    return contactVelocity;
}
