#include "Collision/ContactResolver.hpp"
#include "Collision/Contact.hpp"
#include "Rigidbody.hpp"

ContactResolver::ContactResolver(int iterations)
{
	this->iterations = iterations;
	this->iterationsUsed = 0;
}

void ContactResolver::ResolveContacts(std::vector<std::shared_ptr<Contact>>& contacts, float duration)
{
	if (contacts.size() == 0) return;

	ResolveVelocity(contacts, duration);
	ResolveInterpenetration(contacts, duration);

    contacts.clear();
}

void ContactResolver::ResolveVelocity(std::vector<std::shared_ptr<Contact>>& contacts, float duration)
{
	iterationsUsed = 0;

    Vector3f velocityChange[2], rotationChange[2];
    Vector3f deltaVelocity;

	while (iterationsUsed < iterations)
	{
		float max = 0.01f;

		int index = contacts.size();

		for (int i = 0; i < contacts.size(); i++)
		{
            if (contacts[i]->deltaVelocity > max)
            {
                max = contacts[i]->deltaVelocity;
                index = i;
            }
		}

        if (index == contacts.size()) break;

        Matrix3f inverseInertiaTensor[2];
        inverseInertiaTensor[0] = contacts[index]->rigidbodies[0]->GetInverseInertiaTensorWorld();

        if (contacts[index]->rigidbodies[1])
            inverseInertiaTensor[1] = contacts[index]->rigidbodies[1]->GetInverseInertiaTensorWorld();

        Vector3f impulseContact;
        float friction = 0.0f;

        if (friction == 0.0f)
        {
            impulseContact = CalculateImpulse(contacts[index], inverseInertiaTensor, false);
        }
        else
        {
            impulseContact = CalculateImpulse(contacts[index], inverseInertiaTensor, true);
        }

        Vector3f impulse = contacts[index]->contactToWorld.TransformTranspose(impulseContact);

        Vector3f impulsiveTorque = Vector3f::CrossProduct(contacts[index]->relativeContactPosition[0], impulse);
        rotationChange[0] = inverseInertiaTensor[0].TransformTranspose(impulsiveTorque);
        velocityChange[0] = Vector3f(0.f, 0.f, 0.f);
        velocityChange[0] += impulse * contacts[index]->rigidbodies[0]->inverseMass;

        contacts[index]->rigidbodies[0]->velocity += velocityChange[0];
        contacts[index]->rigidbodies[0]->transform.rotation.AddScaleVector(rotationChange[0], 1.f);

        if (contacts[index]->rigidbodies[1])
        {
            Vector3 impulsiveTorque = Vector3f::CrossProduct(impulse, contacts[index]->relativeContactPosition[1]);
            rotationChange[1] = inverseInertiaTensor[1].TransformTranspose(impulsiveTorque);
            velocityChange[1] = Vector3f(0.f, 0.f, 0.f);
            velocityChange[1] += impulse * -contacts[index]->rigidbodies[1]->inverseMass;

            contacts[index]->rigidbodies[1]->velocity += velocityChange[1];
            contacts[index]->rigidbodies[1]->transform.rotation.AddScaleVector(rotationChange[1], 1.f);
        }

        for (int i = 0; i < contacts.size(); i++)
        {
            for (int j = 0; j < 2; j++)
            {
                if (contacts[i]->rigidbodies[j])
                {
                    for (int x = 0; x < 2; x++)
                    {
                        if (contacts[i]->rigidbodies[j] == contacts[index]->rigidbodies[x])
                        {
                            deltaVelocity = velocityChange[x] + rotationChange[x].Cross(contacts[i]->relativeContactPosition[j]);

                            contacts[i]->contactVelocity += contacts[i]->contactToWorld.TransformTranspose(deltaVelocity) * (j ? -1 : 1);
                            contacts[i]->CalculateDeltaVelocity(duration);
                        }
                    }
                }
            }
        }

		iterationsUsed++;
	}
}

void ContactResolver::ResolveInterpenetration(std::vector<std::shared_ptr<Contact>>& contacts, float duration)
{
    int i, index;
    Vector3f linearChange[2], angularChange[2];
    float max;
    Vector3f deltaPosition;

    iterationsUsed = 0;

    while (iterationsUsed < iterations)
    {
        max = 0.01f;
        index = contacts.size();

        for (i = 0; i < contacts.size(); i++)
        {
            if (contacts[i]->penetration > max)
            {
                max = contacts[i]->penetration;
                index = i;
            }
        }

        if (index == contacts.size()) break;

        float angularLimit = 0.2f;
        float angularMove[2];
        float linearMove[2];

        float totalInertia = 0;
        float linearInertia[2];
        float angularInertia[2];

        for (int j = 0; j < 2; j++) 
        {
            if (contacts[index]->rigidbodies[j])
            {
                Matrix3f inverseInertiaTensor = contacts[index]->rigidbodies[j]->inverseInertiaTensorWorld;

                Vector3f angularInertiaWorld = Vector3f::CrossProduct(contacts[index]->relativeContactPosition[j], contacts[index]->contactNormal);
                angularInertiaWorld = inverseInertiaTensor.TransformTranspose(angularInertiaWorld);
                angularInertiaWorld = Vector3f::CrossProduct(angularInertiaWorld, contacts[index]->relativeContactPosition[j]);
                angularInertia[j] = angularInertiaWorld * contacts[index]->contactNormal;

                linearInertia[j] = contacts[index]->rigidbodies[j]->inverseMass;

                totalInertia += linearInertia[j] + angularInertia[j];
            }
        }

        for (int j = 0; j < 2; j++)
        {
            if (contacts[index]->rigidbodies[j])
            {
                float sign = (j == 0) ? 1 : -1;
                angularMove[j] = sign * contacts[index]->penetration * (angularInertia[j] / totalInertia);
                linearMove[j] = sign * contacts[index]->penetration * (linearInertia[j] / totalInertia);

                Vector3f projection = contacts[index]->relativeContactPosition[j];
                projection += contacts[index]->contactNormal * -Vector3f::DotProduct(contacts[index]->relativeContactPosition[j], contacts[index]->contactNormal);

                float maxLength = angularLimit * projection.GetLength();

                if (angularMove[j] < -maxLength)
                {
                    float totalMove = angularMove[j] + linearMove[j];
                    angularMove[j] = -maxLength;
                    linearMove[j] = totalMove - angularMove[j];
                }
                else if (angularMove[j] > maxLength)
                {
                    float totalMove = angularMove[j] + linearMove[j];
                    angularMove[j] = maxLength;
                    linearMove[j] = totalMove - angularMove[j];
                }

                if (angularMove[j] == 0)
                {
                    angularChange[j] = Vector3f(0, 0, 0);
                }
                else
                {
                    Vector3 targetAngularDirection = Vector3f::CrossProduct(contacts[index]->relativeContactPosition[j], contacts[index]->contactNormal);
                    Matrix3f inverseInertiaTensor = contacts[index]->rigidbodies[j]->inverseInertiaTensorWorld;

                    angularChange[j] = inverseInertiaTensor.TransformTranspose(targetAngularDirection) * (angularMove[j] / angularInertia[j]);
                }

                linearChange[j] = contacts[index]->contactNormal * linearMove[j];


                contacts[index]->rigidbodies[j]->transform.position = contacts[index]->contactNormal * linearMove[j];
                contacts[index]->rigidbodies[j]->transform.rotation.AddScaleVector(angularChange[j], 1.0f);


                if (!contacts[index]->rigidbodies[j]->isAwake) 
                    contacts[index]->rigidbodies[j]->CalculateDerivedData();
            }
        }

        for (i = 0; i < contacts.size(); i++)
        {
            for (int j = 0; j < 2; j++)
            {
                if (contacts[i]->rigidbodies[j])
                {
                    for (int x = 0; x < 2; x++)
                    {
                        if (contacts[i]->rigidbodies[j] == contacts[index]->rigidbodies[x])
                        {
                            deltaPosition = linearChange[x] + angularChange[x].Cross(contacts[i]->relativeContactPosition[j]);

                            contacts[i]->penetration += Vector3f::DotProduct(deltaPosition, contacts[i]->contactNormal) * (j ? 1 : -1);
                        }
                    }
                }
            }
        }

        iterationsUsed++;
    }
}

Vector3f ContactResolver::CalculateImpulse(std::shared_ptr<Contact>& contact, Matrix3f* inverseTensor, bool hasFriction)
{
    Vector3f impulseContact;
    Vector3f deltaVelocityWorld = Vector3f::CrossProduct(contact->relativeContactPosition[0], contact->contactNormal);
    deltaVelocityWorld = inverseTensor[0].TransformTranspose(deltaVelocityWorld);
    deltaVelocityWorld = Vector3f::CrossProduct(deltaVelocityWorld, contact->relativeContactPosition[0]);

    float deltaVelocity = deltaVelocityWorld * contact->contactNormal;
    deltaVelocity += contact->rigidbodies[0]->inverseMass;

    if (contact->rigidbodies[1])
    {
        Vector3f deltaVelocityWorld = Vector3f::CrossProduct(contact->relativeContactPosition[1], contact->contactNormal);
        deltaVelocityWorld = inverseTensor[1].TransformTranspose(deltaVelocityWorld);
        deltaVelocityWorld = Vector3f::CrossProduct(deltaVelocityWorld, contact->relativeContactPosition[1]);

        deltaVelocity += deltaVelocityWorld * contact->contactNormal;
        deltaVelocity += contact->rigidbodies[1]->inverseMass;
    }

    impulseContact.x = contact->deltaVelocity / deltaVelocity;
    impulseContact.x = 0;
    impulseContact.x = 0;

    return impulseContact;
}
