#include <bullet/btBulletDynamicsCommon.h>
#include <iostream>

int main(int argc, char **argv) {
    // 1. Set up the Bullet world with basic configurations
    btDefaultCollisionConfiguration *collisionConfiguration =
        new btDefaultCollisionConfiguration();
    btCollisionDispatcher *dispatcher =
        new btCollisionDispatcher(collisionConfiguration);
    btBroadphaseInterface *broadphase = new btDbvtBroadphase();
    btSequentialImpulseConstraintSolver *solver =
        new btSequentialImpulseConstraintSolver();

    // Dynamics world - this is where the physics takes place
    btDiscreteDynamicsWorld *dynamicsWorld = new btDiscreteDynamicsWorld(
        dispatcher, broadphase, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, -9.81, 0)); // Gravity in the world

    // 2. Create ground (a static plane)
    btCollisionShape *groundShape =
        new btStaticPlaneShape(btVector3(0, 1, 0), 1); // Flat ground on Y-axis
    btDefaultMotionState *groundMotionState = new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(
        0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody *groundRigidBody = new btRigidBody(groundRigidBodyCI);
    dynamicsWorld->addRigidBody(groundRigidBody);

    // 3. Create a dynamic box (cube)
    btCollisionShape *fallShape = new btBoxShape(btVector3(1, 1, 1)); // 1x1x1 cube
    btDefaultMotionState *fallMotionState = new btDefaultMotionState(btTransform(
        btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0))); // Start 50 units high
    btScalar mass = 1;                                   // Cube mass
    btVector3 fallInertia(0, 0, 0);
    fallShape->calculateLocalInertia(mass, fallInertia); // Calculate inertia

    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(
        mass, fallMotionState, fallShape, fallInertia);
    btRigidBody *fallRigidBody = new btRigidBody(fallRigidBodyCI);
    dynamicsWorld->addRigidBody(fallRigidBody);

    // 4. Simulate the world for a few steps
    for (int i = 0; i < 300; i++) {
        dynamicsWorld->stepSimulation(
            1 / 60.f, 10); // Step the simulation (1/60 second per step)

        btTransform trans;
        fallRigidBody->getMotionState()->getWorldTransform(
            trans); // Get cube position
        std::cout << "Cube position at step " << i << ": "
                  << trans.getOrigin().getX() << ", " << trans.getOrigin().getY()
                  << ", " << trans.getOrigin().getZ() << std::endl;
    }

    // 5. Clean up memory
    dynamicsWorld->removeRigidBody(fallRigidBody);
    delete fallRigidBody->getMotionState();
    delete fallRigidBody;
    delete fallShape;

    dynamicsWorld->removeRigidBody(groundRigidBody);
    delete groundRigidBody->getMotionState();
    delete groundRigidBody;
    delete groundShape;

    delete dynamicsWorld;
    delete solver;
    delete broadphase;
    delete dispatcher;
    delete collisionConfiguration;

    return 0;
}