#pragma once

#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>

using namespace irr;
using namespace core;
using namespace video;

namespace {
  double MASS = 1.0;
}

class Winky
{
public:
  Winky(ISceneManager *smgr, btDynamicsWorld *world) : direction(0,0,1)
  {
    shape = new btSphereShape(1);

    IAnimatedMesh *mesh = smgr->getMesh("/Users/dylanmckay/assets/winky.3ds");
    node = smgr->addAnimatedMeshSceneNode(mesh);
    node->setPosition(vector3df(0,5,0));
    node->setMaterialFlag(EMF_LIGHTING, false);

    btMotionState *state = new MotionState(node);

    btVector3 inertia(0,0,0);
    shape->calculateLocalInertia(MASS, inertia);
    btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(MASS,
                                                         state,
                                                         shape,
                                                         inertia);
    body = new btRigidBody(rigidBodyCI);
    body->setFriction(1.0);
    body->setRollingFriction(1.0);

    world->addRigidBody(body);
  }

  ~Winky() {
    delete body->getCollisionShape();
    delete body->getMotionState();
    delete body;
  }

  void moveForward() {
    body->applyCentralImpulse(forwardsVector());
  }

  void moveBackward() {
    body->applyCentralImpulse(-forwardsVector());
  }

  void positionCamera(ICameraSceneNode *camera) {
    camera->setPosition(node->getPosition() - vector3df(0,-2,4));
    camera->setTarget(node->getPosition());
  }

  btVector3 getPosition() const { return body->getCenterOfMassTransform().getOrigin(); }
  btVector3 getDirection() const { return direction; }

  btVector3 forwardsVector() const {
    return btVector3(1,0,0);
  }

private:
  btVector3 direction;

  btCollisionShape *shape;
  btRigidBody *body;
  ISceneNode *node;
};
