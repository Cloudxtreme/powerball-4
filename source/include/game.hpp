#pragma once

#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;

namespace {
  const float STEP = 1.0/1000.0;

  const float WINKY_MASS = 1.0;
  const btVector3 GRAVITY(0,-20,0);
}

class Game : public IEventReceiver
{
public:
  Game() {
    this->device = createDevice(
      video::EDT_OPENGL,
      dimension2d<u32>(1024, 768),
      16,
      false,
      false,
      false,
      this);

    if (!device) return;

    device->setWindowCaption(L"Powerball");

    this->driver = device->getVideoDriver();
    this->smgr = device->getSceneManager();

    btBroadphaseInterface *broadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
    btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher *dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver();
    world = new btDiscreteDynamicsWorld(dispatcher, broadPhase, solver, collisionConfiguration);

    world->setGravity(GRAVITY);

    this->winkyShape = new btSphereShape(1);

    IAnimatedMesh *mesh = smgr->getMesh("/Users/dylanmckay/Desktop/winky.3ds");
    IAnimatedMesh *groundMesh = smgr->getMesh("/Users/dylanmckay/Desktop/map.3ds");

    if (!mesh || !groundMesh) {
      device->drop();
      return;
    }

    winkyNode = smgr->addAnimatedMeshSceneNode(mesh);
    winkyNode->setPosition(vector3df(0,5,0));
    winkyNode->setMaterialFlag(EMF_LIGHTING, false);

    groundNode = smgr->addAnimatedMeshSceneNode(groundMesh);
    groundNode->setMaterialTexture(0, driver->getTexture("/Users/dylanmckay/Desktop/brick.jpg"));

    camera = smgr->addCameraSceneNode();

    ILightSceneNode *light = smgr->addLightSceneNode(nullptr, vector3df(0,10,0));
    smgr->addSkyDomeSceneNode(driver->getTexture("/Users/dylanmckay/Desktop/ngc346.jpg"));

    camera->setPosition(vector3df(13,7,-13));
    camera->setTarget(vector3df(0,0,0));

    groundShape = new btStaticPlaneShape(btVector3(0,1,0), 1);
    btMotionState *groundState =
      new btDefaultMotionState(btTransform(
          btQuaternion(0,0,0,1),
          btVector3(0, -1, 0)));

    btRigidBody::btRigidBodyConstructionInfo groundCI(0, groundState, groundShape, btVector3(0,0,0));
    groundBody = new btRigidBody(groundCI);
    groundBody->setFriction(1.0);
    groundBody->setRollingFriction(1.0);


    btMotionState *winkyState = new MotionState(winkyNode);

    btVector3 winkyInertia(0,0,0);
    winkyShape->calculateLocalInertia(WINKY_MASS, winkyInertia);
    btRigidBody::btRigidBodyConstructionInfo winkyRigidBodyCI(WINKY_MASS,
                                                              winkyState,
                                                              winkyShape,
                                                              winkyInertia);
    winkyBody = new btRigidBody(winkyRigidBodyCI);
    winkyBody->setFriction(1.0);
    winkyBody->setRollingFriction(1.0);

    world->addRigidBody(winkyBody);
    world->addRigidBody(groundBody);
  }

  ~Game() {
    device->drop();

    delete winkyBody->getMotionState();
    delete groundBody->getMotionState();

    delete winkyBody;
    delete groundBody;

    delete winkyShape;
    delete groundShape;
  }

  void run() {
    while (device->run()) {
      world->stepSimulation(STEP);

      camera->setPosition(winkyNode->getPosition() - vector3df(0,-2,4));
      camera->setTarget(winkyNode->getPosition());

      driver->beginScene(true, true, SColor(255,100,101,140));

      smgr->drawAll();

      driver->endScene();
    }
  }

  bool OnEvent(const SEvent &event) {
    switch (event.EventType) {
    case EET_KEY_INPUT_EVENT:
      return keyEvent(event.KeyInput);
    default:
      return false;
    }
  }

  bool keyEvent(const SEvent::SKeyInput &event) {
    if (!event.PressedDown) return false;

    double accelerationSize = 50.0;

    btVector3 forwardVector = toBullet(camera->getTarget());
    btVector3 leftVector = btVector3(1,0,0);

    switch (event.Key) {
    case KEY_UP:
      accelerate(accelerationSize * forwardVector);
      return true;
    case KEY_DOWN:
      accelerate(accelerationSize * -forwardVector);
      return true;
    default:
      return false;
    }
  }

  void accelerate(btVector3 vec) {
    this->winkyBody->applyCentralImpulse(vec);
  }

private:
  IrrlichtDevice *device;
  IVideoDriver *driver;
  ISceneManager *smgr;
  ICameraSceneNode *camera;
  ISceneNode *winkyNode;
  ISceneNode *groundNode;

  btDiscreteDynamicsWorld *world;

  btCollisionShape *winkyShape;
  btCollisionShape *groundShape;
  btRigidBody *winkyBody;
  btRigidBody *groundBody;
};

