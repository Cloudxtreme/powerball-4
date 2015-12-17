#pragma once

#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>

#include "Winky.hpp"

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
  Game() : debugMode(false) {
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

    IAnimatedMesh *groundMesh = smgr->getMesh("/Users/dylanmckay/assets/map.obj");

    if (!groundMesh) {
      device->drop();
      return;
    }

    groundNode = smgr->addAnimatedMeshSceneNode(groundMesh);
    groundNode->setScale(vector3df(10,10,10));
    groundNode->setPosition(vector3df(0,-10,0));
    groundNode->setMaterialFlag(EMF_LIGHTING, false);

    setDebug(false);

    ILightSceneNode *light = smgr->addLightSceneNode(nullptr, vector3df(0,-100,0));
    smgr->addSkyDomeSceneNode(driver->getTexture("/Users/dylanmckay/assets/ngc346.jpg"));

    groundShape = new btStaticPlaneShape(btVector3(0,1,0), 1);
    btMotionState *groundState =
      new btDefaultMotionState(btTransform(
          btQuaternion(0,0,0,1),
          btVector3(0, -1, 0)));

    btRigidBody::btRigidBodyConstructionInfo groundCI(0, groundState, groundShape, btVector3(0,0,0));
    groundBody = new btRigidBody(groundCI);
    groundBody->setFriction(1.0);
    groundBody->setRollingFriction(1.0);

    world->addRigidBody(groundBody);

    this->winky = new Winky(smgr, world);
  }

  ~Game() {
    device->drop();

    delete winky;
    delete groundBody->getMotionState();
    delete groundBody;
    delete groundShape;
  }

  void run() {
    while (device->run()) {
      world->stepSimulation(STEP);

      if (!debugMode) {
        winky->positionCamera(camera);
      }

      driver->beginScene(true, true, SColor(255,100,101,140));

      smgr->drawAll();

      driver->endScene();
    }
  }

  void setDebug(bool debug) {
    debugMode = debug;

    if (debugMode) {
      camera = smgr->addCameraSceneNodeFPS(nullptr, 100.0f, 0.1);
    } else {
      camera = smgr->addCameraSceneNode();
    }

    camera->setPosition(vector3df(13,7,-13));
    camera->setTarget(vector3df(0,0,0));
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
      if (!debugMode) {
        winky->moveForward();
        return true;
      } else {
        return false;
      }
    case KEY_DOWN:
      if (!debugMode) {
        winky->moveBackward();
        return true;
      } else {
        return false;
      }
    case KEY_KEY_D:
      setDebug(!debugMode);
      return true;
    default:
      return false;
    }
  }

private:
  IrrlichtDevice *device;
  IVideoDriver *driver;
  ISceneManager *smgr;
  ICameraSceneNode *camera;

  ISceneNode *groundNode;

  btDiscreteDynamicsWorld *world;

  btCollisionShape *groundShape;
  btRigidBody *groundBody;

  Winky *winky;

  bool debugMode;
};

