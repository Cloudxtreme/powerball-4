#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>
#include <iostream>

using namespace irr;
using namespace std;

using namespace core;
using namespace scene;
using namespace video;
using namespace gui;
using namespace io;

namespace {
  const float STEP = 1.0/1000.0;

  const float WINKY_MASS = 1.0;
  const btVector3 GRAVITY(0,-20,0);
}

class MotionState : public btMotionState
{
protected:
  ISceneNode *node;

public:
  MotionState(ISceneNode *node) :
    node(node) {
  }

  void getWorldTransform(btTransform &worldTrans) const
  {
      vector3df pos = node->getPosition();
      worldTrans.setIdentity();

      worldTrans.setOrigin(toBullet(node->getPosition()));
      worldTrans.setRotation(btQuaternion(pos.X, pos.Y, pos.Z));
  }

  void setWorldTransform(const btTransform &worldTrans)
  {
      if(this->node == nullptr) return;

      this->node->setPosition(toIrrlicht(worldTrans.getOrigin()));
      this->node->setRotation(toIrrlicht(worldTrans.getRotation()));
  }

private:
  vector3df toIrrlicht(btVector3 vec) const {
    return vector3df(vec.getX(), vec.getY(), vec.getZ());
  }

  vector3df toIrrlicht(btQuaternion quat) const {
    btVector3 euler;
    quaternionToEuler(quat, euler);
    return toIrrlicht(euler);
  }

  btVector3 toBullet(vector3df vec) const {
    return btVector3(vec.X, vec.Y, vec.Z);
  }

  // Converts a quaternion to an euler angle
  void quaternionToEuler(const btQuaternion &quat, btVector3 &euler) const {
    btScalar W = quat.getW();
    btScalar X = quat.getX();
    btScalar Y = quat.getY();
    btScalar Z = quat.getZ();
    float WSquared = W * W;
    float XSquared = X * X;
    float YSquared = Y * Y;
    float ZSquared = Z * Z;

    euler.setX(atan2f(2.0f * (Y * Z + X * W), -XSquared - YSquared + ZSquared + WSquared));
    euler.setY(asinf(-2.0f * (X * Z - Y * W)));
    euler.setZ(atan2f(2.0f * (X * Y + Z * W), XSquared - YSquared - ZSquared + WSquared));
    euler *= core::RADTODEG;
  }
};

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

    groundNode = smgr->addAnimatedMeshSceneNode(groundMesh);

    camera = smgr->addCameraSceneNode();

    ILightSceneNode *light = smgr->addLightSceneNode(nullptr, vector3df(0,10,0));

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

      //camera->setPosition(winkyNode->getPosition() - vector3df(0,2,1));
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

    btVector3 forwardVector = btVector3(0,0,1);
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
    this->winkyBody->applyTorqueImpulse(vec);
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

int main() {
  Game game;
  game.run();
  return 0;
}
