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
  const btVector3 GRAVITY(0,-3,0);
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

      btQuaternion rot = worldTrans.getRotation();


      btVector3 pos = worldTrans.getOrigin();
      this->node->setPosition(toIrrlicht(pos));
  }
private:
  vector3df toIrrlicht(btVector3 vec) const {
    return vector3df(vec.getX(), vec.getY(), vec.getZ());
  }

  vector3df toIrrlicht(btQuaternion quat) const {
    quaternion q = quaternion(quat.getX(), quat.getY(), quat.getZ(), quat.getW());

    vector3df euler;
    q.toEuler(euler);
    return euler;
  }

   btVector3 toBullet(vector3df vec) const {
     return btVector3(vec.X, vec.Y, vec.Z);
   }
};


int main() {
  IrrlichtDevice *device = createDevice(
    video::EDT_OPENGL,
    dimension2d<u32>(1024, 768),
    16,
    false,
    false,
    false,
    0);

  btDiscreteDynamicsWorld *world;
  btBroadphaseInterface *broadPhase = new btAxisSweep3(btVector3(-1000, -1000, -1000), btVector3(1000, 1000, 1000));
  btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher *dispatcher = new btCollisionDispatcher(collisionConfiguration);
  btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver();
  world = new btDiscreteDynamicsWorld(dispatcher, broadPhase, solver, collisionConfiguration);

  world->setGravity(GRAVITY);

  btCollisionShape *winkyShape = new btSphereShape(1);

  if (!device) return 1;

  device->setWindowCaption(L"Powerball");
  IVideoDriver *driver = device->getVideoDriver();
  ISceneManager *smgr = device->getSceneManager();
  IGUIEnvironment *gui = device->getGUIEnvironment();

  IAnimatedMesh *mesh = smgr->getMesh("/Users/dylanmckay/winky.obj");
  IAnimatedMesh *groundMesh = smgr->getMesh("/Users/dylanmckay/Desktop/map.3ds");

  if (!mesh || !groundMesh) {
    device->drop();
    return 1;
  }

  IAnimatedMeshSceneNode *winkyNode = smgr->addAnimatedMeshSceneNode(mesh);

  winkyNode->setPosition(vector3df(0,5,0));

  IAnimatedMeshSceneNode *groundNode = smgr->addAnimatedMeshSceneNode(groundMesh);

  SKeyMap keyMap[8];
  keyMap[0].Action = EKA_MOVE_FORWARD;
  keyMap[0].KeyCode = KEY_UP;
  keyMap[1].Action = EKA_MOVE_FORWARD;
  keyMap[1].KeyCode = KEY_KEY_W;

  keyMap[2].Action = EKA_MOVE_BACKWARD;
  keyMap[2].KeyCode = KEY_DOWN;
  keyMap[3].Action = EKA_MOVE_BACKWARD;
  keyMap[3].KeyCode = KEY_KEY_S;

  keyMap[4].Action = EKA_STRAFE_LEFT;
  keyMap[4].KeyCode = KEY_LEFT;
  keyMap[5].Action = EKA_STRAFE_LEFT;
  keyMap[5].KeyCode = KEY_KEY_A;

  keyMap[6].Action = EKA_STRAFE_RIGHT;
  keyMap[6].KeyCode = KEY_RIGHT;
  keyMap[7].Action = EKA_STRAFE_RIGHT;
  keyMap[7].KeyCode = KEY_KEY_D;

  //ICameraSceneNode *camera = smgr->addCameraSceneNodeFPS(nullptr, 100.0, 0.1, -1, keyMap, 8);
  ICameraSceneNode *camera = smgr->addCameraSceneNode();

  ILightSceneNode *light = smgr->addLightSceneNode(nullptr, vector3df(0,10,0));

  camera->setPosition(vector3df(13,7,-13));
  camera->setTarget(vector3df(0,0,0));

  btCollisionShape *groundShape = new btStaticPlaneShape(btVector3(0,1,0), 1);
  btDefaultMotionState *groundState =
    new btDefaultMotionState(btTransform(
        btQuaternion(0,0,0,1),
        btVector3(0, -1, 0)));

  btRigidBody::btRigidBodyConstructionInfo groundCI(0, groundState, groundShape, btVector3(0,0,0));
  btRigidBody *groundBody = new btRigidBody(groundCI);


  MotionState *winkyState = new MotionState(winkyNode);

  btVector3 winkyInertia(0,0,0);
  winkyShape->calculateLocalInertia(WINKY_MASS, winkyInertia);
  btRigidBody::btRigidBodyConstructionInfo winkyRigidBodyCI(WINKY_MASS,
                                                            winkyState,
                                                            winkyShape,
                                                            winkyInertia);
  btRigidBody *winkyBody = new btRigidBody(winkyRigidBodyCI);

  world->addRigidBody(winkyBody);
  world->addRigidBody(groundBody);

  while (device->run()) {
    world->stepSimulation(STEP, 10);

    //camera->setPosition(winkyNode->getPosition() - vector3df(0,2,1));
    camera->setTarget(winkyNode->getPosition());
    driver->beginScene(true, true, SColor(255,100,101,140));

    smgr->drawAll();
    gui->drawAll();

    driver->endScene();
  }

  device->drop();

  delete winkyBody;
  delete groundBody;
  delete winkyState;
  delete groundState;
  delete winkyShape;
  delete groundShape;

  return 0;
}
