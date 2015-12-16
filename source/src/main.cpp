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

  world->setGravity(btVector3(0, -9.81, 0));

  btCollisionShape *winkyShape = new btSphereShape(1);

  btDefaultMotionState *groundState =
    new btDefaultMotionState(btTransform(
        btQuaternion(0,0,0,1),
        btVector3(0, -1, 0)));

  if (!device) return 1;

  device->setWindowCaption(L"Powerball");
  IVideoDriver *driver = device->getVideoDriver();
  ISceneManager *smgr = device->getSceneManager();
  IGUIEnvironment *gui = device->getGUIEnvironment();

  IAnimatedMesh *mesh = smgr->getMesh("/Users/dylanmckay/winky.obj");

  if (!mesh) {
    device->drop();
    return 1;
  }

  IAnimatedMeshSceneNode *winkyNode = smgr->addAnimatedMeshSceneNode(mesh);

  MotionState *winkyState = new MotionState(winkyNode);

  btVector3 winkyInertia(0,0,0);
  winkyShape->calculateLocalInertia(WINKY_MASS, winkyInertia);
  btRigidBody::btRigidBodyConstructionInfo winkyRigidBodyCI(WINKY_MASS,
                                                            winkyState,
                                                            winkyShape,
                                                            winkyInertia);
  btRigidBody *winkyBody = new btRigidBody(winkyRigidBodyCI);

  world->addRigidBody(winkyBody);

  while (device->run()) {
    world->stepSimulation(STEP, 10);

    driver->beginScene(true, true, SColor(255,100,101,140));

    smgr->drawAll();
    gui->drawAll();

    driver->endScene();
  }

  device->drop();

  return 0;
}
