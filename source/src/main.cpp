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

class MotionState : public btMotionState
{
protected:
  btTransform m_InitialPosition;
  ISceneNode *node;

public:
  MotionState(const btTransform &initialPosition,
              ISceneNode *node) : m_InitialPosition(initialPosition),
                                  node(node) {
  }

  void getWorldTransform(btTransform &worldTrans) const
  {
      worldTrans = m_InitialPosition;
  }

  void setWorldTransform(const btTransform &worldTrans)
  {
      if(this->node == nullptr) return;

      btQuaternion rot = worldTrans.getRotation();


      btVector3 pos = worldTrans.getOrigin();
      this->node->setPosition(toIrrlicht(pos));
  }
private:
  vector3df toIrrlicht(btVector3 vec) {
    return vector3df(vec.getX(), vec.getY(), vec.getZ());
  }

  vector3df toIrrlicht(btQuaternion quat) {
    quaternion q = quaternion(quat.getX(), quat.getY(), quat.getZ(), quat.getW());

    vector3df euler;
    q.toEuler(euler);
    return euler;
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

  while (device->run()) {
    driver->beginScene(true, true, SColor(255,100,101,140));

    smgr->drawAll();
    gui->drawAll();

    driver->endScene();
  }

  device->drop();

  return 0;
}
