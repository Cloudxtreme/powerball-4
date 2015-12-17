#pragma once

#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>

#include "util.hpp"

using namespace irr;
using namespace scene;
using namespace irr::core;

using namespace util;

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
};

