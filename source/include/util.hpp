#pragma once

#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>

using namespace irr;
using namespace core;

namespace util
{
  // Converts a quaternion to an euler angle
  void quaternionToEuler(const btQuaternion &quat, btVector3 &euler) {
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

  vector3df toIrrlicht(btVector3 vec) {
    return vector3df(vec.getX(), vec.getY(), vec.getZ());
  }

  vector3df toIrrlicht(btQuaternion quat) {
    btVector3 euler;
    quaternionToEuler(quat, euler);
    return toIrrlicht(euler);
  }

  btVector3 toBullet(vector3df vec) {
    return btVector3(vec.X, vec.Y, vec.Z);
  }
}
