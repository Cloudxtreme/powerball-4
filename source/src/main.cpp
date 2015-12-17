#include <irrlicht.h>
#include <btBulletDynamicsCommon.h>
#include <iostream>

#include "../include/util.hpp"
#include "../include/MotionState.hpp"
#include "../include/game.hpp"

using namespace irr;
using namespace std;

using namespace core;
using namespace scene;
using namespace video;
using namespace gui;
using namespace io;

int main() {
  Game game;
  game.run();
  return 0;
}
