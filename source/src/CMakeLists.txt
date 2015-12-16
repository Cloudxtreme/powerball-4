add_executable (powerball MACOSX_BUNDLE main.cpp)


target_include_directories (powerball PUBLIC
  /usr/local/include/irrlicht
  /usr/local/include/bullet
  )

if (APPLE)
  find_package(OpenGL REQUIRED)
  find_library(IOKIT_LIBRARY IOKit)
  find_library(Cocoa REQUIRED)
  find_package(Bullet REQUIRED)

  set(LIBS
    "-framework Cocoa"
    irrlicht
    ${BULLET_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${IOKIT_LIBRARY}
    )
endif()

target_link_libraries (powerball LINK_PUBLIC ${LIBS})
