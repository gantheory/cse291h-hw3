#include "camera.h"

#include "glm/gtx/euler_angles.hpp"

Camera::Camera() { Reset(); }

void Camera::Update() {
  // Compute camera world matrix
  glm::mat4 world(1);
  world[3][1] = -4.5;
  world[3][2] = Distance;
  world = glm::eulerAngleY(glm::radians(-Azimuth)) *
          glm::eulerAngleX(glm::radians(-Incline)) * world;

  // Compute view matrix (inverse of world matrix)
  glm::mat4 view = glm::inverse(world);

  // Compute perspective projection matrix
  glm::mat4 project =
      glm::perspective(glm::radians(FOV), Aspect, NearClip, FarClip);

  // Compute final view-projection matrix
  ViewProjectMtx = project * view;
}

void Camera::Reset() {
  FOV = 45.0f;
  Aspect = 1.00f;
  NearClip = 0.1f;
  FarClip = 100.0f;

  Distance = 25.0f;
  Azimuth = -45.0f;
  Incline = 25.0f;
}
