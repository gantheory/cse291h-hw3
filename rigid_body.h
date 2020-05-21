#ifndef _RIGID_BODY_H_
#define _RIGID_BODY_H_

#include "core.h"

class RigidBody {
 private:
  const float kGround = -10.f;
  const float kDensity = 1.f;       // g / cm^3
  const float kG = 980.f;           // cm / s^2
  const float kTimestep = 0.00005;  // s
  const float kMiuS = 1.5f;
  const float kMiuD = 1.0f;
  const float kEpsilon = 0.80f;  // 0.25 [0, 1], 0.1
  const float kBuss = false;

  const glm::mat4 kModel = glm::mat4(1.0f);
  const glm::vec3 kColor = glm::vec3(1.0f);

  // Rendering.
  GLuint VAO;
  GLuint VBO_positions, VBO_normals, EBO;
  glm::vec3 rigidBodyMin, rigidBodyMax;
  std::vector<glm::vec3> minMaxPoints, originalPoints;

  // Basic properties.
  float mass;
  glm::vec3 principalInertia;
  glm::mat3 inverseTheta0;

  // Variables.
  glm::vec3 position;
  glm::vec3 momentum;
  glm::mat3 orientation;
  glm::vec3 angularMomentum;

  // Temps.
  glm::vec3 force;
  glm::vec3 torque;

  void ApplyForce(const glm::vec3& f, const glm::vec3& pos);

  void Integrate(float timestep);

  glm::mat3 GetRotationMatrix(glm::vec3 theta);

  glm::vec3 GetAngularVelocity();

  glm::mat3 GetHatOperator(glm::vec3 v);

  glm::mat3 GetInverseThetaMatrix();

  glm::mat3 GetInverseMassMatrix(glm::vec3 r);

  bool IsInsideFrictionCone(glm::vec3 impulse);

  float GetL2Norm(glm::vec3& v);

  glm::vec3 GetImpulse(glm::vec3 point);

 public:
  RigidBody(float a, float b, float c, glm::vec3 initAngularVelocity);

  ~RigidBody();

  void Draw(const glm::mat4& viewProjMtx, GLuint shader);

  void Update();
};

#endif
