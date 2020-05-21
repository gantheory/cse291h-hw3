#include "rigid_body.h"
#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <glm/gtx/matrix_operation.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <random>
#include "core.h"

#define PI 3.1415926f

RigidBody::RigidBody(float a, float b, float c, glm::vec3 initAngularVelocity) {
  mass = kDensity * a * b * c;
  principalInertia.x = mass / 12.f * (b * b + c * c);
  principalInertia.y = mass / 12.f * (a * a + c * c);
  principalInertia.z = mass / 12.f * (a * a + b * b);
  inverseTheta0 = glm::diagonal3x3(1.f / principalInertia);

  position = glm::vec3(0.f);
  momentum = glm::vec3(0.f, mass * 100.f, 0.f);

  orientation = glm::mat3(1.f);
  angularMomentum = glm::diagonal3x3(principalInertia) * initAngularVelocity;

  rigidBodyMin = -glm::vec3(a * 0.5f, b * 0.5f, c * 0.5f);
  rigidBodyMax = glm::vec3(a * 0.5f, b * 0.5f, c * 0.5f);

  minMaxPoints = {rigidBodyMin, rigidBodyMax};

  originalPoints = {
      glm::vec3(minMaxPoints[0].x, minMaxPoints[0].y, minMaxPoints[0].z),
      glm::vec3(minMaxPoints[0].x, minMaxPoints[1].y, minMaxPoints[0].z),
      glm::vec3(minMaxPoints[1].x, minMaxPoints[1].y, minMaxPoints[0].z),
      glm::vec3(minMaxPoints[1].x, minMaxPoints[0].y, minMaxPoints[0].z),
      glm::vec3(minMaxPoints[0].x, minMaxPoints[0].y, minMaxPoints[1].z),
      glm::vec3(minMaxPoints[0].x, minMaxPoints[1].y, minMaxPoints[1].z),
      glm::vec3(minMaxPoints[1].x, minMaxPoints[1].y, minMaxPoints[1].z),
      glm::vec3(minMaxPoints[1].x, minMaxPoints[0].y, minMaxPoints[1].z)};

  // Generate a vertex array (VAO) and two vertex buffer objects (VBO).
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO_positions);
  glGenBuffers(1, &VBO_normals);
}

RigidBody::~RigidBody() {
  // Delete the VBOs and the VAO.
  glDeleteBuffers(1, &VBO_positions);
  glDeleteBuffers(1, &VBO_normals);
  glDeleteBuffers(1, &EBO);
  glDeleteVertexArrays(1, &VAO);
}

void RigidBody::Draw(const glm::mat4& viewProjMtx, GLuint shader) {
  auto points = originalPoints;
  for (auto& point : points) point = orientation * point + position;

  std::vector<glm::vec3> positions = {
      points[0], points[1], points[2],  // for format
      points[0], points[2], points[3],  // for format
      points[0], points[5], points[1],  // for format
      points[0], points[4], points[5],  // for format
      points[0], points[3], points[7],  // for format
      points[0], points[7], points[4],  // for format
      points[6], points[5], points[4],  // for format
      points[6], points[4], points[7],  // for format
      points[6], points[2], points[1],  // for format
      points[6], points[1], points[5],  // for format
      points[6], points[7], points[3],  // for format
      points[6], points[3], points[2]};

  std::vector<glm::vec3> normals(positions.size());
  for (size_t i = 0; i < positions.size() / 3; ++i) {
    glm::vec3 v = glm::cross(positions[3 * i + 1] - positions[3 * i],
                             positions[3 * i + 2] - positions[3 * i]);
    for (int j = 0; j < 3; ++j) normals[3 * i + j] = v;
  }

  std::vector<unsigned int> indices(positions.size());
  iota(indices.begin(), indices.end(), 0);

  // Bind to the VAO.
  glBindVertexArray(VAO);

  // Bind to the first VBO - We will use it to store the vertices
  glBindBuffer(GL_ARRAY_BUFFER, VBO_positions);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * positions.size(),
               positions.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);

  // Bind to the second VBO - We will use it to store the normals
  glBindBuffer(GL_ARRAY_BUFFER, VBO_normals);
  glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * normals.size(),
               normals.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);

  // Generate EBO, bind the EBO to the bound VAO and send the data
  glGenBuffers(1, &EBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(),
               indices.data(), GL_STATIC_DRAW);

  // Unbind the VBOs.
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  // actiavte the shader program
  glUseProgram(shader);

  // get the locations and send the uniforms to the shader
  glUniformMatrix4fv(glGetUniformLocation(shader, "viewProj"), 1, false,
                     (float*)&viewProjMtx);
  glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE,
                     (float*)&kModel);
  glUniform3fv(glGetUniformLocation(shader, "DiffuseColor"), 1, &kColor[0]);

  // Bind to the VAO.
  glBindVertexArray(VAO);

  // draw the points using triangles, indexed with the EBO
  glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

  // Unbind the VAO and shader program
  glBindVertexArray(0);
  glUseProgram(0);
}

void RigidBody::Update() {
  force = torque = glm::vec3(0.f);

  if (position.y > kGround) {
    glm::vec3 gravity(0.f);
    gravity.y = -mass * kG;
    ApplyForce(gravity, position);
  }

  float minY = 1e9;
  int idx = -1;
  // Ground collision.
  for (size_t i = 0; i < originalPoints.size(); ++i) {
    auto& originalPoint = originalPoints[i];
    auto point = orientation * originalPoint + position;
    if (point.y <= kGround) {
      if (point.y < minY) {
        idx = i;
        minY = fmin(minY, point.y);
      }
    }
  }

  if (minY <= kGround) {
    glm::vec3 point = orientation * originalPoints[idx] + position;
    glm::vec3 impulse = GetImpulse(point);

    glm::vec3 f = impulse / kTimestep;
    ApplyForce(f, point);
  }

  Integrate(kTimestep);
}

void RigidBody::ApplyForce(const glm::vec3& f, const glm::vec3& pos) {
  force += f;
  torque += glm::cross(pos - position, f);
}

void RigidBody::Integrate(float timestep) {
  // Linear motion.
  momentum += force * timestep;
  glm::vec3 v = momentum / mass;
  position += v * timestep;

  // Angular motion.
  if (kBuss) {
    glm::vec3 w = GetAngularVelocity();
    glm::vec3 wDot =
        GetInverseThetaMatrix() * (torque - glm::cross(w, angularMomentum));
    float h = timestep;
    glm::vec3 wDotDot =
        w + h * 0.5f * wDot + h * h / 12.f * glm::cross(wDot, w);
    orientation = GetRotationMatrix(wDotDot * timestep) * orientation;
    angularMomentum += torque * timestep;
  } else {
    angularMomentum += torque * timestep;
    glm::vec3 w = GetAngularVelocity();
    orientation = GetRotationMatrix(w * timestep) * orientation;
  }
}

glm::mat3 RigidBody::GetRotationMatrix(glm::vec3 theta) {
  glm::mat3 rotateX(glm::vec3(1.f, 0.f, 0.f),
                    glm::vec3(0.f, cos(theta.x), sin(theta.x)),
                    glm::vec3(0.f, -sin(theta.x), cos(theta.x)));
  glm::mat3 rotateY(glm::vec3(cos(theta.y), 0.f, -sin(theta.y)),
                    glm::vec3(0.f, 1.f, 0.f),
                    glm::vec3(sin(theta.y), 0.f, cos(theta.y)));
  glm::mat3 rotateZ(glm::vec3(cos(theta.z), sin(theta.z), 0.f),
                    glm::vec3(-sin(theta.z), cos(theta.z), 0.f),
                    glm::vec3(0.f, 0.f, 1.f));
  return rotateZ * rotateY * rotateX;
}

glm::vec3 RigidBody::GetAngularVelocity() {
  return GetInverseThetaMatrix() * angularMomentum;
}

glm::mat3 RigidBody::GetHatOperator(glm::vec3 v) {
  return glm::mat3(glm::vec3(0, v.z, -v.y), glm::vec3(-v.z, 0, v.x),
                   glm::vec3(v.y, -v.x, 0));
}

glm::mat3 RigidBody::GetInverseThetaMatrix() {
  return orientation * inverseTheta0 * glm::transpose(orientation);
}

glm::mat3 RigidBody::GetInverseMassMatrix(glm::vec3 r) {
  return glm::diagonal3x3(glm::vec3(1.f / mass)) -
         GetHatOperator(r) * GetInverseThetaMatrix() * GetHatOperator(r);
}

bool RigidBody::IsInsideFrictionCone(glm::vec3 impulse) {
  float tangent = std::sqrt(pow(impulse.x, 2) + pow(impulse.z, 2));
  float normal = std::fabs(impulse.y);
  return tangent <= kMiuS * normal;
}

float RigidBody::GetL2Norm(glm::vec3& v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

glm::vec3 RigidBody::GetImpulse(glm::vec3 point) {
  glm::vec3 r = point - position;
  glm::vec3 n(0.f, 1.f, 0.f);
  glm::vec3 vr = momentum / mass + glm::cross(GetAngularVelocity(), r);
  glm::vec3 vNorm = vr * n;
  glm::vec3 vTan = vr - vNorm;
  glm::mat3 inverseM = GetInverseMassMatrix(r);
  glm::vec3 vNormPrime = -kEpsilon * vNorm;
  glm::mat3 M = glm::inverse(inverseM);
  glm::vec3 impulse = M * (vNormPrime - vr);

  if (!IsInsideFrictionCone(impulse)) {
    glm::vec3 T = vTan / GetL2Norm(vTan);
    glm::vec3 den = inverseM * (n - kMiuD * T);
    impulse = -(kEpsilon + 1) * vNorm / den.y;
  }
  return impulse;
}
