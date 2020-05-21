#ifndef _MESH_H_
#define _MESH_H_

#include "core.h"

class Mesh {
 public:
  virtual ~Mesh() = 0;

  virtual void Draw(const glm::mat4& viewProjMtx, GLuint shader) = 0;

  virtual void Update() = 0;

  virtual int GetNumOfParticles() = 0;
};

#endif
