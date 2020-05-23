#include "window.h"

// Window Properties
int Window::width;
int Window::height;
const char* Window::windowTitle = "CSE 291H HW1";

// Objects to render
RigidBody* Window::rigidBody;

// Camera Properties
Camera* Cam;

// Interaction Variables
bool LeftDown, RightDown;
int MouseX, MouseY;

// The shader program id
GLuint Window::shaderProgram;

// Constructors and desctructors
bool Window::initializeProgram() {
  // Create a shader program with a vertex shader and a fragment shader.
  shaderProgram = LoadShaders("shaders/shader.vert", "shaders/shader.frag");

  // Check the shader program.
  if (!shaderProgram) {
    std::cerr << "Failed to initialize shader program" << std::endl;
    return false;
  }

  return true;
}

bool Window::initializeObjects(float a, float b, float c,
                               glm::vec3 angularVelocity) {
  if (rigidBody) delete rigidBody;
  // Create a rigidBody
  rigidBody = new RigidBody(a, b, c, angularVelocity);
  return true;
}

void Window::cleanUp() {
  // Deallcoate the objects.
  delete rigidBody;

  // Delete the shader program.
  glDeleteProgram(shaderProgram);
}

// for the Window
GLFWwindow* Window::createWindow(int width, int height) {
  // Initialize GLFW.
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW" << std::endl;
    return NULL;
  }

  // 4x antialiasing.
  glfwWindowHint(GLFW_SAMPLES, 4);

#ifdef __APPLE__
  // Apple implements its own version of OpenGL and requires special treatments
  // to make it uses modern OpenGL.

  // Ensure that minimum OpenGL version is 3.3
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  // Enable forward compatibility and allow a modern OpenGL context
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  // Create the GLFW window.
  GLFWwindow* window = glfwCreateWindow(width, height, windowTitle, NULL, NULL);

  // Check if the window could not be created.
  if (!window) {
    std::cerr << "Failed to open GLFW window." << std::endl;
    glfwTerminate();
    return NULL;
  }

  // Make the context of the window.
  glfwMakeContextCurrent(window);

#ifndef __APPLE__
  // On Windows and Linux, we need GLEW to provide modern OpenGL functionality.

  // Initialize GLEW.
  if (glewInit()) {
    std::cerr << "Failed to initialize GLEW" << std::endl;
    return NULL;
  }
#endif

  // Set swap interval to 1.
  glfwSwapInterval(0);

  // set up the camera
  Cam = new Camera();
  Cam->SetAspect(float(width) / float(height));

  // initialize the interaction variables
  LeftDown = RightDown = false;
  MouseX = MouseY = 0;

  // Call the resize callback to make sure things get drawn immediately.
  Window::resizeCallback(window, width, height);

  return window;
}

void Window::resizeCallback(GLFWwindow* window, int width, int height) {
#ifdef __APPLE__
  // In case your Mac has a retina display.
  glfwGetFramebufferSize(window, &width, &height);
#endif
  Window::width = width;
  Window::height = height;
  // Set the viewport size.
  glViewport(0, 0, width, height);

  Cam->SetAspect(float(width) / float(height));
}

// update and draw functions
void Window::idleCallback() {
  // Perform any updates as necessary.
  Cam->Update();

  rigidBody->Update();
}

void Window::displayCallback(GLFWwindow* window) {
  // Clear the color and depth buffers.
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Render the object.
  rigidBody->Draw(Cam->GetViewProjectMtx(), Window::shaderProgram);

  // Gets events, including input such as keyboard and mouse or window resizing.
  glfwPollEvents();
  // Swap buffers.
  glfwSwapBuffers(window);
}

// helper to reset the camera
void Window::resetCamera() {
  Cam->Reset();
  Cam->SetAspect(float(Window::width) / float(Window::height));
}

// callbacks - for Interaction
void Window::keyCallback(GLFWwindow* window, int key, int scancode, int action,
                         int mods) {
  /*
   * TODO: Modify below to add your key callbacks.
   */
  const float sideA = 1;
  const float sideB = 5;
  const float sideC = 3;
  const float w = 30;

  // Check for a key press.
  if (action == GLFW_PRESS) {
    switch (key) {
      case GLFW_KEY_ESCAPE:
        // Close the window. This causes the program to also terminate.
        glfwSetWindowShouldClose(window, GL_TRUE);
        break;

        // case GLFW_KEY_R:
        //   resetCamera();
        //   break;

      case GLFW_KEY_Q:
        initializeObjects(sideA, sideA, sideA, glm::vec3(w, 0.f, 0.f));
        break;

      case GLFW_KEY_W:
        initializeObjects(sideA, sideA, sideA, glm::vec3(0.f, w, 0.f));
        break;

      case GLFW_KEY_E:
        initializeObjects(sideA, sideA, sideA, glm::vec3(0.f, 0.f, w));
        break;

      case GLFW_KEY_R:
        initializeObjects(sideA, sideA, sideA, glm::vec3(w, w, 0.f));
        break;

      case GLFW_KEY_T:
        initializeObjects(sideA, sideA, sideA, glm::vec3(w, 0.f, w));
        break;

      case GLFW_KEY_Y:
        initializeObjects(sideA, sideA, sideA, glm::vec3(0.f, w, w));
        break;

      case GLFW_KEY_U:
        initializeObjects(sideA, sideA, sideA, glm::vec3(w, w, w));
        break;

      case GLFW_KEY_A:
        initializeObjects(sideB, sideA, sideA, glm::vec3(w, 0.f, 0.f));
        break;

      case GLFW_KEY_S:
        initializeObjects(sideB, sideA, sideA, glm::vec3(0.f, w, 0.f));
        break;

      case GLFW_KEY_D:
        initializeObjects(sideB, sideA, sideA, glm::vec3(0.f, 0.f, w));
        break;

      case GLFW_KEY_F:
        initializeObjects(sideB, sideA, sideA, glm::vec3(w, w, 0.f));
        break;

      case GLFW_KEY_G:
        initializeObjects(sideB, sideA, sideA, glm::vec3(w, 0.f, w));
        break;

      case GLFW_KEY_H:
        initializeObjects(sideB, sideA, sideA, glm::vec3(0.f, w, w));
        break;

      case GLFW_KEY_J:
        initializeObjects(sideB, sideA, sideA, glm::vec3(w, w, w));
        break;

      case GLFW_KEY_Z:
        initializeObjects(sideC, sideB, sideA, glm::vec3(w, 0.f, 0.f));
        break;

      case GLFW_KEY_X:
        initializeObjects(sideC, sideB, sideA, glm::vec3(0.f, w, 0.f));
        break;

      case GLFW_KEY_C:
        initializeObjects(sideC, sideB, sideA, glm::vec3(0.f, 0.f, w));
        break;

      case GLFW_KEY_V:
        initializeObjects(sideC, sideB, sideA, glm::vec3(w, w, 0.f));
        break;

      case GLFW_KEY_B:
        initializeObjects(sideC, sideB, sideA, glm::vec3(w, 0.f, w));
        break;

      case GLFW_KEY_N:
        initializeObjects(sideC, sideB, sideA, glm::vec3(0.f, w, w));
        break;

      case GLFW_KEY_M:
        initializeObjects(sideC, sideB, sideA, glm::vec3(w, w, w));
        break;

      default:
        break;
    }
  }
}

void Window::mouse_callback(GLFWwindow* window, int button, int action,
                            int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    LeftDown = (action == GLFW_PRESS);
  }
  if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    RightDown = (action == GLFW_PRESS);
  }
}

void Window::cursor_callback(GLFWwindow* window, double currX, double currY) {
  int maxDelta = 100;
  int dx = glm::clamp((int)currX - MouseX, -maxDelta, maxDelta);
  int dy = glm::clamp(-((int)currY - MouseY), -maxDelta, maxDelta);

  MouseX = (int)currX;
  MouseY = (int)currY;

  // Move camera
  // NOTE: this should really be part of Camera::Update()
  if (LeftDown) {
    const float rate = 1.0f;
    Cam->SetAzimuth(Cam->GetAzimuth() + dx * rate);
    Cam->SetIncline(glm::clamp(Cam->GetIncline() - dy * rate, -90.0f, 90.0f));
  }
  if (RightDown) {
    const float rate = 0.005f;
    float dist =
        glm::clamp(Cam->GetDistance() * (1.0f - dx * rate), 0.01f, 1000.0f);
    Cam->SetDistance(dist);
  }
}
