#include <iostream>
#include "gl-util/glfw-util.h"
#include "util/stopwatch.h"
#include "util/quat.h"
#include "sim/scene.h"
using namespace std;

static void LogGLFWError(int code, const char *message) {
  std::cerr << "glfw error " << code << ": " << message << std::endl;
}

static glfw::Window* window;
static size_t frame_idx;

static void KeyCallback(
    GLFWwindow* w, int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS) {
    if (key == GLFW_KEY_ESCAPE) {
      window->SetShouldClose();
    }
  }
}

static void ScrollCallback(GLFWwindow *win, double dx, double dy) {
  
}

static void MouseButtonCallback(
    GLFWwindow *win, int button, int action, int mods) {
  
}

static void CursorPosCallback(GLFWwindow *w, double x, double y) {
  
}

static void UpdateFPS() {
  static size_t last_update_frame = 0;
  static Stopwatch fps_stopwatch;
  const size_t period_frames = 10;
  const double period_seconds = .5;
  if (frame_idx - last_update_frame >= period_frames ||
      fps_stopwatch.TimeSinceRestart() > period_seconds) {
    double fps = (frame_idx - last_update_frame)
      / fps_stopwatch.Restart();
    last_update_frame = frame_idx;
    window->SetTitle("FPS: " + std::to_string(fps));
  }
}

// Order of keys is: forward, backwards, left, right, up, down (-z, +z, -x, +x, +y, -y), ALL CAPS.
dvec3 ThreeDofInput(glfw::Window& w, const char* keys) {
  dvec3 r = {0, 0, 0};
  if (w.IsKeyPressed(keys[2]))
    r.x -= 1;
  if (w.IsKeyPressed(keys[3]))
    r.x += 1;
  if (w.IsKeyPressed(keys[0]))
    r.z -= 1;
  if (w.IsKeyPressed(keys[1]))
    r.z += 1;
  if (w.IsKeyPressed(keys[4]))
    r.y += 1;
  if (w.IsKeyPressed(keys[5]))
    r.y -= 1;
  return r;
}

double rnd() {
  return (((rand()+.5)/(RAND_MAX+1.)+rand())/(RAND_MAX+1.)+rand())/(RAND_MAX+1.);
}

int main() {
  try {
    glfwSetErrorCallback(&LogGLFWError);
    glfw::Initializer glfw_init;
    glfw::Window win_(ivec2(0, 0), ivec2(512, 512), "hello world", false);
    ::window = &win_;
    window->MakeCurrent();
    GL::InitGl3wIfNeeded();
    GL::LogInfo();

    window->SetKeyCallback(&KeyCallback);
    window->SetScrollCallback(&ScrollCallback);
    window->SetMouseButtonCallback(&MouseButtonCallback);
    window->SetCursorPosCallback(&CursorPosCallback);

    Scene scene;

    //Body* table = scene.AddBody(MakeBox(dvec3(3, .1, 3)));
    //table->pos.y = -.05;
    //scene.AddConstraint(-1, table->idx, dvec3(0, 0, 0), dquat(1, 0, 0, 0), Constraint::DOF::POS | Constraint::DOF::ROT);

    Body* box = scene.AddBody(MakeBox(dvec3(.06,.06,.06)).MultiplyMass(1000));
    scene.AddConstraint(-1, box->idx, dvec3(-.03, -.03, 0), dquat(1, 0, 0, 0), Constraint::DOF::POS | Constraint::DOF::RX | Constraint::DOF::RY);

    auto reset = [&] {
                   box->pos = dvec3(0, .03, 0);
                   box->rot = dquat(1, 0, 0, 0);
                   box->momentum = dvec3(0, 0, 0);
                   box->ang = dvec3(0, 0, 0);
                 };

    reset();

    //scene.gravity = dvec3(0, -9.8, 0);
    scene.gravity = dvec3(0.1, 0, 0);

    box->forces.emplace_back();
    auto& force = box->forces.back();

    scene.camera.pos = fvec3(-.1, .12, .15);
    scene.camera.LookAt(box->pos);

    Stopwatch frame_stopwatch;
    while (!window->ShouldClose()) {
      double dt = frame_stopwatch.Restart();
      ++frame_idx;

      UpdateFPS();

      if (window->IsKeyPressed(GLFW_KEY_R))
        reset();

      dvec3 in = ThreeDofInput(*window, "IKJLUM");
      force.first = box->pos;
      force.second = in * 2.2;
      //scene.gravity = dvec3(0, 0, 0) + in*2.2;

      scene.PhysicsStep(dt);

      if (frame_idx % 120 == 0) {
        cerr << "energy: " << scene.GetEnergy()
             << "; leaked:  x: " << scene.leaked_translation << ", r: " << scene.leaked_rotation
             << ", v: " << scene.leaked_velocity << ", w: " << scene.leaked_angular_velocity
             << "  res ok: " << scene.force_resolution_success << " fail: " << scene.force_resolution_failed << endl;
      }
      
      scene.Render();
      
      window->SwapBuffers();
      glfwPollEvents();
    }
  } catch (std::exception& e) {
    std::cerr << "exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
