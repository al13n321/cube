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
dvec3 SixDofInput(glfw::Window& w, const char* keys) {
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

    vector<Body*> chain;
    const double thickness = .1;
    const double length = .5;
    const double gap = .1;
    const double topy = 3;
    for (int i = 0; i < 6; ++i) {
      Body* b = scene.AddBody(MakeBox(dvec3(thickness, length, thickness)).MultiplyMass(2700));
      chain.push_back(b);
    }

    auto stop_movement = [&] {
      for (Body* b: chain) {
        b->momentum = dvec3(0, 0, 0);
        b->ang = dvec3(0, 0, 0);
      }
    };
    auto reset = [&] {
      stop_movement();
      for (int i = 0; i < (int)chain.size(); ++i) {
        Body* b = chain[i];
        b->pos = dvec3(0, topy - (length + gap)*(i + .5), 0);
        b->rot = dquat(1, 0, 0, 0);
      }
    };

    reset();

    for (int i = 0; i < (int)chain.size(); ++i)
      scene.AddConstraint(i ? chain[i-1]->idx : -1, chain[i]->idx, dvec3(0, (length + gap)*.5, 0), dquat(1, 0, 0, 0), Constraint::DOF::POS);
    scene.gravity = dvec3(0, -9.8, 0);

    chain.back()->forces.emplace_back(); auto& force = chain.back()->forces.back();

    scene.camera.pos = fvec3(-2, 2, 2.5);
    scene.camera.LookAt(chain[chain.size()/2]->pos);

    Stopwatch frame_stopwatch;
    while (!window->ShouldClose()) {
      double dt = frame_stopwatch.Restart();
      ++frame_idx;

      UpdateFPS();

      if (window->IsKeyPressed(GLFW_KEY_R))
        reset();
      if (window->IsKeyPressed(GLFW_KEY_V))
        stop_movement();

      dvec3 in = SixDofInput(*window, "IKJLUM") * 1e2;
      force.first = chain.back()->rot.Transform(dvec3(0, -(length + gap)*.5, 0)) + chain.back()->pos;
      force.second = in;

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
