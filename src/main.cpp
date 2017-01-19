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
    Body* body = scene.AddBody(MakeTHandle().MultiplyMass(2700));

    // Very heavy second body linked to the first one.
    //Body* body2 __attribute__((unused)) = scene.AddBody(MakeTHandle().Scale(.5).MultiplyMass(27000));
    Body* body2 __attribute__((unused)) = scene.AddBody(MakeBox(dvec3(.04, 0.05, .06)).MultiplyMass(27000));

    auto stop_translation = [&] {
      body->momentum = body2->momentum = dvec3(0, 0, 0);
    };
    auto stop_rotation = [&] {
      body->ang = body2->ang = dvec3(0, 0, 0);
    };
    auto reset = [&] {
      stop_translation();
      stop_rotation();
      body->pos = dvec3(0, 0, 0);
      body->rot = dquat(1, 0, 0, 0);
      body2->rot = dquat(1, 0, 0, 0);

      //body2->pos = dvec3(.07, 0, .07);
      //body->ang = dvec3(0,-0.00711448,0);
    };

    reset();

    scene.AddConstraint(body->idx, body2->idx, dvec3(0, 0, 0), dquat(1, 0, 0, 0), Constraint::DOF::POS);

    body->forces.emplace_back(dvec3(-1, 0, 0), dvec3(0, 0, 0)); auto& forcenx = body->forces.back();
    body->forces.emplace_back(dvec3(+1, 0, 0), dvec3(0, 0, 0)); auto& forcepx = body->forces.back();
    body->forces.emplace_back(dvec3(0, -1, 0), dvec3(0, 0, 0)); auto& forceny = body->forces.back();
    body->forces.emplace_back(dvec3(0, +1, 0), dvec3(0, 0, 0)); auto& forcepy = body->forces.back();

    scene.camera.pos = fvec3(-.1, .1, .15);
    scene.camera.LookAt(scene.bodies.begin()->pos);

    Stopwatch frame_stopwatch;
    double energy_after_last_force = scene.GetEnergy();
    while (!window->ShouldClose()) {
      double dt = frame_stopwatch.Restart();
      ++frame_idx;

      UpdateFPS();

      bool have_forces = false;

      if (window->IsKeyPressed(GLFW_KEY_R)) {
        reset();
        have_forces = true;
      }
      if (window->IsKeyPressed(GLFW_KEY_V)) {
        stop_translation();
        have_forces = true;
      }
      if (window->IsKeyPressed(GLFW_KEY_C)) {
        stop_rotation();
        have_forces = true;
      }

      dvec3 in = SixDofInput(*window, "WSADQZ") * 1e-3;
      have_forces |= !in.IsZero();
      forcenx.second = dvec3(0, in.x, -in.y);
      forcepx.second = dvec3(0, -in.x, in.y);
      forceny.second = dvec3(0, 0, -in.z);
      forcepy.second = dvec3(0, 0, in.z);

      in = SixDofInput(*window, "IKJLUM") * 1e-1;
      have_forces |= !in.IsZero();
      forcenx.second += in;
      forcepx.second += in;

      //if (frame_idx % 120 == 0) cerr << body->ang << endl;

      scene.PhysicsStep(dt);

      if (have_forces)
        energy_after_last_force = scene.GetEnergy();
      else if (frame_idx % 120 == 0)
        cerr << "energy difference: " << (scene.GetEnergy() - energy_after_last_force) / energy_after_last_force << endl;

      if (frame_idx % 120 == 0) {
        cerr << "leaked:  x: " << scene.leaked_translation << ", r: " << scene.leaked_rotation
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
