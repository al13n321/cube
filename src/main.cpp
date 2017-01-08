#include <iostream>
#include "gl-util/glfw-util.h"
#include "util/stopwatch.h"
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

    Stopwatch frame_stopwatch;
    while (!window->ShouldClose()) {
      //double frame_time = frame_stopwatch.Restart();
      ++frame_idx;

      UpdateFPS();

      // Render here.
      
      window->SwapBuffers();
      glfwPollEvents();
    }
  } catch (std::exception& e) {
    std::cerr << "exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
