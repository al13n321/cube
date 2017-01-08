#include "sim/scene.h"
#include "gl-util/gl-common.h"
using namespace std;

void Camera::LookAt(fvec3 p) {
  p -= pos;
  yaw = atan2(-p.x, -p.z);
  pitch = asin(p.y / p.Length());
}

fmat4 Camera::ViewProjection() const {
  // Right to left:
  //  1. translate,
  //  2. yaw,
  //  3. pitch,
  //  4. project.
  return
    fmat4::PerspectiveProjection(fov, aspect_ratio, 1e-3, 0) *
    fmat4::RotationX(-pitch) *
    fmat4::RotationY(-yaw) *
    fmat4::Translation(-pos);
}

static const char* vertex_shader = R"(
  #version 330 core
  layout(location = 0) in vec3 vert_pos;
  layout(location = 1) in vec3 vert_normal;
  layout(location = 2) in vec3 vert_color;
  uniform mat4 model_mat;
  uniform mat4 view_proj_mat;
  out vec3 pos;
  out vec3 normal;
  out vec3 color;
  void main(){
    gl_Position.xyz = vert_pos;
    gl_Position.w = 1.0;
  }
)";

static const char* fragment_shader = R"(
  #version 330 core
  uniform vec3 light_vec;
  uniform vec3 tint_color;
  in vec3 pos;
  in vec3 normal;
  in vec3 color;
  out vec3 frag_color;
  void main(){
    frag_color = vec3(.8,.2,.2);
  }
)";

Scene::Scene(): shader_("vert", "frag", vertex_shader, fragment_shader) {}

void Scene::Render(const Camera& cam) {
  glClearColor(.5, .5, 1, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  shader_.Use();
  shader_.SetVec3("light_vec", -cam.pos.Normalized());
  for (auto& body: bodies) {
    shader_.SetVec3("tint_color", body.model.tint);
    body.model.vao->Draw();
  }
}

Body* Scene::AddBody() {
  bodies.emplace_back(bodies.size());
  return &bodies.back();
}

Body* Scene::AddBody(BodyEdit edit) {
  edit.Translate(-edit.com);
  Body* b = AddBody();

  struct Vertex {
    fvec3 pos;
    fvec3 normal;
    fvec3 color;
  };

  b->mass = edit.mass;
  b->inertia = edit.inertia;
  b->model.vao.reset(new GL::VertexArray(edit.vertices.size()));

  using Attribute = GL::VertexArray::Attribute;
  vector<Attribute> attrs = {
    Attribute(0, 3, GL_FLOAT, sizeof(BodyEdit::Vertex), offsetof(BodyEdit::Vertex, pos)),
    Attribute(1, 3, GL_FLOAT, sizeof(BodyEdit::Vertex), offsetof(BodyEdit::Vertex, normal)),
    Attribute(2, 3, GL_FLOAT, sizeof(BodyEdit::Vertex), offsetof(BodyEdit::Vertex, color)),
  };
  b->model.vao->AddAttributes(attrs.size(), &attrs[0], edit.vertices.size() * sizeof(edit.vertices[0]), &edit.vertices[0]);

  return b;
}
