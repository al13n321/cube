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
  out vec3 normal;
  out vec3 color;
  void main(){
    vec4 p = model_mat * vec4(vert_pos, 1);
    normal = (model_mat * vec4(vert_normal, 0)).xyz;
    color = vert_color;
    gl_Position =  view_proj_mat * p;
  }
)";

static const char* fragment_shader = R"(
  #version 330 core
  uniform vec3 light_vec;
  uniform vec3 tint_color;
  uniform float time;
  in vec3 normal;
  in vec3 color;
  out vec3 frag_color;
  void main(){
    frag_color = color * (.05 + max(0.f, dot(light_vec, normalize(normal)))) + tint_color * sin(time);
  }
)";

Scene::Scene(): shader_("vert", "frag", vertex_shader, fragment_shader) {}

void Scene::Render() {
  glClearColor(.5, .5, 1, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  //glEnable(GL_CULL_FACE);
  shader_.Use();
  shader_.SetVec3("light_vec", light_vec);
  double t = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();
  t -= floor(t/(3600*24))*(3600*24);
  shader_.SetScalar("time", t);
  shader_.SetMat4("view_proj_mat", camera.ViewProjection());
  for (auto& body: bodies) {
    shader_.SetVec3("tint_color", body.mesh.tint);
    shader_.SetMat4("model_mat", fmat4::Translation(body.pos) * body.rot.ToMatrix4());
    body.mesh.vao->Draw();
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

  b->inv_mass = 1/edit.mass;
  b->inv_inertia = edit.inertia.Inverse();
  b->mesh.vao.reset(new GL::VertexArray(edit.vertices.size()));

  using Attribute = GL::VertexArray::Attribute;
  vector<Attribute> attrs = {
    Attribute(0, 3, GL_FLOAT, sizeof(BodyEdit::Vertex), offsetof(BodyEdit::Vertex, pos)),
    Attribute(1, 3, GL_FLOAT, sizeof(BodyEdit::Vertex), offsetof(BodyEdit::Vertex, normal)),
    Attribute(2, 3, GL_FLOAT, sizeof(BodyEdit::Vertex), offsetof(BodyEdit::Vertex, color)),
  };
  b->mesh.vao->AddAttributes(attrs.size(), &attrs[0], edit.vertices.size() * sizeof(edit.vertices[0]), &edit.vertices[0]);

  return b;
}
