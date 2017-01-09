#include "sim/scene.h"
using namespace std;

void Scene::PhysicsStep(double dt) {
  for (Body& body: bodies) {
    for (const auto& f: body.forces) {
      body.momentum += f.second * dt;
      body.ang += f.first.Cross(f.second * dt);
    }
    body.pos += body.momentum * (dt / body.mass);
    dvec3 av = body.inv_inertia * body.rot.Untransform(body.ang);
    body.rot += body.rot * fquat(0, av.x, av.y, av.z) * (dt * .5);
    body.rot.NormalizeMe();
  }
}
