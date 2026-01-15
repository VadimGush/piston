#include "raylib.h"
// I prefer math types and functions from GLSL, therefore I use GLM
// instead of whatever math functionality provided by Raylib.
#include "glm/vec2.hpp"
#include "glm/common.hpp"
#include "glm/trigonometric.hpp"
#include "glm/mat3x3.hpp"
#include "glm/matrix.hpp"
#include "glm/gtx/matrix_transform_2d.hpp"
#include <stdio.h>
using namespace glm;

const float EPSILON = 0.001f;
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const int TARGET_FPS = 60;

bool is_zero(const float a) { return abs(a) < EPSILON; }
float square(const float a) { return a * a; }

// Defines the engine state / dimensions / parameters
struct engine {
  
  struct crankshaft {
    // Distance between the center of the crankshaft
    // and the large end of the connecting rod.
    float radius = 50;
    float angle = 0;
  };

  // Position of the cylinder is defined by 2 vectors.
  // Those vectors describe a 2D ray on which cylinder is positioned.
  // Piston will move along that 2D ray in the positive direction.
  struct cylinder {
    vec2 origin = vec2(0, 0);
    vec2 direction = vec2(0, 20);
  };

  crankshaft crank;
  cylinder cylinder;
  float connecting_rod_length = 70;

};

// Defines a 2D camera which can be scaled, moved around and rotated.
// There are 2 type of coordinates: world coordinates and display coordinates.
// World coordinates are coordinates in which we define all vectors for calculations (millimeters).
// Display coordinates are coordinates which describe where on the screen objects are rendered (pixels).
struct view {

  mat3 view = glm::translate(glm::scale(mat3(1.), vec2(1, -1)), vec2(WINDOW_WIDTH / 2, -WINDOW_HEIGHT / 2));
  void translate(const vec2& vec) { view = glm::translate(view, vec); }
  void scale(const float& value) { view = glm::scale(view, vec2(value, value)); }

  // From world size to display size
  float transform(const float value) const {
    const vec3 v = view * vec3(value, 0, 0);
    return length(v);
  }

  // From display size to world size
  float inverse_transform(const float value) const {
    const vec3 v = inverse(view) * vec3(value, 0, 0);
    return length(v);
  }

  // From display coordinates to world coordinates
  vec2 inverse_transform(const Vector2& vector) const {
    const vec3 v = inverse(view) * vec3(vector.x, vector.y, 1.f);
    return vec2{v.x, v.y};
  }

  // From world coordinates to display coordinates
  Vector2 transform(const vec2& vector) const {
    const vec3 v = view * vec3(vector, 1.f);
    return Vector2{v.x, v.y};
  }

};

// Defines the calculated position of the piston. Depending on whether the
// piston position can be calculated at all, the position might be either valid or invalid.
struct piston_position {

  const bool is_valid = false;
  const vec2 value = vec2(0,0);

  static piston_position invalid() {
    return {false, vec2(0,0)};
  }

  static piston_position valid(const vec2& position) {
    return {true, position};
  }

  static piston_position calculate(const engine& engine) {
    // Cylinder.direction can be defined as basically any vector of any size,
    // so make sure that it is always normalized before performing calculations
    const vec2 cylinder_direction = normalize(engine.cylinder.direction);

    const float& dx = cylinder_direction.x;
    const float& dy = cylinder_direction.y;
    const float& lx = engine.cylinder.origin.x;
    const float& ly = engine.cylinder.origin.y;
    const float& r = engine.crank.radius;
    const float& rcr = engine.connecting_rod_length;
    const float& alpha = engine.crank.angle;

    // See README.md on how to derive the following equations
    const float a = square(dx) + square(dy);
    const float b = 2 * (lx * dx + ly * dy - r * dx * cos(alpha) - r * dy * sin(alpha));
    const float c = square(lx) + square(ly) - 2 * r * lx * cos(alpha) - 2 * r * ly * sin(alpha) - square(rcr) + square(r);

    // The equation is quadratic, which means it has 2 solutions. That makes sense, considering that
    // there are 2 possible positions for the piston (up and down (vertical cylinder) or left and right (horizontal cylinder)). 
    // We will always choose the largest solution that is in the positive direction of cylinder.direction.
    // If no solutions are found, connecting rod is too short and doesn't reach the cylinder.
    const float discriminant = square(b) - 4 * a * c;
    const float divisor = 2 * a;

    if (is_zero(divisor)) return piston_position::invalid();
    if (discriminant < 0) return piston_position::invalid();
    const float t = (-b + sqrt(discriminant)) / divisor;

    return piston_position::valid(engine.cylinder.origin + cylinder_direction * t);
  }

};

void draw_coordinates(const view&);
void draw_cylinder_guides(const view&, engine&);
void draw_crankshaft(const view&, const engine&);
void draw_connecting_rod(const view& view, const engine&, const vec2&);
void draw_piston(const view&, const engine&, const vec2&);

int main() {
  engine engine;
  view view;

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Piston");
  SetTargetFPS(TARGET_FPS);

  // Smooth zoom
  float zoom_speed = 0;
  float zoom_max_speed = 0.05;
  float zoom_dump = 0.8;

  while (!WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(RAYWHITE);

    // TODO: Delta should be in seconds, this way we can defined any speed as <something>/second.
    const float delta = GetFrameTime() / 0.016f;

    // Update angle of the crankshaft
    engine.crank.angle += 0.05 * delta;

    // Render the engine
    draw_coordinates(view);
    draw_cylinder_guides(view, engine);
    draw_crankshaft(view, engine);

    const auto piston_position = piston_position::calculate(engine);
    if (piston_position.is_valid) {
      draw_connecting_rod(view, engine, piston_position.value);
      draw_piston(view, engine, piston_position.value);
    }

    // Smooth zoom control
    zoom_speed = zoom_speed * (zoom_dump * delta);
    if (abs(zoom_speed) <= 0.001f) zoom_speed = 0;

    // View control
    const float move_speed = 5 / view.transform(1);
    if (IsKeyDown(KEY_W)) view.translate(vec2(0, -move_speed * delta));
    if (IsKeyDown(KEY_S)) view.translate(vec2(0, move_speed * delta));
    if (IsKeyDown(KEY_A)) view.translate(vec2(move_speed * delta, 0));
    if (IsKeyDown(KEY_D)) view.translate(vec2(-move_speed * delta, 0));
    const float mouse_wheel = GetMouseWheelMove();
    if (mouse_wheel != 0) zoom_speed = (mouse_wheel / abs(mouse_wheel)) * zoom_max_speed;
    if (zoom_speed != 0 && !(zoom_speed < 0 && view.transform(1) < 0.1f)) view.scale(1 + (zoom_speed * delta));

    EndDrawing();
  }

  CloseWindow();
  return 0;
}

void draw_cylinder_guides(const view& view, engine& params) {
  Color color{150, 150, 175, 255};

  vec2& origin = params.cylinder.origin;
  vec2& direction = params.cylinder.direction;
  const float guide_origin_radius = 20;

  const vec2 mouse_position = view.inverse_transform(GetMousePosition());

  // User is moving the origin position of the cylinder guide
  if (length(mouse_position - origin) < guide_origin_radius) {
    color = Color{100, 100, 255, 255};
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
      origin = mouse_position;
  } 

  // User is moving the direction of the cylinder guide
  if (length(mouse_position - (origin + direction)) < guide_origin_radius) {
    color = Color{100, 100, 255, 255};
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
      direction = mouse_position - origin;
  } 

  // Draw the direction of the cylinder guide
  const vec2 line_direction = direction * view.inverse_transform(1000);
  DrawLineV(view.transform(origin - line_direction), view.transform(origin + line_direction), color);

  // Draw the origin position of the cylinder guide
  DrawCircleV(view.transform(origin), view.transform(guide_origin_radius), color);
  DrawCircleV(view.transform(origin), view.transform(guide_origin_radius * 0.8), WHITE);

  // Draw the direction of the cylinder guide as a point on the screen
  DrawCircleV(view.transform(origin + direction), view.transform(guide_origin_radius), color);
  DrawCircleV(view.transform(origin + direction), view.transform(guide_origin_radius * 0.8), WHITE);
}

void draw_coordinates(const view& view) {
  const Color color{0, 0, 0, 25};
  const float size = view.inverse_transform(1000);
  DrawLineV(view.transform(vec2(-size, 0)), view.transform(vec2(size, 0)), color);
  DrawLineV(view.transform(vec2(0, -size)), view.transform(vec2(0, size)), color);

  for (int i = -1000; i < 1000; i += 10) 
    DrawLineV(view.transform(vec2(i, -5)), view.transform(vec2(i, 5)), color);
  for (int i = -1000; i < 1000; i += 10) 
    DrawLineV(view.transform(vec2(-5, i)), view.transform(vec2(5, i)), color);
}

void draw_rectangle(const view& view, const vec2& start, const vec2& end, const float width, const Color& color) {
  const vec2 direction = end - start;
  const vec2 normal = normalize(vec2(-direction.y, direction.x));
  DrawTriangle(
    view.transform(start + normal * (width/2)),
    view.transform(start - normal * (width/2)),
    view.transform(end + normal * (width/2)),
    color
  );
  DrawTriangle(
    view.transform(start - normal * (width/2)),
    view.transform(end - normal * (width/2)),
    view.transform(end + normal * (width/2)),
    color
  );
}

void draw_crankshaft(const view& view, const engine& params) {
  const Color color{50, 50, 200, 255};
  const vec2 origin = vec2(0, 0);
  const vec2 crank = vec2(cos(params.crank.angle) * params.crank.radius, sin(params.crank.angle) * params.crank.radius);
  const float bearing_size = 10;

  DrawCircleV(view.transform(origin), view.transform(bearing_size), color);
  draw_rectangle(view, origin, crank, 10, color);
  DrawCircleV(view.transform(crank), view.transform(bearing_size), color);
}

void draw_connecting_rod(const view& view, const engine& params, const vec2& piston_position) {
  const Color color{200, 50, 50, 255};
  const vec2 crank = vec2(cos(params.crank.angle) * params.crank.radius, sin(params.crank.angle) * params.crank.radius);
  const float bearing_size = 10;

  DrawCircleV(view.transform(crank), view.transform(bearing_size), color);
  draw_rectangle(view, crank, piston_position, 10, color);
  DrawCircleV(view.transform(piston_position), view.transform(bearing_size), color);
}

void draw_piston(const view& view, const engine& params, const vec2& piston_position) {
  const Color color{50, 200, 50, 255};
  const float piston_length = 30;
  const vec2 start = piston_position;
  const vec2 end = piston_position + normalize(params.cylinder.direction) * piston_length;

  draw_rectangle(view, start, end, 50, color);
}