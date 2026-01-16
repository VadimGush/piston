#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "dependencies/raygui.h"
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

// ============ ENGINE CALCULATION STRUCTURES =============

// Defines main components of the internal combustion engine 
// and its dimensions as well as other parameters.
struct engine {
  
  struct crankshaft {
    // Crank radius is the distance between the center of
    // the crankshaft and the crankpin
    float crank_radius = 50;
    vec2 crankpin_position = vec2{0,0};
    float angle = 0;
  };
  crankshaft crankshaft;

  // Position and orientation of the cylinder is defined by 2 vectors.
  // Those vectors describe a 2D ray on which cylinder is positioned.
  // Piston will move along that 2D ray in the positive direction.
  struct cylinder {
    vec2 origin = vec2(0, 0);
    vec2 direction = vec2(0, 20);
  };
  cylinder cylinder;

  // Position of the piston might exist or not exist 
  // If engine dimensions do not allow piston to reach the cylinder,
  // there is no piston position to be found
  struct piston {
    vec2 position = vec2(0,0);
    bool exists = false;
  };
  piston piston;

  float connecting_rod_length = 100;

  // Calculates the positon of the crankpin and the position of the piston
  void calculate_positions() {
    crankshaft.crankpin_position = vec2{
      cos(crankshaft.angle) * crankshaft.crank_radius, 
      sin(crankshaft.angle) * crankshaft.crank_radius
    };
    const vec2 cylinder_direction = normalize(cylinder.direction);

    // TODO: Reevalute this equation considering that crankpin_position is already calculated
    const float& dx = cylinder_direction.x;
    const float& dy = cylinder_direction.y;
    const float& lx = cylinder.origin.x;
    const float& ly = cylinder.origin.y;
    const float& r = crankshaft.crank_radius;
    const float& rcr = connecting_rod_length;
    const float& alpha = crankshaft.angle;

    const float rcos = r * cos(alpha);
    const float rsin = r * sin(alpha);
    const float a = square(dx) + square(dy);
    const float b = 2 * (lx * dx + ly * dy - dx * rcos - dy * rsin);
    const float c = square(lx) + square(ly) - 2 * lx * rcos - 2 * ly * rsin - square(rcr) + square(r);

    // The equation is quadratic, which means it has 2 solutions. That makes sense, considering that
    // there are 2 possible positions for the piston 
    // (up and down (vertical cylinder) or left and right (horizontal cylinder)). 
    // We will always choose the largest solution that is in the positive direction of cylinder.direction.
    // If no solutions are found, connecting rod is too short and doesn't reach the cylinder.
    const float discriminant = square(b) - 4 * a * c;
    const float divisor = 2 * a;

    if (is_zero(divisor)) {
      piston.exists = false;
      return;
    }
    if (discriminant < 0) {
      piston.exists = false;
      return;
    }

    const float t = (-b + sqrt(discriminant)) / divisor;
    piston.position = cylinder.origin + cylinder_direction * t;
    piston.exists = true;
  }
};

// ================== RENDER STRUCTURES ===================

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

// Describes the state of the UI components
struct interface {
  enum class component {
    NONE,
    CYLINDER_GUIDE_DIRECTION,
    CYLINDER_GUIDE_POSITION
  };

  bool show_cylinder_guides = true;
  // When we're dragging something on the screen, we don't want to accidentally
  // trigger other UI components when mouse cursor goes through them.
  // For that, we define an active component. If there is an active component set,
  // other UI components will basically ignore any mouse inputs.
  // As soon as mouse released, the active component is reset.
  component active_component = component::NONE;

  // Set an active component only if no component is active
  void set_active(component c) {
    if (active_component == component::NONE)
      active_component = c;
  }

  bool is_active(component c) {
    return active_component == c;
  }
};

void draw_coordinates(const view&);
void draw_cylinder_guides(interface& interface, const view&, engine&);
void draw_crankshaft(const view&, const engine&);
void draw_connecting_rod(const view& view, const engine&);
void draw_piston(const view&, const engine&);

// ================= MAIN IMPLEMENTATION ==================

int main() {
  engine engine;
  view view;
  interface interface;

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Piston");
  SetTargetFPS(TARGET_FPS);

  // Smooth zoom
  float zoom_speed = 0;
  float zoom_max_speed = 0.05;
  float zoom_dump = 0.8;

  // Smooth camera translate
  vec2 camera_speed = vec2{0,0};
  float camera_dumping = 0.8f;

  while (!WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(RAYWHITE);

    // === UPDATE ==
    const float delta = GetFrameTime() / 0.016f;
    engine.crankshaft.angle += 0.05 * delta;
    engine.calculate_positions();

    // === RENDER ==
    draw_coordinates(view);
    draw_crankshaft(view, engine);
    if (engine.piston.exists) {
      draw_connecting_rod(view, engine);
      draw_piston(view, engine);
    }
    if (interface.show_cylinder_guides)
      draw_cylinder_guides(interface, view, engine);

    // Reset the active component if the mouse was released
    if (IsMouseButtonUp(MOUSE_BUTTON_LEFT))
      interface.active_component = interface::component::NONE;

    // === CONTROL ===
    // Smooth zoom control
    zoom_speed = zoom_speed * (zoom_dump * delta);
    if (is_zero(abs(zoom_speed))) zoom_speed = 0;
    // Smooth camera translate
    camera_speed = camera_speed * (camera_dumping * delta);
    if (is_zero(length(camera_speed))) camera_speed = vec2{0, 0};

    // View control
    const float camera_move_speed = 1 / view.transform(1);
    if (IsKeyDown(KEY_W)) camera_speed += vec2(0, -camera_move_speed);
    if (IsKeyDown(KEY_S)) camera_speed += vec2(0, camera_move_speed);
    if (IsKeyDown(KEY_A)) camera_speed += vec2(camera_move_speed, 0);
    if (IsKeyDown(KEY_D)) camera_speed += vec2(-camera_move_speed, 0);
    const float mouse_wheel = GetMouseWheelMove();
    if (mouse_wheel != 0) zoom_speed = (mouse_wheel / abs(mouse_wheel)) * zoom_max_speed;

    if (!is_zero(length(camera_speed))) view.translate(camera_speed * delta);
    // The second condition is to prevent the case when we zoom so much, that we invert the camera coordinates
    if (!is_zero(zoom_speed) && !(zoom_speed < 0 && view.transform(1) < 0.1f)) view.scale(1 + (zoom_speed * delta));

    EndDrawing();
  }

  CloseWindow();
  return 0;
}

void draw_cylinder_guides(interface& interface, const view& view, engine& params) {
  Color color{150, 150, 175, 255};

  vec2& origin = params.cylinder.origin;
  vec2& direction = params.cylinder.direction;
  const float guide_origin_radius = 20;

  // Get position of the mouse in world coordinates
  const vec2 mouse_position = view.inverse_transform(GetMousePosition());

  // User is moving the origin position of the cylinder guide
  const bool guide_position_active = length(mouse_position - origin) < guide_origin_radius;
  if (guide_position_active && IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    interface.set_active(interface::component::CYLINDER_GUIDE_POSITION);
  if (interface.is_active(interface::component::CYLINDER_GUIDE_POSITION)) {
    color = Color{100, 100, 255, 255};
    origin = mouse_position;
  } 

  // User is moving the direction of the cylinder guide
  const bool guide_direction_active = length(mouse_position - (origin + direction)) < guide_origin_radius;
  if (guide_direction_active && IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    interface.set_active(interface::component::CYLINDER_GUIDE_DIRECTION);
  if (interface.is_active(interface::component::CYLINDER_GUIDE_DIRECTION)) {
    color = Color{100, 100, 255, 255};
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

void draw_crankshaft(const view& view, const engine& engine) {
  const Color color{50, 50, 200, 255};
  const vec2 origin = vec2(0, 0);
  const float bearing_size = 10;

  DrawCircleV(view.transform(origin), view.transform(bearing_size), color);
  draw_rectangle(view, origin, engine.crankshaft.crankpin_position, 10, color);
  DrawCircleV(view.transform(engine.crankshaft.crankpin_position), view.transform(bearing_size), color);
}

void draw_connecting_rod(const view& view, const engine& engine) {
  const Color color{200, 50, 50, 255};
  const float bearing_size = 10;

  DrawCircleV(view.transform(engine.crankshaft.crankpin_position), view.transform(bearing_size), color);
  draw_rectangle(view, engine.crankshaft.crankpin_position, engine.piston.position, 10, color);
  DrawCircleV(view.transform(engine.piston.position), view.transform(bearing_size), color);
}

void draw_piston(const view& view, const engine& engine) {
  const Color color{50, 200, 50, 255};
  const float piston_length = 30;
  const vec2 start = engine.piston.position;
  const vec2 end = engine.piston.position + normalize(engine.cylinder.direction) * piston_length;

  draw_rectangle(view, start, end, 50, color);
}