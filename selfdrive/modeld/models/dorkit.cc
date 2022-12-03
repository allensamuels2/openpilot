 #include "selfdrive/modeld/models/driving.h"

#include <fcntl.h>
#include <unistd.h>

#include <cassert>
#include <cstring>
#include <cmath>
#include <cfloat>
#include <thread>
#include <sstream>
#include <iomanip>
#include <sys/time.h>
#include <iostream>

#define M_SQRT2_2 (M_SQRT2 / 2.0)

#include "selfdrive/common/clutil.h"
#include "selfdrive/common/params.h"
#include "selfdrive/common/timing.h"
#include "selfdrive/common/swaglog.h"

#include <capnp/schema.h>
#include <capnp/dynamic.h>
using ::capnp::DynamicValue;
using ::capnp::DynamicStruct;
using ::capnp::DynamicEnum;
using ::capnp::DynamicList;
using ::capnp::List;
using ::capnp::Schema;
using ::capnp::StructSchema;
using ::capnp::EnumSchema;

using ::capnp::Void;
using ::capnp::Text;

#include "dorkit.h"
#include "simple_cpp_sockets.h"
static bool nearlyEqual(double l, double r) {
  return fabs(l-r) < .00001;
}      

struct Vec2 {
  double x, y;
  Vec2(double _x = 0.0, double _y = 0.0) : x(_x), y(_y) {}
  Vec2 operator+(const Vec2& rhs) const { return Vec2(x + rhs.x, y + rhs.y); }
  Vec2 operator-(const Vec2& rhs) const { return Vec2(x - rhs.x, y - rhs.y); }
  Vec2 operator*(double s)        const { return Vec2(s * x, s * y); }
  // Use approximate comparison operator
  bool operator==(const Vec2& rhs) const { return nearlyEqual(x,rhs.x) && nearlyEqual(y, rhs.y); }
  bool operator!=(const Vec2& rhs) const { return !((*this) == rhs); }
  friend std::ostream& operator<<(std::ostream& os, const Vec2& v) {
    return os << "(X:" << v.x << ",Y:" << v.y << ")";
  }
};

struct Vec3 : private Vec2 {
  double z;
  using Vec2::x;    // make public
  using Vec2::y;    // make public
  Vec3(double _x = 0.0, double _y = 0.0, double _z = 0.0) : Vec2(_x, _y), z(_z) {}
  bool operator==(const Vec3& rhs) const { return this->Vec2::operator==(rhs) && nearlyEqual(z, rhs.z); }
  bool operator!=(const Vec3& rhs) const { return !((*this) == rhs); }
  friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
    return os << "(X:" << v.x << ",Y:" << v.y << ", Z:" << v.z << ")";
  }
};

struct Vec4 : private Vec3 {
  double t;
  using Vec3::x;    // make public
  using Vec3::y;    // make public
  using Vec3::z;    // make public
  Vec4(double _x = 0.0, double _y = 0.0, double _z = 0.0, double _t = 0.0) : Vec3(_x, _y, _z), t(_t) {}
  bool operator==(const Vec4& rhs) const { return this->Vec3::operator==(rhs) && nearlyEqual(t, rhs.t); }
  bool operator!=(const Vec4& rhs) const { return !((*this) == rhs); }
  friend std::ostream& operator<<(std::ostream& os, const Vec4& v) {
    return os << "(X:" << v.x << ",Y:" << v.y << ", Z:" << v.z << ", T:" << v.t << ")";
  }
};

struct RPY : private Vec3 {
  RPY(double r = 0.0, double p = 0.0, double y = 0.0) : Vec3(r,p,y) {}
  double& roll()  { return x; }
  double& pitch() { return y; }
  double& yaw()   { return z; }
  const double& roll()  const { return x; }
  const double& pitch() const { return y; }
  const double& yaw()   const { return z; }
  friend std::ostream& operator<<(std::ostream& os, const RPY& v) {
    return os << "(R:" << v.roll() << ",P:" << v.pitch() << ", Y:" << v.yaw() << ")";
  }
  bool operator==(const RPY& rhs) const { return this->Vec3::operator==(rhs); }
  bool operator!=(const RPY& rhs) const { return !RPY::operator==(rhs); }
};

struct UnitVector : public Vec2 {
        double& cos()       { return x; };
  const double& cos() const { return x; };
        double& sin()       { return y; }
  const double& sin() const { return y; }
  explicit UnitVector(double angle) {
    sincos(angle, &sin(), &cos());
  }
  UnitVector() : Vec2(INFINITY, INFINITY) {}
};

double circleRadiusFromThreePoints(const Vec2 a, const Vec2 b, const Vec2 c) {
    Vec2 d1 = Vec2(b.y - a.y, a.x - b.x);
    Vec2 d2 = Vec2(c.y - a.y, a.x - c.x);
    double k = d2.x * d1.y - d2.y * d1.x;
    if (k > -0.00001 && k < 0.00001) {
        return std::numeric_limits<double>::infinity();
    }
    Vec2 s1 = Vec2((a.x + b.x) / 2, (a.y + b.y) / 2);
    Vec2 s2 = Vec2((a.x + c.x) / 2, (a.y + c.y) / 2);
    double l = d1.x * (s2.y - s1.y) - d1.y * (s2.x - s1.x);
    double m = l / k;
    Vec2 center = Vec2(s2.x + m * d2.x, s2.y + m * d2.y);
    double dx = center.x - a.x;
    double dy = center.y - a.y;
    double radius = sqrt(dx * dx + dy * dy);
    return radius;
  
}

Vec2 rotatePointAroundPoint(const Vec2& center, const Vec2& point, const UnitVector& angle) {
  Vec2 p(point - center); // Move to center
  Vec2 rotated((p.x * angle.cos()) - (p.y * angle.sin()), (p.x * angle.sin()) + (p.y * angle.cos()));
  return rotated + center;
}

//
// Move a distance from a point using a heading
//
Vec2 moveHeading(const Vec2& start, double distance, const UnitVector& heading) {
  return start + (heading * distance);
}

//
// Compute turning radius and turning center, assuming that the center of the front wheels is 0,0 in X/Y plane
//
struct BicycleModel {
  //
  // Create the model.
  //
  BicycleModel(double wheel_base) {
    memset(this, 0, sizeof(*this));
    m_wheel_base = wheel_base;
  }

  BicycleModel(double steer_angle, double wheel_base) : BicycleModel(wheel_base) {
    setSteeringAngle(steer_angle);
  }

  //
  // You can do this multiple times.... but it resets the Distance travelled
  //
  void setSteeringAngle(double steer_angle) {
    m_steer_angle = steer_angle;
    m_turning_radius =  m_wheel_base / abs(sin(m_steer_angle));
    if (std::isinf(m_turning_radius)) {
      // going straight
      m_turning_center_y = 0;
    } else {
      // Now we have a triangle, compute the third side
      m_turning_center_y = sqrt(m_turning_radius * m_turning_radius - m_wheel_base * m_wheel_base);
    }
    if (m_steer_angle < 0) m_turning_center_y = -m_turning_center_y;
    // Valid after distance is set.
    setDistance(0.0);
  }
  //
  // Set the distance travelled from 0. Maybe repeatedly called.
  //
  void setDistance(double distance) {
      if (std::isinf(m_turning_radius)) {
        m_rotation_angle = 0.0;
        m_rotation_vector = UnitVector(0);
        m_position = Vec2(distance, 0.0);
      } else {
        // First compute rotation angle
        double circumference = 2.0 * M_PI * m_turning_radius;
        double fraction_of_circumference = fmod(distance / circumference, 1.0);
        m_rotation_angle = 2.0 * M_PI * fraction_of_circumference;
        if (m_steer_angle < 0.0) {
          m_rotation_angle = -m_rotation_angle;
        }
        m_rotation_vector = UnitVector(m_rotation_angle);
        m_position = rotatePointAroundPoint(getTurningCenter(), Vec2(0,0), m_rotation_vector);
      }
  }
  //
  // Geo Information available
  //

  bool isStraight() const { return std::isinf(m_turning_radius); }
  Vec2 getTurningCenter() const { return Vec2(-m_wheel_base, m_turning_center_y); }
  double getTurningRadius() const { return m_turning_radius; }
  double getWheelBase() const { return m_wheel_base; }

  Vec2 getPosition() const { return m_position; }
  //
  // Get position of a theoretical wheel, negative distances are for "left" side.
  //
  Vec2 getWheelPosition(double distance) {
    return moveHeading(m_position, distance, UnitVector(m_rotation_angle + M_PI_2));
  }

  double getRotationAngle() const { return m_rotation_angle; }
  UnitVector getRotationVector() const { return m_rotation_vector; }

  double getAccelerationAngle() const { return isStraight() ? 0.0 : m_rotation_angle + (m_steer_angle < 0 ? -M_PI_2 : M_PI_2); }
  UnitVector getAccelerationVector() const { return UnitVector(getAccelerationAngle()); }

  double getSteerAngle() const { return m_steer_angle; }

private:
  double m_steer_angle;
  double m_wheel_base;
  double m_turning_radius;
  double m_turning_center_y;
  double m_rotation_angle;
  Vec2 m_position;
  UnitVector m_rotation_vector;
};

struct T_variables {
  double t;       // Time
  double v_0;     // velocity at time 0
  double a;       // acceleration
  double v_t;     // velocity at time t
  double v_avg;   // average velocity in [0, t]
  double d;       // Distance travelled
  Vec2 position;
  Vec2 velocity;
  Vec2 acceleration;
  RPY orientation;
  RPY orientationRate;

  T_variables() { memset(this, 0, sizeof(*this)); }
  //
  // Compute the t-variables at a specific time
  //
  T_variables(double _t, double _v_0, double _a, BicycleModel& bike) : t(_t), v_0(_v_0), a(_a) {
    v_t = v_0 + (a * t);          // Velocity at time t.
    v_avg = (v_t + v_0) / 2.0;    // Average velocity
    d = v_avg * t;                // Distance travelled

    bike.setDistance(d);

    position = bike.getPosition();
    velocity = bike.getRotationVector() * v_avg;
    if (std::isinf(bike.getTurningRadius()) || t == 0.0) {
      acceleration = UnitVector(0.0) * a;
      orientation = RPY(0.0, 0.0, 0.0);
      orientationRate = RPY(0.0, 0.0, 0.0);
    } else {
      // We just account for centripetal acceleration and ignore the linear acceleration component
      // Centripal = V**2 / r
      double acceleration_magnitude = (v_avg * v_avg) / bike.getTurningRadius();
      acceleration = bike.getAccelerationVector() * acceleration_magnitude;
      orientation = RPY(0.0, 0.0, bike.getRotationAngle());
      //
      // Compute the orientationRate (angular velocity). this is in Radians/second.
      // orientationRate = 2 * PI * f
      // where f is in units of circles / second => velocity / circumference
      // W = 2 * PI * (V / (2 * PI * R)) => V / R
      //
      auto w = v_avg / bike.getTurningRadius();
      orientationRate = RPY(0.0, 0.0, bike.getSteerAngle() < 0 ? -w : w);
    }
  }
};

struct X_variables {
  double x;
  double v;
  double a;
  Vec4 laneLines[4]; // outer-left, inner-left, inner-right, outer-right
  Vec4 roadEdges[2];
  X_variables() { memset(this, 0, sizeof(*this)); }
  //
  // Edges: [left road, outer left lane, inner left lane, ]
  X_variables(double _x, double _v, double _a, const BicycleModel& bike, double roadwidth, double lane_width, double lane_marker_width) : X_variables() {
    x = _x;
    v = _v;
    a = _a;
    roadEdges[0] = intersect(x, v, a, bike, -roadwidth/2);
    roadEdges[1] = intersect(x, v, a, bike,  roadwidth/2);
    laneLines[0] = intersect(x, v, a, bike, -(lane_width / 2.0) - lane_marker_width/2.0 );
    laneLines[1] = intersect(x, v, a, bike, -(lane_width / 2.0) + lane_marker_width/2.0 );
    laneLines[2] = intersect(x, v, a, bike,  (lane_width / 2.0) - lane_marker_width/2.0 );
    laneLines[3] = intersect(x, v, a, bike,  (lane_width / 2.0) + lane_marker_width/2.0 );
  }

  static Vec4 intersect(double x, double v, double a, const BicycleModel& bike, double offset) {
    if (bike.isStraight()) {
      return Vec4(x, offset, 0.0, x / v);
    }
    // Make a circle with an adjusted radius, then intersect it with X to yield the Y value, then compute t.
    //
    double new_radius = offset + bike.getTurningRadius();
    Vec2 center = bike.getTurningCenter();
    //  R^2 = (X-h)^2 + (Y-k)^2, setting h = center.x and k = center.y and X = x, solve for Y
    //  R^2 - (X-center.x)^2 = (Y-center.y)^2
    //  +/-sqrt(R^2 - (X-center.x)^2) + center.y = Y
    double x_offset = x - center.x;
    double _sqrt = sqrt(new_radius*new_radius - x_offset*x_offset);
    double y1 = _sqrt + center.y;
    double y2 = -_sqrt + center.y;
    // we want the one that's closer to the origin
    double y = fabs(y1) < fabs(y2) ? y1 : y2;
    return Vec4(x, y, 0.0, x / v);
  }
};

//
// Compare two messages for structural equivalence. Same fields, same types, but values can be different.
//
static void dynamic_structure_compare(const DynamicValue::Reader l, const DynamicValue::Reader r) {
  assert(l.getType() == r.getType());
  switch (l.getType()) {
    case DynamicValue::VOID:
    case DynamicValue::BOOL:
    case DynamicValue::INT:
    case DynamicValue::UINT:
    case DynamicValue::FLOAT:
    case DynamicValue::TEXT:
    case DynamicValue::ENUM:
      break;
    
    case DynamicValue::LIST: {
        auto ll = l.as<DynamicList>();
        auto rr = r.as<DynamicList>();
        assert(size_t(ll.end() - ll.begin()) == size_t(rr.end() - rr.begin()));
        auto li = ll.begin();
        auto ri = rr.begin();
        while (li != ll.end()) {
          dynamic_structure_compare(*li, *ri);
          li++;
          ri++;
        }
      }
      break;
    case DynamicValue::STRUCT: {
        auto ll = l.as<DynamicStruct>();
        auto rr = r.as<DynamicStruct>();
        auto lsf = ll.getSchema().getFields();
        auto rsf = rr.getSchema().getFields();
        assert(size_t(lsf.end() - lsf.begin()) == size_t(rsf.end() - rsf.begin()));
        auto lfield = lsf.begin();
        auto rfield = rsf.begin();
        while (lfield != lsf.end()) {
          assert(lfield->getProto().getName().cStr() == rfield->getProto().getName().cStr());
          if (ll.has(*lfield) && rr.has(*rfield)) {
            dynamic_structure_compare(ll.get(*lfield), rr.get(*rfield));
          }
          lfield++;
          rfield++;
        }
      }
      break;
    default:
      assert(false);

  }
}

void compare_message(const cereal::ModelDataV2::Builder& l, const cereal::ModelDataV2::Builder& r) {
  dynamic_structure_compare(l.asReader(), r.asReader());
}

std::string fmt_degrees(float radians) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(4) << (radians / (M_PI / 180)) << "d";
  return os.str();
}

std::string fmt_meters(float meters) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(2) << meters << "m";
  return os.str();
}

const int port = 6379;

Socket *log_socket = nullptr;
bool show_msg = false;
std::atomic<unsigned> active_connections(0);

struct fake_variables {
  float current_steering = 0.0;
  float future_steering = 0.0;
  bool  autocorrect_steering = true;
  float v = 20.0;
  float a = 0.0;
  float roadwidth = 8.0;    // typical road width
  float lane_width = 3.7;   // 12 feet
  float lane_marker_width = .101;  // 4 inches
  float s_incr = .0017;
  float s_limit = 0.244346;       // Maximum steering deflection
  float wheel_base = 2.78892;  // Highlander wheel base is 109.8 inches => 2.78892 meters.
  float getCorrectedSteering() const;
  void updateCorrectedSteering();
} fake;

bool socket_is_active() { return active_connections != 0; }

//
// Circle translation table
//
class circle_correction {
  float slope;
  float intercept;
public:
  circle_correction(float _slope = 1.0f, float _intercept = 0.0f) : slope(_slope), intercept(_intercept) {}
  float correct(float radius) {
    return (radius * slope) + intercept;
  }

};

float mph_to_ms(float mph) {
  return mph * .44704;
}

std::map<float, circle_correction> correction_table;

void intialize_correction_table() {
  correction_table.clear();
  correction_table[mph_to_ms(0)]  = circle_correction(10.37, -133.39); // Invented.
  correction_table[mph_to_ms(6)]  = circle_correction(10.37, -133.39);
  correction_table[mph_to_ms(11)] = circle_correction(5.205, -26.45);
  correction_table[mph_to_ms(14)] = circle_correction(4.95, -36.44);
  correction_table[mph_to_ms(17)] = circle_correction(4.099, -35.74);
  correction_table[mph_to_ms(30)] = circle_correction();
}

float correct_circle_radius(float ms, float requested) {
  if (correction_table.empty()) return requested;
  if (ms <= correction_table.begin()->first) {
    // <= first entry, no interpolation
    return correction_table.begin()->second.correct(requested);
  } else if (ms >= correction_table.rbegin()->first) {
    // >= first entry, interpolate with unity
    return correction_table.rbegin()->second.correct(requested);
  } else {
    auto high = correction_table.lower_bound(ms);
    assert(high != correction_table.end());
    assert(high != correction_table.begin());
    auto low = std::prev(high);
    assert(ms >= low->first);
    assert(ms <= high->first);
    auto diff = high->first - low->first;
    assert(diff > 0.0);
    auto alpha = (ms - low->first) / diff;
    assert(alpha >= 0.0 && alpha <= 1.0);
    // std::cerr << "low:" << low->first << " high:" << high->first << " diff:" << diff << " ms:" << ms << " alpha:" << alpha << "\n";
    auto high_circle = high->second.correct(requested);
    auto low_circle = low->second.correct(requested);
    // std::cerr << "high_circle:" << high_circle << " low_circle:" << low_circle;
    // std::cerr << " Result:" << ((1-alpha) * high_circle) + (alpha * low_circle) << "\n";
    return ((1-alpha) * low_circle) + (alpha * high_circle);
  }
}

float fake_variables::getCorrectedSteering() const {
  // Convert steering angle to radius.
  if (!autocorrect_steering) return current_steering;
  auto requested_radius = BicycleModel(current_steering, wheel_base).getTurningRadius();
  auto corrected_radius = correct_circle_radius(v, requested_radius);
  auto angle = std::asin(wheel_base / corrected_radius);
  return current_steering < 0 ? -angle : angle;
}

void fake_variables::updateCorrectedSteering() {
  // Move current_steering toward future_steering by no more than fake.s_incr
  if (current_steering < future_steering) {
    float diff = future_steering - current_steering;
    current_steering += std::min(fake.s_incr, diff);
    current_steering = std::min(current_steering, fake.s_limit);
  } else {
    float diff = current_steering - future_steering;
    current_steering -= std::min(fake.s_incr, diff);
    current_steering = std::max(current_steering, -fake.s_limit);
  }
}

std::string helpText() {
  std::ostringstream os;
  os << 
    "\r\n"
    "z                           Zero steering deflection\r\n"
    "l                           bump steering to left\r\n"
    "r                           bump steering to right\r\n"
    "s      <degrees>            set steering angle (uncorrected)\r\n"
    "S      <degrees>            set steering angle (Corrected)\r\n"
    "wb     <meters>             set wheel base\r\n"
    "c      <meters>             set steering for circle of indicated radius (Uncorrected)\r\n"
    "C      <meters>             set steering for circle of indicated radius (Corrected)\r\n"
    "rw     <meters>             set road width\r\n"
    "lw     <meters>             set lane width\r\n"
    "lmw    <meters>             set lane marker width\r\n"
    "ate    <mph> <slope> <offset>  add table entry\r\n"
    "rt                          reset table\r\n"
    "rl     <degrees/frame>      set steer increment (rate limit)\r\n"
    "sl     <degrees>            set steer limit(+/- deflection limit)\r\n"
    "\r\n"
    "  -- Debug Only Commands, not functional in vehicle --\r\n"
    "\r\n"
    "v      <speed>              Set speed\r\n"
    "time   <count>              Simulate <count> frame times\r\n"
    "msg                         show internal DL messages\r\n"
    "help or h                   show this message"
    "\r\n";
  os 
    << ">>>> Current Settings: <<<<\r\n"
    << "Current Steering   : " << fmt_degrees(fake.current_steering) << "\r\n"
    << "Future  Steering   : " << fmt_degrees(fake.future_steering) << "\r\n"
    << "Corrected Steering : " << fmt_degrees(fake.getCorrectedSteering()) << "\r\n"
    << "Steer Increment    : " << fmt_degrees(fake.s_incr)  << "\r\n"
    << "Steer Limit        : " << fmt_degrees(fake.s_limit) << "\r\n"
    << "Road Width         : " << fmt_meters(fake.roadwidth) << "\r\n"
    << "Lane Width         : " << fmt_meters(fake.lane_width) << "\r\n"
    << "Lane Marker Width  : " << fmt_meters(fake.lane_marker_width) << "\r\n"
    << "Wheel Base         : " << fmt_meters(fake.wheel_base) << "\r\n"
    << "\r\n";
  return os.str();
}

std::string parse_error;

template<typename t>
bool parse_degrees(std::istream& is, t& value, std::string error_msg = "degrees") {
  t temp;
  is >> temp;
  if (!is.bad()) {
    value = temp * (M_PI / 180);
    return false;
  } else {
    parse_error += error_msg;
    return true;
  }
}
template<typename t>
bool parse_number(std::istream& is, t& value, t min_value = -std::numeric_limits<t>::infinity(), t max_value = std::numeric_limits<t>::infinity(), std::string error_msg = "number") {
  t temp;
  is >> temp;
  if (!is.bad() && temp >= min_value && temp <= max_value) {
    value = temp;
    return false;
  } else {
    parse_error += error_msg;
    return true;
  }
}

static void show_status(Socket &rcv) {
      std::ostringstream os;
      os  << "Speed:"  << fmt_meters(fake.v) << "/s"
          << " Accel:" << fmt_meters(fake.a) << "/s^2"
          << " RawSteer:" << fmt_degrees(fake.current_steering)
          << " FutureSteer:" << fmt_degrees(fake.future_steering)
          << " Steer:" << fmt_degrees(fake.getCorrectedSteering())
          << " Circle:" << fmt_meters(BicycleModel(fake.getCorrectedSteering(), fake.wheel_base).getTurningRadius())
          << "\r\n";
      LOGE("%s", os.str().c_str());
      rcv.send(os.str());
}

static void status_ticker() {
  while (true) {
    usleep(10000000); // 10 seconds
    if (log_socket) {
      show_status(*log_socket);
    }
  }
}

static void process_cmd(Socket& rcv, std::string line) {
  LOGE("Got cmd: %s", line.c_str());
  //
  // Execute command
  //
  parse_error = "";
  std::istringstream is(line);
  std::string cmd;
  std::string prefix;
  is >> cmd;
  bool error = false;
  //
  // See if we have a prefix
  //
  if (!cmd.empty() && cmd[0] == '<') {
    auto rbracket = cmd.find('>');
    if (rbracket == std::string::npos) {
      parse_error = "Expecting suffix after prefix";
      error = true;
    } else {
      prefix = cmd.substr(0, rbracket+1);
      cmd = cmd.substr(rbracket+1);
    }
  }
  if (cmd == "" || cmd == "\n" || cmd == "\r\n") {
    ; // end of line?
  } else if (cmd == "q" || cmd == "quit") {
    ;
  } else if (cmd == "v") {
    error = parse_number(is, fake.v, 0.f, 35.7f); // [0..100 MPH]
  } else if (cmd == "z") {
    fake.current_steering = fake.future_steering = 0;
  } else if (cmd == "rl") {
    error = parse_degrees(is, fake.s_incr);
  } else if (cmd == "s" || cmd == "S") {
    error = parse_degrees(is, fake.future_steering);
    if (!error)fake.autocorrect_steering = (cmd == "S");
  } else if (cmd == "wb") {
    error = parse_number(is, fake.wheel_base, 1.0f, 10.0f);
  } else if (cmd == "lmw") {
    error = parse_number(is, fake.lane_marker_width, 0.f, 1.0f);
  } else if (cmd == "lw") {
    error = parse_number(is, fake.lane_width, 2.0f, 10.0f);
  } else if (cmd == "l") {
    fake.current_steering -= fake.s_incr;
  } else if (cmd == "r") {
    fake.current_steering += fake.s_incr;
  } else if (cmd == "rl") {
    error = parse_number(is, fake.s_incr, 0.0f, .01f);
  } else if (cmd == "sl") {
    error = parse_degrees(is, fake.s_limit);
  } else if (cmd == "c" || cmd == "C") {
    float radius = 0.0;
    error = parse_number(is, radius);
    error |= abs(radius) <= fake.wheel_base;
    if (!error) {
      fake.future_steering = std::asin(fake.wheel_base / radius);
      fake.autocorrect_steering = (cmd == "C");
    }
  } else if (cmd == "ate") {
    float mph, slope, offset;
    error = parse_number(is, mph, 0.f, 100.f, "mph");
    error |= parse_number(is, slope, -100.f, 100.f, "slope");
    error |= parse_number(is, offset, -100.f, 100.f, "offset");
    if (!error) {
      correction_table[mph_to_ms(mph)] = circle_correction(slope, offset);
    }
  } else if (cmd == "rt") {
    intialize_correction_table();
  } else if (cmd == "time") {
    size_t ticks;
    error = parse_number(is, ticks, 0ul, 1000ul, "ticks");
    if (!error) {
      for (auto t = 0; t < ticks; ++t) {
        fake.updateCorrectedSteering();
      }
      show_msg = true;
    }
  } else if (cmd == "msg") {
    show_msg = true;
    while (show_msg) usleep(50000);  // 50mSec
  } else if (cmd == "help" || cmd == "h") {
    rcv.send(helpText());
  } else {
    LOGE("Unknown Command: '%s'\r\n",line.c_str());
    rcv.send("Unknown Command: " + line + "\r\n");
  }
  if (error) { 
    LOGE("Parse Error(%s) for command: %s",parse_error.c_str(), line.c_str());
    rcv.send(helpText());
  } else if (!prefix.empty()) {
    rcv.send(prefix + "\r\n");
  }
}

static std::string strip_char(const std::string& inp, char c) {
  std::string out;
  for (auto i = 0; i < inp.length(); ++i) if (inp[i] != c) out += inp[i];
  return out;
}

static void process_line(Socket& rcv, std::string& line) {
  //
  // Parse line and execute
  //
  auto nl = line.find("\n");
  while (nl != std::string::npos) {
    auto cmd = line.substr(0, nl);
    process_cmd(rcv, cmd);
    line = line.substr(nl+1);
    nl = line.find("\n");
  }
  show_status(rcv);
}

static void handle_conn(Socket rcv) {
  intialize_correction_table();
  active_connections++;
  log_socket = &rcv;
  std::string command_line;
  try {
    show_status(rcv);
    while (true) {
        LOGE("Waiting for input on connection %s", rcv.format().c_str());
        std::string chunk = rcv.recv();
        command_line.append(strip_char(chunk,'\r'));
        process_line(rcv, command_line);
    }
  } catch(recv_err) {
    LOGE(("Closing socket" + rcv.format()).c_str());
  }
  active_connections--;
  log_socket = nullptr;
}

static void socket_listener() {
  LOGE(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Listening on port %d", port);
    TCPServer server(port);
    try {
        server.bind();
    } catch(bind_err) {
        LOGE("Can't bind to port %d", port);
        return;
    }
    while (true) {
        LOGE("Waiting to connect on port %d", port);
        Socket rcv = server.accept();
        LOGE("Got connection %s",rcv.format().c_str());
        std::thread handler(handle_conn, rcv);
        handler.detach();
    }
}

template<class T, size_t size>
constexpr const kj::ArrayPtr<const T> to_kj_array_ptr(const std::array<T, size> &arr) {
  return kj::ArrayPtr(arr.data(), arr.size());
}

template<size_t size>
void fill_xyzt(cereal::ModelDataV2::XYZTData::Builder xyzt, const std::array<float, size> &t,
               const std::array<float, size> &x, const std::array<float, size> &y, const std::array<float, size> &z) {
  xyzt.setT(to_kj_array_ptr(t));
  xyzt.setX(to_kj_array_ptr(x));
  xyzt.setY(to_kj_array_ptr(y));
  xyzt.setZ(to_kj_array_ptr(z));
}

template<size_t size>
void fill_xyzt(cereal::ModelDataV2::XYZTData::Builder xyzt, const std::array<float, size> &t,
               const std::array<float, size> &x, const std::array<float, size> &y, const std::array<float, size> &z,
               const std::array<float, size> &x_std, const std::array<float, size> &y_std, const std::array<float, size> &z_std) {
  fill_xyzt(xyzt, t, x, y, z);
  xyzt.setXStd(to_kj_array_ptr(x_std));
  xyzt.setYStd(to_kj_array_ptr(y_std));
  xyzt.setZStd(to_kj_array_ptr(z_std));
}

void override_message(
                      cereal::ModelDataV2::Builder &nmsg, 
                      const std::array<float, TRAJECTORY_SIZE> &t_idxs_float,
                      const std::array<float, TRAJECTORY_SIZE> &x_idxs_float) {
  fake.updateCorrectedSteering();
  BicycleModel bm(fake.getCorrectedSteering(), fake.wheel_base);
  //
  // First do the "T" variables
  //
  std::array<float, TRAJECTORY_SIZE> 
      pos_x, pos_y, pos_z, pos_x_std, pos_y_std, pos_z_std,
      vel_x, vel_y, vel_z,
      rot_x, rot_y, rot_z,
      rot_rate_x, rot_rate_y, rot_rate_z;

  for (size_t i = 0; i < TRAJECTORY_SIZE; ++i) {
    T_variables tv(t_idxs_float[i], fake.v, fake.a, bm);
    pos_x[i] = tv.position.x;
    pos_y[i] = tv.position.y;
    pos_z[i] = .1;
    pos_x_std[i] = 0;
    pos_y_std[i] = 0;
    pos_z_std[i] = 0;
    vel_x[i] = tv.velocity.x;
    vel_y[i] = tv.velocity.y;
    vel_z[i] = 0;
    rot_x[i] = tv.orientation.roll();
    rot_y[i] = tv.orientation.pitch();
    rot_z[i] = tv.orientation.yaw();
    rot_rate_x[i] = tv.orientationRate.roll();
    rot_rate_y[i] = tv.orientationRate.pitch();
    rot_rate_z[i] = tv.orientationRate.yaw();
  }
  fill_xyzt(nmsg.initPosition(), t_idxs_float, pos_x, pos_y, pos_z, pos_x_std, pos_y_std, pos_z_std);
  fill_xyzt(nmsg.initVelocity(), t_idxs_float, vel_x, vel_y, vel_z);
  fill_xyzt(nmsg.initOrientation(), t_idxs_float, rot_x, rot_y, rot_z);
  fill_xyzt(nmsg.initOrientationRate(), t_idxs_float, rot_rate_x, rot_rate_y, rot_rate_z);

  // lane lines
  std::array<float, TRAJECTORY_SIZE> left_far_y, left_far_z;
  std::array<float, TRAJECTORY_SIZE> left_near_y, left_near_z;
  std::array<float, TRAJECTORY_SIZE> right_near_y, right_near_z;
  std::array<float, TRAJECTORY_SIZE> right_far_y, right_far_z;
  // road edges
  std::array<float, TRAJECTORY_SIZE> left_y, left_z;
  std::array<float, TRAJECTORY_SIZE> right_y, right_z;
  // compute the "T" values for each X index
  std::array<float, TRAJECTORY_SIZE> plan_t;
  std::fill_n(plan_t.data(), plan_t.size(), NAN);
  plan_t[0] = 0.0;
  for (int xidx=1, tidx=0; xidx<TRAJECTORY_SIZE; xidx++) {
    // increment tidx until we find an element that's further away than the current xidx
    for (int next_tid = tidx + 1; next_tid < TRAJECTORY_SIZE && pos_x[next_tid] < X_IDXS[xidx]; next_tid++) {
      tidx++;
    }
    if (tidx == TRAJECTORY_SIZE - 1) {
      // if the Plan doesn't extend far enough, set plan_t to the max value (10s), then break
      plan_t[xidx] = T_IDXS[TRAJECTORY_SIZE - 1];
      break;
    }

    // interpolate to find `t` for the current xidx
    float current_x_val = pos_x[tidx];
    float next_x_val = pos_x[tidx+1];
    float p = (X_IDXS[xidx] - current_x_val) / (next_x_val - current_x_val);
    plan_t[xidx] = p * T_IDXS[tidx+1] + (1 - p) * T_IDXS[tidx];
  }


  for (size_t j = 0; j < TRAJECTORY_SIZE; ++j) {
    X_variables xv(x_idxs_float[j], fake.v, fake.a, bm, fake.roadwidth, fake.lane_width, fake.lane_marker_width);
    // Lane lines
    left_far_y[j] = xv.laneLines[0].y;
    left_far_z[j] = xv.laneLines[0].z;
    left_near_y[j] = xv.laneLines[1].y;
    left_near_z[j] = xv.laneLines[1].z;
    right_near_y[j] = xv.laneLines[2].y;
    right_near_z[j] = xv.laneLines[2].z;
    right_far_y[j] = xv.laneLines[3].y;
    right_far_z[j] = xv.laneLines[3].z;
    // Road edges
    left_y[j] = xv.roadEdges[0].y;
    left_z[j] = xv.roadEdges[0].z;
    right_y[j] = xv.roadEdges[1].y;
    right_z[j] = xv.roadEdges[1].z;
  }
  auto lane_lines = nmsg.initLaneLines(4);
  fill_xyzt(lane_lines[0], plan_t, x_idxs_float, left_far_y, left_far_z);
  fill_xyzt(lane_lines[1], plan_t, x_idxs_float, left_near_y, left_near_z);
  fill_xyzt(lane_lines[2], plan_t, x_idxs_float, right_near_y, right_near_z);
  fill_xyzt(lane_lines[3], plan_t, x_idxs_float, right_far_y, right_far_z);
  nmsg.setLaneLineStds({0.0, 0.0, 0.0, 0.0});
  auto s = sigmoid(1.0);
  nmsg.setLaneLineProbs({s, s, s, s});
  // road edges
  auto road_edges = nmsg.initRoadEdges(2);
  fill_xyzt(road_edges[0], plan_t, x_idxs_float, left_y, left_z);
  fill_xyzt(road_edges[1], plan_t, x_idxs_float, right_y, right_z);
  nmsg.setRoadEdgeStds({1.0, 1.0});
}

void dynamicPrintValue(DynamicValue::Reader value, std::ostream& os) {
  // Print an arbitrary message via the dynamic API by
  // iterating over the schema.  Look at the handling
  // of STRUCT in particular. >>> Stole this from the web page

  switch (value.getType()) {
    case DynamicValue::VOID:
      os << "";
      break;
    case DynamicValue::BOOL:
      os << (value.as<bool>() ? "true" : "false");
      break;
    case DynamicValue::INT:
      os << value.as<int64_t>();
      break;
    case DynamicValue::UINT:
      os << value.as<uint64_t>();
      break;
    case DynamicValue::FLOAT:
      os << value.as<double>();
      break;
    case DynamicValue::TEXT:
      os << '\"' << value.as<Text>().cStr() << '\"';
      break;
    case DynamicValue::LIST: {
      os << "[";
      bool first = true;
      for (auto element: value.as<DynamicList>()) {
        if (first) {
          first = false;
        } else {
          os << ", ";
        }
        dynamicPrintValue(element, os);
      }
      os << "]";
      break;
    }
    case DynamicValue::ENUM: {
      auto enumValue = value.as<DynamicEnum>();
      KJ_IF_MAYBE(enumerant, enumValue.getEnumerant()) {
        os <<
            enumerant->getProto().getName().cStr();
      } else {
        // Unknown enum value; output raw number.
        os << enumValue.getRaw();
      }
      break;
    }
    case DynamicValue::STRUCT: {
      os << "(";
      auto structValue = value.as<DynamicStruct>();
      bool first = true;
      for (auto field: structValue.getSchema().getFields()) {
        if (!structValue.has(field)) continue;
        if (first) {
          first = false;
        } else {
          os << ", ";
        }
        os << field.getProto().getName().cStr()
                  << " = ";
        dynamicPrintValue(structValue.get(field), os);
      }
      os << ")";
      break;
    }
    default:
      // There are other types, we aren't handling them.
      os << "*UnknownType*";
      break;
  }
}

void show_message(MessageBuilder &us, MessageBuilder& them) {
  std::ostringstream os;
  os << "***** Them *****\r\n";
  cereal::Event::Reader tr = them.getRoot<cereal::Event>();
  dynamicPrintValue(capnp::toDynamic(tr), os);
  os << "\r\n ***** Us *****\r\n";
  cereal::Event::Reader ur = us.getRoot<cereal::Event>();
  dynamicPrintValue(capnp::toDynamic(ur), os);
  os << "\r\n";
  try {
    if (log_socket) log_socket->send(os.str());
  } catch(...) {
    LOGE("Got exception on show_message");
  }
}

void dorkit(PubMaster& pm, MessageBuilder& omsg_builder, cereal::ModelDataV2::Builder &omsg, const ModelOutput& net_outputs) {
  fill_model(omsg, net_outputs);
  if (!socket_is_active()) {
    // Normal path
    pm.send("modelV2", omsg_builder);
  } else {
    MessageBuilder nmsg_builder;
    cereal::ModelDataV2::Builder nmsg = nmsg_builder.initEvent(true).initModelV2();
    //
    // Clone the fields that are always present
    //
    nmsg.setFrameId(omsg.getFrameId());
    nmsg.setFrameAge(omsg.getFrameAge());
    nmsg.setFrameDropPerc(omsg.getFrameDropPerc());
    nmsg.setTimestampEof(omsg.getTimestampEof());
    nmsg.setModelExecutionTime(omsg.getModelExecutionTime());
    if (omsg.hasRawPredictions()) {
      nmsg.setRawPredictions(omsg.getRawPredictions());
    }
    //
    // Construct the new message
    //
    fill_model(nmsg, net_outputs);
    override_message(nmsg, T_IDXS_FLOAT, X_IDXS_FLOAT);
    if (show_msg) {
      compare_message(omsg, nmsg);
      show_message(nmsg_builder, omsg_builder);
      show_msg = false;
    }
    pm.send("modelV2", nmsg_builder);
  }
}

void carState_listener() {
  SubMaster sm({"carState"});
  while (true) {
    sm.update(100); // 100 milliSeconds between updates
    fake.v = sm["carState"].getCarState().getVEgo();
    fake.a = sm["carState"].getCarState().getAEgo();
  }
}

#if defined(QCOM) || defined(QCOM2)

std::thread socket_listener_thread;
std::thread carState_listener_thread;
std::thread status_ticker_thread;

static void initialize() {
  LOGD("Staring listener thread");
  socket_listener_thread = std::thread(socket_listener);
  carState_listener_thread = std::thread(carState_listener);
  status_ticker_thread = std::thread(status_ticker);
}

// Misnamed, this is the startup for normal operations
void dorkitUnitTest(int argc, char **argv) {
  initialize();
  (void)argc;
  (void)argv;
}

#else
#include <gtest/gtest.h>
//
// Invoked for unit testing.
//
int dorkitUnitTest(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "socket") {
    std::cerr << "Starting socket listener\n";
    (void)socket_listener();
    (void)&status_ticker;
    return 0;
  } else {
    testing::InitGoogleTest(&argc, argv);
    exit(RUN_ALL_TESTS());
  }
}

TEST(moveHeader, zero) {
  Vec2 x(1,1);
  for (auto angle : {0.0, M_PI/2.0, M_PI/4.0}) {
    EXPECT_EQ(moveHeading(x, 0, UnitVector(angle)),x);
  }
  EXPECT_EQ(moveHeading(x, 1.0, UnitVector(0.0)), Vec2(2.0, 1.0));
  EXPECT_EQ(moveHeading(x, 1.0, UnitVector(M_PI/2.0)), Vec2(1.0, 2.0));
  EXPECT_EQ(moveHeading(x, -1.0, UnitVector(M_PI/2.0)), Vec2(1.0, 0.0));
  EXPECT_EQ(moveHeading(Vec2(), 1.0, UnitVector(M_PI_4)), Vec2(M_SQRT2_2, M_SQRT2_2));
}

TEST(BicycleModel, Straight) {
  // Straight
  BicycleModel b(0, 1);
  EXPECT_EQ(b.getTurningCenter().x, -1.0);
  EXPECT_TRUE(isinf(b.getTurningRadius()));
  EXPECT_EQ(b.getPosition(), Vec2( 0.0, 0.0));
  b.setDistance(1);
  EXPECT_EQ(b.getPosition(), Vec2( 1.0, 0.0));
  EXPECT_EQ(b.getWheelPosition( 1.0), Vec2(1.0, 1.0));
  EXPECT_EQ(b.getWheelPosition(-1.0), Vec2(1.0,-1.0));
}

TEST(BicycleModel, Right) {
  BicycleModel b(M_PI_4, 1);  // 45 degree angle
  EXPECT_DOUBLE_EQ(b.getTurningRadius(), M_SQRT2);
  EXPECT_EQ(b.getTurningCenter(), Vec2(-1.0, 1.0));
  EXPECT_EQ(b.getPosition(), Vec2( 0.0, 0.0));
  // Move PI/4 of the way around the circle.
  double circumference = 2 * M_PI * M_SQRT2;
  double distance = circumference / 8; // 2*PI radians in the circle
  b.setDistance(distance);
  EXPECT_DOUBLE_EQ(b.getRotationAngle(), M_PI_4);
  EXPECT_EQ(b.getPosition(), Vec2(b.getTurningRadius() - b.getWheelBase(), b.getTurningCenter().y));
  EXPECT_EQ(b.getWheelPosition(1.0) , b.getPosition() + UnitVector(M_PI_4 + M_PI_2));
  EXPECT_EQ(b.getWheelPosition(-1.0), b.getPosition() + UnitVector(M_PI_4 - M_PI_2));
}

TEST(BicycleModel, Left) {
  BicycleModel b(-M_PI_4, 1);  // 45 degree angle
  EXPECT_DOUBLE_EQ(b.getTurningRadius(), M_SQRT2);
  EXPECT_EQ(b.getTurningCenter(), Vec2(-1.0, -1.0));
  EXPECT_EQ(b.getPosition(), Vec2( 0.0, 0.0));
  // Move PI/4 of the way around the circle.
  double circumference = 2 * M_PI * M_SQRT2;
  double distance = circumference / 8; // 2*PI radians in the circle
  b.setDistance(distance);
  EXPECT_DOUBLE_EQ(b.getRotationAngle(), -M_PI_4);
  EXPECT_EQ(b.getPosition(), Vec2(b.getTurningRadius() - b.getWheelBase(), b.getTurningCenter().y));
  EXPECT_EQ(b.getWheelPosition(1.0) , b.getPosition() + UnitVector(-M_PI_4 + M_PI_2));
  EXPECT_EQ(b.getWheelPosition(-1.0), b.getPosition() + UnitVector(-M_PI_4 - M_PI_2));
}

TEST(T_vars, zero) {
  BicycleModel bike(1);
  bike.setSteeringAngle(0.0);
  bike.setDistance(0);
  T_variables tv(0, 0, 0, bike);
  EXPECT_EQ(tv.position, Vec2());
  EXPECT_EQ(tv.acceleration, Vec2());
  EXPECT_EQ(tv.velocity, Vec2());
  EXPECT_EQ(tv.orientation, RPY());
  EXPECT_EQ(tv.orientationRate, RPY());
}

TEST(T_vars, one) {
  BicycleModel b(1);
  b.setSteeringAngle(0.0);
  T_variables tv(1.0, 1.0, 0, b);   // t = 1.0, v = 1.0, a = 0
  EXPECT_EQ(tv.position, Vec2(1.0, 0));
  EXPECT_EQ(tv.acceleration, Vec2());
  EXPECT_EQ(tv.velocity, Vec2(1.0, 0.0));
  EXPECT_EQ(tv.orientation, RPY());
  EXPECT_EQ(tv.orientationRate, RPY());
}

TEST(T_vars, one_45_right) { // 45 to the right, one unit
  BicycleModel b(1);
  b.setSteeringAngle(M_PI_4);
  EXPECT_EQ(b.getTurningRadius(), M_SQRT2);
  //
  // Compute Time to go PI/8 around the circle
  double circumference = 2 * M_PI * b.getTurningRadius();
  double distance = circumference / 8; // 2*PI radians in the circle
  double v = distance / 1.0;

  T_variables tv(1.0, v, 0, b);   // t = 1.0, v = distance/ 1.0 seconds , a = 0
  EXPECT_EQ(tv.position, Vec2(b.getTurningRadius() - b.getWheelBase(), b.getTurningCenter().y));
  // V**2 / r, r = SQRT(2)
  EXPECT_EQ(tv.velocity, UnitVector(M_PI_4) * v);
  EXPECT_EQ(tv.acceleration, UnitVector(3 * M_PI_4) * (v * v / b.getTurningRadius()));
  EXPECT_EQ(tv.orientation, RPY(0.0, 0.0, M_PI_4));
  EXPECT_EQ(tv.orientationRate, RPY(0.0, 0.0, M_PI_4));
}

TEST(T_vars, two_45_right) { // 45 to the right, two time units
  BicycleModel b(1);
  b.setSteeringAngle(M_PI_4);
  EXPECT_EQ(b.getTurningRadius(), M_SQRT2);
  //
  // Compute Time to go PI/8 around the circle
  double circumference = 2 * M_PI * b.getTurningRadius();
  double distance = circumference / 8; // 2*PI radians in the circle
  double v = distance / 1.0;

  T_variables tv(2.0, v, 0, b);   // t = 2.0, v = distance/ 1.0 seconds , a = 0
  EXPECT_EQ(tv.position, Vec2(0, 2.0));
  // V**2 / r, r = SQRT(2)
  EXPECT_EQ(tv.velocity, UnitVector(M_PI_4 + M_PI_4) * v);
  EXPECT_EQ(tv.acceleration, UnitVector(3 * M_PI_4 + M_PI_4) * (v * v / b.getTurningRadius()));
  EXPECT_EQ(tv.orientation, RPY(0.0, 0.0, M_PI_4 + M_PI_4));
  EXPECT_EQ(tv.orientationRate, RPY(0.0, 0.0, M_PI_4));
}

TEST(T_vars, one_45_left) { // 45 to the right, one unit
  BicycleModel b(1);
  b.setSteeringAngle(-M_PI_4);
  EXPECT_EQ(b.getTurningRadius(), M_SQRT2);
  //
  // Compute Time to go PI/8 around the circle
  double circumference = 2 * M_PI * b.getTurningRadius();
  double distance = circumference / 8; // 2*PI radians in the circle
  double v = distance / 1.0;

  T_variables tv(1.0, v, 0, b);   // t = 1.0, v = distance/ 1.0 seconds , a = 0
  EXPECT_EQ(tv.position, Vec2(b.getTurningRadius() - b.getWheelBase(), b.getTurningCenter().y));
  // V**2 / r, r = SQRT(2)
  EXPECT_EQ(tv.velocity, UnitVector(-M_PI_4) * v);
  EXPECT_EQ(tv.acceleration, UnitVector(3 * -M_PI_4) * (v * v / b.getTurningRadius()));
  EXPECT_EQ(tv.orientation, RPY(0.0, 0.0, -M_PI_4));
  EXPECT_EQ(tv.orientationRate.roll(), 0.0);
  EXPECT_EQ(tv.orientationRate.pitch(), 0.0);
  EXPECT_NEAR(tv.orientationRate.yaw(), -M_PI_4, .01);
}

TEST(intersect, straight_zero) {
  BicycleModel b(1);
  b.setSteeringAngle(0.0);
  //static Vec4 intersect(double x, double v, double a, const BicycleModel& bike, double offset) {
  Vec4 r = X_variables::intersect(0, 1.0, 0.0, b, 0);
  EXPECT_DOUBLE_EQ(r.x, 0.0);
  EXPECT_DOUBLE_EQ(r.y, 0.0);
  EXPECT_DOUBLE_EQ(r.t, 0.0);
}

TEST(intersect, straight_two) {
  BicycleModel b(1);
  b.setSteeringAngle(0.0);
  //static Vec4 intersect(double x, double v, double a, const BicycleModel& bike, double offset) {
  Vec4 r = X_variables::intersect(2.0, 1.0, 0.0, b, 1.0);
  EXPECT_DOUBLE_EQ(r.x, 2.0);
  EXPECT_DOUBLE_EQ(r.y, 1.0);
  EXPECT_DOUBLE_EQ(r.t, 2.0);
}

TEST(intersect, turn_zero) {
  BicycleModel b(1);
  b.setSteeringAngle(M_PI_4);
  //static Vec4 intersect(double x, double v, double a, const BicycleModel& bike, double offset) {
  Vec4 r = X_variables::intersect(0, 1.0, 0.0, b, 0);
  EXPECT_DOUBLE_EQ(r.x, 0.0);
  EXPECT_DOUBLE_EQ(r.y, 0.0);
  EXPECT_DOUBLE_EQ(r.t, 0.0);
}

TEST(intersect, turn_right_two) {
  BicycleModel b(1);
  b.setSteeringAngle(M_PI_4);
  //static Vec4 intersect(double x, double v, double a, const BicycleModel& bike, double offset) {
  Vec4 r = X_variables::intersect(.5, 1.0, 0.0, b, 0.1);
  EXPECT_DOUBLE_EQ(r.x, 0.5);
  EXPECT_DOUBLE_EQ(r.y, 0.79301518781654656);
  EXPECT_DOUBLE_EQ(r.t, 0.5);
}

TEST(intersect, turn_left_two) {
  BicycleModel b(1);
  b.setSteeringAngle(-M_PI_4);
  //static Vec4 intersect(double x, double v, double a, const BicycleModel& bike, double offset) {
  Vec4 r = X_variables::intersect(.5, 1.0, 0.0, b, 0.1);
  EXPECT_DOUBLE_EQ(r.x, 0.5);
  EXPECT_DOUBLE_EQ(r.y, -0.79301518781654656);
  EXPECT_DOUBLE_EQ(r.t, 0.5);
}

TEST(build, with_zeros) {
  ModelOutput *fakeOutput = (ModelOutput *)(new char[sizeof(ModelOutput)]);
  memset(fakeOutput, 0, sizeof(*fakeOutput));
  MessageBuilder nmsg_builder;
  cereal::ModelDataV2::Builder nmsg = nmsg_builder.initEvent(true).initModelV2();
  fill_model(nmsg, *fakeOutput);

  EXPECT_EQ(0, nmsg.getFrameId());
  EXPECT_EQ(0, nmsg.getFrameAge());
}

TEST(build, radius) {
  double r = circleRadiusFromThreePoints(Vec2(-1.0,0.0), Vec2(0.0,1.0), Vec2(1.0, 0.0));
  EXPECT_DOUBLE_EQ(r, 1.0);
}

static void check_xy(cereal::ModelDataV2::XYZTData::Builder xyzt, 
              const std::array<float, TRAJECTORY_SIZE> &t,
              const std::array<float, TRAJECTORY_SIZE> &x, 
              const std::array<float, TRAJECTORY_SIZE> &y,
              size_t size = TRAJECTORY_SIZE) {
  for (size_t i = 0; i < size; ++i) {
    EXPECT_NEAR(t[i], xyzt.getT()[i], .02) << "Failed T @ " << i;
    EXPECT_NEAR(x[i], xyzt.getX()[i], .02) << "Failed X @ " << i;
    EXPECT_NEAR(y[i], xyzt.getY()[i], .02) << "Failed Y @ " << i;
  }
}              

TEST(build, full_msg_straight_ahead) {
  MessageBuilder nmsg_builder;
  cereal::ModelDataV2::Builder nmsg = nmsg_builder.initEvent(true).initModelV2();
  std::array<float, TRAJECTORY_SIZE> t_idxs_float;
  std::array<float, TRAJECTORY_SIZE> x_idxs_float;
  for (size_t i = 0; i < TRAJECTORY_SIZE; ++i) {
    t_idxs_float[i] = float(i);
    x_idxs_float[i] = float(i);
  }
  fake.v = 1.0;
  fake.a = 0.0;
  fake.wheel_base = 1.0;
  fake.current_steering = 0;
  override_message(nmsg, t_idxs_float, x_idxs_float);
  std::array<float, TRAJECTORY_SIZE> pos_x, pos_y, vel_x, vel_y;
  for (size_t i = 0; i < TRAJECTORY_SIZE; ++i) {
    pos_x[i] = i;
    pos_y[i] = 0;
    vel_x[i] = 1.0;
    vel_y[i] = 0.0;
  }
  check_xy(nmsg.getPosition(), t_idxs_float, pos_x, pos_y);
  check_xy(nmsg.getVelocity(), t_idxs_float, vel_x, vel_y);
}

Vec2 PointOnCircle(Vec2 center, float radius, float angle) {
  return Vec2(center.x + (radius * cos(angle)),center.y + (radius * sin(angle)));
}

TEST(build, right_pi_4) {
  MessageBuilder nmsg_builder;
  cereal::ModelDataV2::Builder nmsg = nmsg_builder.initEvent(true).initModelV2();
  std::array<float, TRAJECTORY_SIZE> t_idxs_float;
  std::array<float, TRAJECTORY_SIZE> x_idxs_float;
  for (size_t i = 0; i < TRAJECTORY_SIZE; ++i) {
    t_idxs_float[i] = float(i);
    x_idxs_float[i] = float(i);
  }
  //
  // steer PI/4=>45deg => SQRT(2) turning radius.
  // Thus circumference =2*sqr(2)*PI 
  // To travel 1/4 of a circle in each timestep we set velocity = 
  // circumference / 4
  //
  // Thus we have a turning circle of sqrt(2) radius, with a center of
  // X=-1, Y=1 (-wheel_base, turning_center_y)
  //
  fake.v = (2 * M_SQRT2 * M_PI) / 4.0;
  fake.a = 0.0;
  fake.wheel_base = 1.0;
  fake.current_steering = M_PI_4;
  override_message(nmsg, t_idxs_float, x_idxs_float);
  std::array<float, TRAJECTORY_SIZE> pos_x, pos_y, vel_x, vel_y;
  for (size_t i = 0; i < 4 /*TRAJECTORY_SIZE*/; ++i) {
    vel_x[i] = 1.0;
    vel_y[i] = 0.0;
  }
  pos_x[0] = 0;
  pos_y[0] = 0;
  pos_x[1] = 0;
  pos_y[1] = 2;
  pos_x[2] = -2.0;
  pos_y[2] = 2;
  pos_x[3] = -2.0;
  pos_y[3] = 0;
  pos_x[4] = 0;
  pos_y[4] = 0;

  check_xy(nmsg.getPosition(), t_idxs_float, pos_x, pos_y, 5);
//  check_xy(nmsg.getVelocity(), t_idxs_float, vel_x, vel_y, 2);
}

static void check_correction_table(float requested, float actual, float mph) {
  // std::cerr << "Doing Case: MPH:" << mph << " Actual:" << actual << " Requested:" << requested << " ms:" << mph_to_ms(mph) << "\n";
  EXPECT_NEAR(actual, correct_circle_radius(mph_to_ms(mph), requested), .15*actual);
}

TEST(correction_table, identity) {
  intialize_correction_table();
  check_correction_table(16.1, 35, 6);
  check_correction_table(20.5, 75, 6);
  check_correction_table(20.4, 75, 6);
  check_correction_table(20.2, 75, 6);
  check_correction_table(22.3, 100, 6);
  check_correction_table(27.2, 150, 6);
  check_correction_table(31.6, 200, 6);
  check_correction_table(36.5, 250, 6);
  check_correction_table(42.1, 300, 6);
  check_correction_table(47.4, 350, 6);
  check_correction_table(51.1, 400, 6);
  check_correction_table(61, 500, 6);
  check_correction_table(29.1, 125, 11);
  check_correction_table(53.4, 250, 11);
  check_correction_table(52.8, 250, 11);
  check_correction_table(93, 500, 12);
  check_correction_table(32.8, 125, 14);
  check_correction_table(58.5, 250, 14);
  check_correction_table(102.5, 500, 14);
  check_correction_table(112.9, 500, 14);
  check_correction_table(36.6, 125, 17);
  check_correction_table(74.1, 250, 17);
  check_correction_table(128.9, 500, 17);
}

TEST(correction_table, interpolation_within_same_mph) {
  // Pick two radius and one velocity
  auto low = correct_circle_radius(mph_to_ms(6), 100);
  auto high = correct_circle_radius(mph_to_ms(6), 150);
  auto mid = correct_circle_radius(mph_to_ms(6), 125);
  EXPECT_NEAR(mid, (low+high)/2.0, mid*.05);
}

TEST(correction_table, interpolation_with_same_radius) {
  // Pick one radius and two velocities
  auto low = correct_circle_radius(mph_to_ms(14), 500);
  auto high = correct_circle_radius(mph_to_ms(17), 500);
  auto mid = correct_circle_radius((mph_to_ms(14) + mph_to_ms(17)) * 0.5, 500);
  // std::cerr << "Low:" << low << " High:" << high << " mid:" << mid << "\n";
  EXPECT_NEAR(mid, (low+high)/2.0, mid*.05);

  auto one_quarter = correct_circle_radius(.75*mph_to_ms(14) + .25*mph_to_ms(17), 500);
  EXPECT_NEAR(one_quarter, .75*low+.25*high, mid*.05);
}
#endif
