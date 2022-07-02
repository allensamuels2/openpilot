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
    velocity = bike.getRotationVector() * v_t;
    if (std::isinf(bike.getTurningRadius()) || t == 0.0) {
      acceleration = UnitVector(0.0) * a;
      orientation = RPY(0.0, 0.0, 0.0);
      orientationRate = RPY(0.0, 0.0, 0.0);
    } else {
      // We just account for centripetal acceleration and ignore the linear acceleration component
      // Centripal = V**2 / r
      double acceleration_magnitude = (v_t * v_t) / bike.getTurningRadius();
      acceleration = bike.getAccelerationVector() * acceleration_magnitude;
      orientation = RPY(0.0, 0.0, bike.getRotationAngle());
      orientationRate = RPY(0.0, 0.0, bike.getRotationAngle() / t);
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
  float steering = 0.0;
  float v = 20.0;
  float a = 0.0;
  float roadwidth = 8.0;    // typical road width
  float lane_width = 3.7;   // 12 feet
  float lane_marker_width = .101;  // 4 inches
  float s_incr = .0017;
  float wheel_base = 2.78892;  // Highlander wheel base is 109.8 inches => 2.78892 meters.
} fake;

bool socket_is_active() { return active_connections != 0; }

std::string helpText() {
  std::ostringstream os;
  os << 
    "\r\n"
    "z                           Zero steering deflection\r\n"
    "l                           bump steering to left\r\n"
    "r                           bump steering to right\r\n"
    "si     <degrees>            Set steering bump increment (degrees)\r\n"
    "s      <degrees>            set steering angle\r\n"
    "wb     <meters>             set wheel base\r\n"
    "c      <meters>             set steering for circle of indicated radius\r\n"
    "rw     <meters>             set road width\r\n"
    "lw     <meters>             set lane width\r\n"
    "lmw    <meters>             set lane marker width\r\n"
    "\r\n"
    "  -- Debug Only Commands, not functional in vehicle --\r\n"
    "\r\n"
    "v      <speed>              Set speed\r\n"
    "msg                         show internal DL messages\r\n"
    "help or h                   show this message"
    "\r\n";
  os 
    << ">>>> Current Settings: <<<<\r\n"
    << "Steering          : " << fmt_degrees(fake.steering) << "\r\n"
    << "Steer Increment   : " << fmt_degrees(fake.s_incr)  << "\r\n"
    << "Road Width        : " << fmt_meters(fake.roadwidth) << "\r\n"
    << "Lane Width        : " << fmt_meters(fake.lane_width) << "\r\n"
    << "Lane Marker Width : " << fmt_meters(fake.lane_marker_width) << "\r\n"
    << "Wheel Base        : " << fmt_meters(fake.wheel_base) << "\r\n"
    << "\r\n";
  return os.str();
}

template<typename t>
bool parse_degrees(std::istream& is, t& value) {
  t temp;
  is >> temp;
  if (!is.bad()) {
    value = temp * (M_PI / 180);
    return false;
  } else {
    return true;
  }
}
template<typename t>
bool parse_number(std::istream& is, t& value, t min_value = -std::numeric_limits<t>::infinity(), t max_value = std::numeric_limits<t>::infinity()) {
  t temp;
  is >> temp;
  if (!is.bad() && temp >= min_value && temp <= max_value) {
    value = temp;
    return false;
  } else {
    return true;
  }
}

static void handle_conn(Socket rcv) {
  active_connections++;
  try {
    while (true) {
        LOGE("Waiting for input on connection %s", rcv.format().c_str());
        std::string line = rcv.recv();
        if (line.size() > 0 && line.back() == '\n') line.resize(line.size()-1); // strip trailing newline
        if (line.size() > 0 && line.back() == '\r') line.resize(line.size()-1); // strip trailing return
        LOGE("Got cmd: %s", line.c_str());
        rcv.send(line);
        //
        // Execute command
        //
        std::istringstream is(line);
        std::string cmd;
        is >> cmd;
        bool error = false;
        if (cmd == "" || cmd == "\n" || cmd == "\r\n") {
          ; // end of line?
        } else if (cmd == "q" || cmd == "quit") {
          break;
        } else if (cmd == "v") {
          error = parse_number(is, fake.v, 0.f, 35.7f); // [0..100 MPH]
        } else if (cmd == "z") {
          fake.steering = 0;
        } else if (cmd == "si") {
          error = parse_degrees(is, fake.s_incr);
        } else if (cmd == "s") {
          error = parse_degrees(is, fake.steering);
        } else if (cmd == "wb") {
          error = parse_number(is, fake.wheel_base, 1.0f, 10.0f);
        } else if (cmd == "lmw") {
          error = parse_number(is, fake.lane_marker_width, 0.f, 1.0f);
        } else if (cmd == "lw") {
          error = parse_number(is, fake.lane_width, 2.0f, 10.0f);
        } else if (cmd == "l") {
          fake.steering -= fake.s_incr;
        } else if (cmd == "r") {
          fake.steering += fake.s_incr;
        } else if (cmd == "c") {
          float radius = 0.0;
          error = parse_number(is, radius);
          error |= abs(radius) <= fake.wheel_base;
          if (!error) {
            fake.steering = std::asin(fake.wheel_base / radius);
          }
        } else if (cmd == "msg") {
          log_socket = &rcv;
          show_msg = true;
          while (show_msg) usleep(50000);  // 50mSec
          log_socket = nullptr;
        } else if (cmd == "help" || cmd == "h") {
          rcv.send(helpText());
        } else {
          LOGE("Unknown Command: '%s'\r\n",line.c_str());
          rcv.send("Unknown Commad: " + line + "\r\n");
        }
      if (error) { 
        LOGE("Command in error: %s\r\n", line.c_str());
        rcv.send(helpText());
      }
      std::ostringstream os;
      os  << "Speed:"  << fmt_meters(fake.v) << "/s"
          << " Accel:" << fmt_meters(fake.a) << "/s^2"
          << " Steer:" << fmt_degrees(fake.steering)
          << " Circle:" << fmt_meters(BicycleModel(fake.steering, fake.wheel_base).getTurningRadius())
          << "\r\n";
      LOGE("%s", os.str().c_str());
      rcv.send(os.str());
    }
  } catch(recv_err) {
    LOGE(("Closing socket" + rcv.format()).c_str());
  }
  active_connections--;
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

void override_message(cereal::ModelDataV2::Builder &nmsg) {
  BicycleModel bm(fake.steering, fake.wheel_base);
  //
  // First do the "T" variables
  //
  std::array<float, TRAJECTORY_SIZE> 
      pos_x, pos_y, pos_z, pos_x_std, pos_y_std, pos_z_std,
      vel_x, vel_y, vel_z,
      rot_x, rot_y, rot_z,
      rot_rate_x, rot_rate_y, rot_rate_z;

  for (size_t i = 0; i < TRAJECTORY_SIZE; ++i) {
    T_variables tv(T_IDXS_FLOAT[i], fake.v, fake.a, bm);
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
  fill_xyzt(nmsg.initPosition(), T_IDXS_FLOAT, pos_x, pos_y, pos_z, pos_x_std, pos_y_std, pos_z_std);
  fill_xyzt(nmsg.initVelocity(), T_IDXS_FLOAT, vel_x, vel_y, vel_z);
  fill_xyzt(nmsg.initOrientation(), T_IDXS_FLOAT, rot_x, rot_y, rot_z);
  fill_xyzt(nmsg.initOrientationRate(), T_IDXS_FLOAT, rot_rate_x, rot_rate_y, rot_rate_z);

  // lane lines
  std::array<float, TRAJECTORY_SIZE> left_far_y, left_far_z;
  std::array<float, TRAJECTORY_SIZE> left_near_y, left_near_z;
  std::array<float, TRAJECTORY_SIZE> right_near_y, right_near_z;
  std::array<float, TRAJECTORY_SIZE> right_far_y, right_far_z;
  // road edges
  std::array<float, TRAJECTORY_SIZE> left_y, left_z;
  std::array<float, TRAJECTORY_SIZE> right_y, right_z;
  for (size_t j = 0; j < TRAJECTORY_SIZE; ++j) {
    X_variables xv(X_IDXS_FLOAT[j], fake.v, fake.a, bm, fake.roadwidth, fake.lane_width, fake.lane_marker_width);
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
  fill_xyzt(lane_lines[0], T_IDXS_FLOAT, X_IDXS_FLOAT, left_far_y, left_far_z);
  fill_xyzt(lane_lines[1], T_IDXS_FLOAT, X_IDXS_FLOAT, left_near_y, left_near_z);
  fill_xyzt(lane_lines[2], T_IDXS_FLOAT, X_IDXS_FLOAT, right_near_y, right_near_z);
  fill_xyzt(lane_lines[3], T_IDXS_FLOAT, X_IDXS_FLOAT, right_far_y, right_far_z);
  nmsg.setLaneLineStds({0.0, 0.0, 0.0, 0.0});
  auto s = sigmoid(1.0);
  nmsg.setLaneLineProbs({s, s, s, s});
  // road edges
  auto road_edges = nmsg.initRoadEdges(2);
  fill_xyzt(road_edges[0], T_IDXS_FLOAT, X_IDXS_FLOAT, left_y, left_z);
  fill_xyzt(road_edges[1], T_IDXS_FLOAT, X_IDXS_FLOAT, right_y, right_z);
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
    override_message(nmsg);
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

static void initialize() {
  LOGD("Staring listener thread");
  socket_listener_thread = std::thread(socket_listener);
  carState_listener_thread = std::thread(carState_listener);
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
  EXPECT_EQ(tv.orientationRate, RPY(0.0, 0.0, -M_PI_4));
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

#endif
