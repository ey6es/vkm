#ifndef VKM_TEST_H
#define VKM_TEST_H

#include <sstream>
#include <catch2/catch.hpp>

#include "math.hpp"
#include "geom.hpp"

namespace vkm {

inline Catch::Matchers::Floating::WithinAbsMatcher ApproxEquals (float target) {
  return Catch::Matchers::WithinAbs(target, 0.001f);
}

inline Catch::Matchers::Floating::WithinAbsMatcher ApproxEquals (double target) {
  return Catch::Matchers::WithinAbs(target, 0.00001);
}

template<typename T>
class ApproxEqualsMatcherBase : public Catch::Matchers::Impl::MatcherBase<T> {
public:

  ApproxEqualsMatcherBase (const T& value) : value_(value) {}

  std::string describe () const override {
    std::ostringstream ss;
    ss << "is approximately " << value_;
    return ss.str();
  }

protected:

  T value_;
};

template<typename T>
class Vec2ApproxEqualsMatcher : public ApproxEqualsMatcherBase<vec2<T>> {
  using ApproxEqualsMatcherBase<vec2<T>>::value_;

public:

  Vec2ApproxEqualsMatcher (const vec2<T>& value) : ApproxEqualsMatcherBase<vec2<T>>(value) {}

  bool match (const vec2<T>& value) const override {
    return ApproxEquals(value_.x).match(value.x) && ApproxEquals(value_.y).match(value.y);
  }
};

template<typename T>
Vec2ApproxEqualsMatcher<T> ApproxEquals (const vec2<T>& vec) {
  return {vec};
}

template<typename T>
class Vec3ApproxEqualsMatcher : public ApproxEqualsMatcherBase<vec3<T>> {
  using ApproxEqualsMatcherBase<vec3<T>>::value_;

public:

  Vec3ApproxEqualsMatcher (const vec3<T>& value) : ApproxEqualsMatcherBase<vec3<T>>(value) {}

  bool match (const vec3<T>& value) const override {
    return ApproxEquals(value_.x).match(value.x) && ApproxEquals(value_.y).match(value.y) && ApproxEquals(value_.z).match(value.z);
  }
};

template<typename T>
Vec3ApproxEqualsMatcher<T> ApproxEquals (const vec3<T>& vec) {
  return {vec};
}

template<typename T>
class Vec4ApproxEqualsMatcher : public ApproxEqualsMatcherBase<vec4<T>> {
  using ApproxEqualsMatcherBase<vec4<T>>::value_;

public:

  Vec4ApproxEqualsMatcher (const vec4<T>& value) : ApproxEqualsMatcherBase<vec4<T>>(value) {}

  bool match (const vec4<T>& value) const override {
    return ApproxEquals(value_.x).match(value.x) && ApproxEquals(value_.y).match(value.y) &&
      ApproxEquals(value_.z).match(value.z) && ApproxEquals(value_.w).match(value.w);
  }
};

template<typename T>
Vec4ApproxEqualsMatcher<T> ApproxEquals (const vec4<T>& vec) {
  return {vec};
}

template<typename T>
class EulerApproxEqualsMatcher : public ApproxEqualsMatcherBase<euler<T>> {
  using ApproxEqualsMatcherBase<euler<T>>::value_;

public:

  EulerApproxEqualsMatcher (const euler<T>& value) : ApproxEqualsMatcherBase<euler<T>>(value) {}

  bool match (const euler<T>& value) const override {
    return ApproxEquals(value_.x).match(value.x) && ApproxEquals(value_.y).match(value.y) && ApproxEquals(value_.z).match(value.z);
  }
};

template<typename T>
EulerApproxEqualsMatcher<T> ApproxEquals (const euler<T>& euler) {
  return {euler};
}

template<typename T>
class QuatApproxEqualsMatcher : public ApproxEqualsMatcherBase<quat<T>> {
  using ApproxEqualsMatcherBase<quat<T>>::value_;

public:

  QuatApproxEqualsMatcher (const quat<T>& value) : ApproxEqualsMatcherBase<quat<T>>(value) {}

  bool match (const quat<T>& value) const override {
    return ApproxEquals(value_.x).match(value.x) && ApproxEquals(value_.y).match(value.y) &&
      ApproxEquals(value_.z).match(value.z) && ApproxEquals(value_.w).match(value.w);
  }
};

template<typename T>
QuatApproxEqualsMatcher<T> ApproxEquals (const quat<T>& quat) {
  return {quat};
}

template<typename T>
class Mat2ApproxEqualsMatcher : public ApproxEqualsMatcherBase<mat2<T>> {
  using ApproxEqualsMatcherBase<mat2<T>>::value_;

public:

  Mat2ApproxEqualsMatcher (const mat2<T>& value) : ApproxEqualsMatcherBase<mat2<T>>(value) {}

  bool match (const mat2<T>& value) const override {
    return ApproxEquals(value_.c0).match(value.c0) && ApproxEquals(value_.c1).match(value.c1);
  }
};

template<typename T>
Mat2ApproxEqualsMatcher<T> ApproxEquals (const mat2<T>& mat) {
  return {mat};
}

template<typename T>
class Mat3ApproxEqualsMatcher : public ApproxEqualsMatcherBase<mat3<T>> {
  using ApproxEqualsMatcherBase<mat3<T>>::value_;

public:

  Mat3ApproxEqualsMatcher (const mat3<T>& value) : ApproxEqualsMatcherBase<mat3<T>>(value) {}

  bool match (const mat3<T>& value) const override {
    return ApproxEquals(value_.c0).match(value.c0) && ApproxEquals(value_.c1).match(value.c1) && ApproxEquals(value_.c2).match(value.c2);
  }
};

template<typename T>
Mat3ApproxEqualsMatcher<T> ApproxEquals (const mat3<T>& mat) {
  return {mat};
}

template<typename T>
class Mat4x3ApproxEqualsMatcher : public ApproxEqualsMatcherBase<mat4x3<T>> {
  using ApproxEqualsMatcherBase<mat4x3<T>>::value_;

public:

  Mat4x3ApproxEqualsMatcher (const mat4x3<T>& value) : ApproxEqualsMatcherBase<mat4x3<T>>(value) {}

  bool match (const mat4x3<T>& value) const override {
    return ApproxEquals(value_.c0).match(value.c0) && ApproxEquals(value_.c1).match(value.c1) &&
      ApproxEquals(value_.c2).match(value.c2) && ApproxEquals(value_.c3).match(value.c3);
  }
};

template<typename T>
Mat4x3ApproxEqualsMatcher<T> ApproxEquals (const mat4x3<T>& mat) {
  return {mat};
}

template<typename T>
class Mat4ApproxEqualsMatcher : public ApproxEqualsMatcherBase<mat4<T>> {
  using ApproxEqualsMatcherBase<mat4<T>>::value_;

public:

  Mat4ApproxEqualsMatcher (const mat4<T>& value) : ApproxEqualsMatcherBase<mat4<T>>(value) {}

  bool match (const mat4<T>& value) const override {
    return ApproxEquals(value_.c0).match(value.c0) && ApproxEquals(value_.c1).match(value.c1) &&
      ApproxEquals(value_.c2).match(value.c2) && ApproxEquals(value_.c3).match(value.c3);
  }
};

template<typename T>
Mat4ApproxEqualsMatcher<T> ApproxEquals (const mat4<T>& mat) {
  return {mat};
}

template<typename T>
class TransformApproxEqualsMatcher : public ApproxEqualsMatcherBase<transform<T>> {
  using ApproxEqualsMatcherBase<transform<T>>::value_;

public:

  TransformApproxEqualsMatcher (const transform<T>& value) : ApproxEqualsMatcherBase<transform<T>>(value) {}

  bool match (const transform<T>& value) const override {
    return ApproxEquals(mat4<T>(value_)).match(mat4<T>(value));
  }
};

template<typename T>
TransformApproxEqualsMatcher<T> ApproxEquals (const transform<T>& mat) {
  return {mat};
}

template<typename T>
class BoxApproxEqualsMatcher : public ApproxEqualsMatcherBase<box<T>> {
  using ApproxEqualsMatcherBase<box<T>>::value_;

public:

  BoxApproxEqualsMatcher (const box<T>& value) : ApproxEqualsMatcherBase<box<T>>(value) {}

  bool match (const box<T>& value) const override {
    return ApproxEquals(value_.min).match(value.min) && ApproxEquals(value_.max).match(value.max);
  }
};

template<typename T>
BoxApproxEqualsMatcher<T> ApproxEquals (const box<T>& box) {
  return {box};
}

template<typename T>
class CuboidApproxEqualsMatcher : public ApproxEqualsMatcherBase<cuboid<T>> {
  using ApproxEqualsMatcherBase<cuboid<T>>::value_;

public:

  CuboidApproxEqualsMatcher (const cuboid<T>& value) : ApproxEqualsMatcherBase<cuboid<T>>(value) {}

  bool match (const cuboid<T>& value) const override {
    return ApproxEquals(value_.matrix).match(value.matrix);
  }
};

template<typename T>
CuboidApproxEqualsMatcher<T> ApproxEquals (const cuboid<T>& cuboid) {
  return {cuboid};
}

template<typename T>
class PlaneApproxEqualsMatcher : public ApproxEqualsMatcherBase<plane<T>> {
  using ApproxEqualsMatcherBase<plane<T>>::value_;

public:

  PlaneApproxEqualsMatcher (const plane<T>& value) : ApproxEqualsMatcherBase<plane<T>>(value) {}

  bool match (const plane<T>& value) const override {
    return ApproxEquals(value_.a).match(value.a) && ApproxEquals(value_.b).match(value.b) &&
      ApproxEquals(value_.c).match(value.c) && ApproxEquals(value_.d).match(value.d);
  }
};

template<typename T>
PlaneApproxEqualsMatcher<T> ApproxEquals (const plane<T>& plane) {
  return {plane};
}

template<typename T>
class TriangleApproxEqualsMatcher : public ApproxEqualsMatcherBase<triangle<T>> {
  using ApproxEqualsMatcherBase<triangle<T>>::value_;

public:

  TriangleApproxEqualsMatcher (const triangle<T>& value) : ApproxEqualsMatcherBase<triangle<T>>(value) {}

  bool match (const triangle<T>& value) const override {
    return ApproxEquals(value_.v0).match(value.v0) && ApproxEquals(value_.v1).match(value.v1) &&
      ApproxEquals(value_.v2).match(value.v2);
  }
};

template<typename T>
TriangleApproxEqualsMatcher<T> ApproxEquals (const triangle<T>& triangle) {
  return {triangle};
}

template<typename T>
class SphereApproxEqualsMatcher : public ApproxEqualsMatcherBase<sphere<T>> {
  using ApproxEqualsMatcherBase<sphere<T>>::value_;

public:

  SphereApproxEqualsMatcher (const sphere<T>& value) : ApproxEqualsMatcherBase<sphere<T>>(value) {}

  bool match (const sphere<T>& value) const override {
    return ApproxEquals(value_.center).match(value.center) && ApproxEquals(value_.radius).match(value.radius);
  }
};

template<typename T>
SphereApproxEqualsMatcher<T> ApproxEquals (const sphere<T>& sphere) {
  return {sphere};
}

template<typename T>
class CapsuleApproxEqualsMatcher : public ApproxEqualsMatcherBase<capsule<T>> {
  using ApproxEqualsMatcherBase<capsule<T>>::value_;

public:

  CapsuleApproxEqualsMatcher (const capsule<T>& value) : ApproxEqualsMatcherBase<capsule<T>>(value) {}

  bool match (const capsule<T>& value) const override {
    return ApproxEquals(value_.start).match(value.start) && ApproxEquals(value_.end).match(value.end) &&
      ApproxEquals(value_.radius).match(value.radius);
  }
};

template<typename T>
CapsuleApproxEqualsMatcher<T> ApproxEquals (const capsule<T>& capsule) {
  return {capsule};
}

template<typename T>
class RayApproxEqualsMatcher : public ApproxEqualsMatcherBase<ray<T>> {
  using ApproxEqualsMatcherBase<ray<T>>::value_;

public:

  RayApproxEqualsMatcher (const ray<T>& value) : ApproxEqualsMatcherBase<ray<T>>(value) {}

  bool match (const ray<T>& value) const override {
    return ApproxEquals(value_.origin).match(value.origin) && ApproxEquals(value_.dir).match(value.dir);
  }
};

template<typename T>
RayApproxEqualsMatcher<T> ApproxEquals (const ray<T>& ray) {
  return {ray};
}

}

#endif // VKM_TEST_H
