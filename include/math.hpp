#ifndef VKM_MATH_H
#define VKM_MATH_H

#include <algorithm>
#include <cmath>
#include <ostream>

namespace vkm {

inline float radians (float degrees) { return degrees * float(M_PI) / 180; }
inline double radians (double degrees) { return degrees * M_PI / 180; }

inline float degrees (float radians) { return radians * 180 / float(M_PI); }
inline double degrees (double radians) { return radians * 180 / M_PI; }

inline float mix (float x, float y, float a) { return x + (y - x) * a; }
inline double mix (double x, double y, double a) { return x + (y - x) * a; }

template<typename T>
inline bool is_within (T value, T min, T max) {
  return value >= min && value <= max;
}

template<typename T>
struct vec2 {
  T x, y;

  explicit vec2 (T v = 0) : x(v), y(v) {}
  vec2 (T x, T y) : x(x), y(y) {}

  template<typename S>
  vec2 (const vec2<S>& other) : x(other.x), y(other.y) {}

  T& operator[] (size_t index) {
    return reinterpret_cast<T*>(this)[index];
  }

  T operator[] (size_t index) const {
    return reinterpret_cast<const T*>(this)[index];
  }

  bool operator== (const vec2& other) const {
    return x == other.x && y == other.y;
  }

  bool operator!= (const vec2& other) const {
    return x != other.x || y != other.y;
  }

  vec2 operator+ (const vec2& other) const {
    return vec2(x + other.x, y + other.y);
  }

  vec2 operator- (const vec2& other) const {
    return vec2(x - other.x, y - other.y);
  }

  vec2 operator+ () const {
    return vec2(+x, +y);
  }

  vec2 operator- () const {
    return vec2(-x, -y);
  }

  vec2 operator* (T scalar) const {
    return vec2(x * scalar, y * scalar);
  }

  vec2 operator* (const vec2& other) const {
    return vec2(x * other.x, y * other.y);
  }

  vec2 operator/ (T scalar) const {
    return *this * (1 / scalar);
  }

  vec2 operator/ (const vec2& other) const {
    return vec2(x / other.x, y / other.y);
  }

  vec2& operator+= (const vec2& other) {
    x += other.x;
    y += other.y;
    return *this;
  }

  vec2& operator-= (const vec2& other) {
    x -= other.x;
    y -= other.y;
    return *this;
  }

  vec2& operator*= (T scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
  }

  vec2& operator*= (const vec2& other) {
    x *= other.x;
    y *= other.y;
    return *this;
  }

  vec2& operator/= (T scalar) {
    return *this *= (1 / scalar);
  }

  vec2& operator/= (const vec2& other) {
    x /= other.x;
    y /= other.y;
    return *this;
  }
};

template<typename S, typename T>
inline vec2<T> operator* (S scalar, const vec2<T>& vector) {
  return vec2<T>(scalar * vector.x, scalar * vector.y);
}

template<typename T>
inline T dot (const vec2<T>& a, const vec2<T>& b) {
  return a.x*b.x + a.y*b.y;
}

template<typename T>
inline T length (const vec2<T>& a) {
  return std::sqrt(dot(a, a));
}

template<typename T>
inline T distance (const vec2<T>& a, const vec2<T>& b) {
  return length(a - b);
}

template<typename T>
inline vec2<T> normalize (const vec2<T>& a) {
  return a / length(a);
}

template<typename T>
inline vec2<T> mix (const vec2<T>& x, const vec2<T>& y, T a) {
  return x + (y - x) * a;
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const vec2<T>& vector) {
  return out << '(' << vector.x << ", " << vector.y << ')';
}

typedef vec2<float> vec2f;
typedef vec2<double> vec2d;

template<typename T>
struct vec3 {
  T x, y, z;

  explicit vec3 (T v = 0) : x(v), y(v), z(v) {}
  vec3 (T x, T y, T z) : x(x), y(y), z(z) {}
  vec3 (const vec2<T>& v, T z) : x(v.x), y(v.y), z(z) {}

  template<typename S>
  explicit vec3 (const vec3<S>& other) : x(other.x), y(other.y), z(other.z) {}

  explicit operator vec2<T> () const {
    return vec2<T>(x, y);
  }

  T& operator[] (size_t index) {
    return reinterpret_cast<T*>(this)[index];
  }

  T operator[] (size_t index) const {
    return reinterpret_cast<const T*>(this)[index];
  }

  bool operator== (const vec3& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator!= (const vec3& other) const {
    return x != other.x || y != other.y || z != other.z;
  }

  vec3 operator+ (const vec3& other) const {
    return vec3(x + other.x, y + other.y, z + other.z);
  }

  vec3 operator- (const vec3& other) const {
    return vec3(x - other.x, y - other.y, z - other.z);
  }

  vec3 operator+ () const {
    return vec3(+x, +y, +z);
  }

  vec3 operator- () const {
    return vec3(-x, -y, -z);
  }

  vec3 operator* (T scalar) const {
    return vec3(x * scalar, y * scalar, z * scalar);
  }

  vec3 operator* (const vec3& other) const {
    return vec3(x * other.x, y * other.y, z * other.z);
  }

  vec3 operator/ (T scalar) const {
    return *this * (1 / scalar);
  }

  vec3 operator/ (const vec3& other) const {
    return vec3(x / other.x, y / other.y, z / other.z);
  }

  vec3& operator+= (const vec3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  vec3& operator-= (const vec3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  vec3& operator*= (T scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  vec3& operator*= (const vec3& other) {
    x *= other.x;
    y *= other.y;
    z *= other.z;
    return *this;
  }

  vec3& operator/= (T scalar) {
    return *this *= (1 / scalar);
  }

  vec3& operator/= (const vec3& other) {
    x /= other.x;
    y /= other.y;
    z /= other.z;
    return *this;
  }
};

template<typename S, typename T>
inline vec3<T> operator* (S scalar, const vec3<T>& vector) {
  return vec3<T>(scalar * vector.x, scalar * vector.y, scalar * vector.z);
}

template<typename T>
inline T dot (const vec3<T>& a, const vec3<T>& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

template<typename T>
inline T length (const vec3<T>& a) {
  return std::sqrt(dot(a, a));
}

template<typename T>
inline T distance (const vec3<T>& a, const vec3<T>& b) {
  return length(a - b);
}

template<typename T>
inline vec3<T> normalize (const vec3<T>& a) {
  return a / length(a);
}

template<typename T>
inline vec3<T> mix (const vec3<T>& x, const vec3<T>& y, T a) {
  return x + (y - x) * a;
}

template<typename T>
inline vec3<T> cross (const vec3<T>& a, const vec3<T>& b) {
  return vec3<T>(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

template<typename T>
inline vec3<T> min (const vec3<T>& a, const vec3<T>& b) {
  return vec3<T>(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
}

template<typename T>
inline vec3<T> max (const vec3<T>& a, const vec3<T>& b) {
  return vec3<T>(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const vec3<T>& vector) {
  return out << '(' << vector.x << ", " << vector.y << ", " << vector.z << ')';
}

typedef vec3<float> vec3f;
typedef vec3<double> vec3d;

template<typename T>
struct vec4 {
  T x, y, z, w;

  explicit vec4 (T v = 0) : x(v), y(v), z(v), w(v) {}
  vec4 (T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
  vec4 (const vec2<T>& v, T z, T w) : x(v.x), y(v.y), z(z), w(w) {}
  vec4 (const vec3<T>& v, T w) : x(v.x), y(v.y), z(v.z), w(w) {}

  template<typename S>
  explicit vec4 (const vec4<S>& other) : x(other.x), y(other.y), z(other.z), w(other.w) {}

  explicit operator vec3<T> () const {
    return vec3<T>(x, y, z);
  }

  T& operator[] (size_t index) {
    return reinterpret_cast<T*>(this)[index];
  }

  T operator[] (size_t index) const {
    return reinterpret_cast<const T*>(this)[index];
  }

  bool operator== (const vec4& other) const {
    return x == other.x && y == other.y && z == other.z && w == other.w;
  }

  bool operator!= (const vec4& other) const {
    return x != other.x || y != other.y || z != other.z || w != other.w;
  }

  vec4 operator+ (const vec4& other) const {
    return vec4(x + other.x, y + other.y, z + other.z, w + other.w);
  }

  vec4 operator- (const vec4& other) const {
    return vec4(x - other.x, y - other.y, z - other.z, w - other.w);
  }

  vec4 operator+ () const {
    return vec4(+x, +y, +z, +w);
  }

  vec4 operator- () const {
    return vec4(-x, -y, -z, -w);
  }

  vec4 operator* (T scalar) const {
    return vec4(x * scalar, y * scalar, z * scalar, w * scalar);
  }

  vec4 operator* (const vec4& other) const {
    return vec4(x * other.x, y * other.y, z * other.z, w * other.w);
  }

  vec4 operator/ (T scalar) const {
    return *this * (1 / scalar);
  }

  vec4 operator/ (const vec4& other) const {
    return vec4(x / other.x, y / other.y, z / other.z, w / other.w);
  }

  vec4& operator+= (const vec4& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    w += other.w;
    return *this;
  }

  vec4& operator-= (const vec4& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    w -= other.w;
    return *this;
  }

  vec4& operator*= (T scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    w *= scalar;
    return *this;
  }

  vec4& operator*= (const vec4& other) {
    x *= other.x;
    y *= other.y;
    z *= other.z;
    w *= other.w;
    return *this;
  }

  vec4& operator/= (T scalar) {
    return *this *= (1 / scalar);
  }

  vec4& operator/= (const vec4& other) {
    x /= other.x;
    y /= other.y;
    z /= other.z;
    w /= other.w;
    return *this;
  }
};

template<typename S, typename T>
vec4<T> operator* (S scalar, const vec4<T>& vector) {
  return vec4<T>(scalar * vector.x, scalar * vector.y, scalar * vector.z, scalar * vector.w);
}

template<typename T>
T dot (const vec4<T>& a, const vec4<T>& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}

template<typename T>
T length (const vec4<T>& a) {
  return std::sqrt(dot(a, a));
}

template<typename T>
T distance (const vec4<T>& a, const vec4<T>& b) {
  return length(a - b);
}

template<typename T>
vec4<T> normalize (const vec4<T>& a) {
  return a / length(a);
}

template<typename T>
inline vec4<T> mix (const vec4<T>& x, const vec4<T>& y, T a) {
  return x + (y - x) * a;
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const vec4<T>& vector) {
  return out << '(' << vector.x << ", " << vector.y << ", " << vector.z << ", " << vector.w << ')';
}

typedef vec4<float> vec4f;
typedef vec4<double> vec4d;

template<typename T>
struct euler {
  T x, y, z;

  euler () : x(0), y(0), z(0) {}
  euler (T x, T y, T z) : x(x), y(y), z(z) {}

  template<typename S>
  explicit euler (const euler<S>& other) : x(other.x), y(other.y), z(other.z) {}

  T& operator[] (size_t index) {
    return reinterpret_cast<T*>(this)[index];
  }

  T operator[] (size_t index) const {
    return reinterpret_cast<const T*>(this)[index];
  }

  bool operator== (const euler& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator!= (const euler& other) const {
    return x != other.x || y != other.y || z != other.z;
  }

  euler operator+ (const euler& other) const {
    return euler(x + other.x, y + other.y, z + other.z);
  }

  euler operator- (const euler& other) const {
    return euler(x - other.x, y - other.y, z - other.z);
  }

  euler operator+ () const {
    return euler(+x, +y, +z);
  }

  euler operator- () const {
    return euler(-x, -y, -z);
  }

  euler operator* (T scalar) const {
    return euler(x * scalar, y * scalar, z * scalar);
  }

  euler operator* (const euler& other) const {
    return euler(x * other.x, y * other.y, z * other.z);
  }

  euler operator/ (T scalar) const {
    return *this * (1 / scalar);
  }

  euler operator/ (const euler& other) const {
    return euler(x / other.x, y / other.y, z / other.z);
  }

  euler& operator+= (const euler& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  euler& operator-= (const euler& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  euler& operator*= (T scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  euler& operator*= (const euler& other) {
    x *= other.x;
    y *= other.y;
    z *= other.z;
    return *this;
  }

  euler& operator/= (T scalar) {
    return *this *= (1 / scalar);
  }

  euler& operator/= (const euler& other) {
    x /= other.x;
    y /= other.y;
    z /= other.z;
    return *this;
  }

  vec3<T> operator* (const vec3<T>& vec) const;
};

template<typename S, typename T>
inline euler<T> operator* (S scalar, const euler<T>& euler) {
  return vec3<T>(scalar * euler.x, scalar * euler.y, scalar * euler.z);
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const euler<T>& euler) {
  return out << '(' << euler.x << ", " << euler.y << ", " << euler.z << ')';
}

typedef euler<float> eulerf;
typedef euler<double> eulerd;

template<typename T>
struct quat {
  T x, y, z, w;

  quat () : x(0), y(0), z(0), w(1) {}
  quat (T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
  quat (T angle, const vec3<T>& axis);
  explicit quat (const euler<T>& euler);

  template<typename S>
  explicit quat (const quat<S>& other) : x(other.x), y(other.y), z(other.z), w(other.w) {}

  explicit operator euler<T> () const;

  T& operator[] (size_t index) {
    return reinterpret_cast<T*>(this)[index];
  }

  T operator[] (size_t index) const {
    return reinterpret_cast<const T*>(this)[index];
  }

  bool operator== (const quat& other) const {
    return x == other.x && y == other.y && z == other.z && w == other.w;
  }

  bool operator!= (const quat& other) const {
    return x != other.x || y != other.y || z != other.z || w != other.w;
  }

  quat operator+ (const quat& other) const {
    return quat(x + other.x, y + other.y, z + other.z, w + other.w);
  }

  quat operator- (const quat& other) const {
    return quat(x - other.x, y - other.y, z - other.z, w - other.w);
  }

  quat operator+ () const {
    return quat(+x, +y, +z, +w);
  }

  quat operator- () const {
    return quat(-x, -y, -z, -w);
  }

  quat operator* (T s) const {
    return quat(x * s, y * s, z * s, w * s);
  }

  quat operator/ (T scalar) const {
    return *this * (1 / scalar);
  }

  quat& operator+= (const quat& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    w += other.w;
    return *this;
  }

  quat& operator-= (const quat& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    w -= other.w;
    return *this;
  }

  quat& operator*= (T scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    w *= scalar;
    return *this;
  }

  quat& operator/= (T scalar) {
    return *this *= (1 / scalar);
  }

  quat operator* (const quat& other) const {
    // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
    return quat(
      w*other.x + x*other.w + y*other.z - z*other.y,
      w*other.y + y*other.w + z*other.x - x*other.z,
      w*other.z + z*other.w + x*other.y - y*other.x,
      w*other.w - x*other.x - y*other.y - z*other.z);
  }

  vec3<T> operator* (const vec3<T>& vec) const;
};

template<typename S, typename T>
inline quat<T> operator* (S scalar, const quat<T>& a) {
  return quat<T>(scalar * a.x, scalar * a.y, scalar * a.z, scalar * a.w);
}

template<typename T>
inline T dot (const quat<T>& a, const quat<T>& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}

template<typename T>
inline T length (const quat<T>& a) {
  return std::sqrt(dot(a, a));
}

template<typename T>
inline quat<T> normalize (const quat<T>& a) {
  T scale = 1 / length(a);
  return quat<T>(a.x * scale, a.y * scale, a.z * scale, a.w * scale);
}

template<typename T>
inline quat<T> inverse (const quat<T>& a) {
  return quat<T>(-a.x, -a.y, -a.z, a.w);
}

template<typename T>
quat<T> mix (const quat<T>& x, const quat<T>& y, T a);

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const quat<T>& quat) {
  return out << '(' << quat.x << ", " << quat.y << ", " << quat.z << ", " << quat.w << ')';
}

typedef quat<float> quatf;
typedef quat<double> quatd;

template<typename T>
struct mat2 {
  vec2<T> c0, c1;

  static mat2 identity () {
    return mat2(1);
  }

  explicit mat2 (T v = 0) : c0(v, 0), c1(0, v) {}
  mat2 (const vec2<T>& c0, const vec2<T>& c1) : c0(c0), c1(c1) {}
  mat2 (T m00, T m01, T m10, T m11) : c0(m00, m01), c1(m10, m11) {}

  vec2<T>& operator[] (size_t index) {
    return reinterpret_cast<vec2<T>*>(this)[index];
  }

  const vec2<T>& operator[] (size_t index) const {
    return reinterpret_cast<const vec2<T>*>(this)[index];
  }

  bool operator== (const mat2& other) const {
    return c0 == other.c0 && c1 == other.c1;
  }

  bool operator!= (const mat2& other) const {
    return c0 != other.c0 || c1 != other.c1;
  }

  mat2 operator+ (const mat2& other) const {
    return mat2(c0 + other.c0, c1 + other.c1);
  }

  mat2 operator- (const mat2& other) const {
    return mat2(c0 - other.c0, c1 - other.c1);
  }

  mat2 operator+ () const {
    return mat2(+c0, +c1);
  }

  mat2 operator- () const {
    return mat2(-c0, -c1);
  }

  mat2 operator* (T scalar) const {
    return mat2(c0 * scalar, c1 * scalar);
  }

  vec2<T> operator* (const vec2<T>& v) const {
    return vec2<T>(c0.x*v.x + c1.x*v.y, c0.y*v.x + c1.y*v.y);
  }

  mat2 operator* (const mat2& o) const {
    return mat2(
      c0.x*o.c0.x + c1.x*o.c0.y,
      c0.y*o.c0.x + c1.y*o.c0.y,
      c0.x*o.c1.x + c1.x*o.c1.y,
      c0.y*o.c1.x + c1.y*o.c1.y);
  }

  mat2 operator/ (T scalar) const {
    return *this * (1 / scalar);
  }

  mat2& operator+= (const mat2& other) {
    c0 += other.c0;
    c1 += other.c1;
    return *this;
  }

  mat2& operator-= (const mat2& other) {
    c0 -= other.c0;
    c1 -= other.c1;
    return *this;
  }

  mat2& operator*= (T scalar) {
    c0 *= scalar;
    c1 *= scalar;
    return *this;
  }

  mat2& operator/= (T scalar) {
    return *this *= (1 / scalar);
  }
};

template<typename S, typename T>
inline mat2<T> operator* (S scalar, const mat2<T>& matrix) {
  return mat2<T>(scalar * matrix.c0, scalar * matrix.c1);
}

template<typename T>
inline vec2<T> operator* (const vec2<T>& v, const mat2<T>& m) {
  return vec2<T>(dot(v, m.c0), dot(v, m.c1));
}

template<typename T>
inline mat2<T> transpose (const mat2<T>& m) {
  return mat2<T>(m.c0.x, m.c1.x, m.c0.y, m.c1.y);
}

template<typename T>
inline T determinant (const mat2<T>& m) {
  return m.c0.x*m.c1.y - m.c1.x*m.c0.y;
}

template<typename T>
inline mat2<T> inverse (const mat2<T>& m) {
  // https://en.wikipedia.org/wiki/Invertible_matrix#Inversion_of_2_%C3%97_2_matrices
  T s = 1 / determinant(m);
  return mat2<T>(m.c1.y * s, -m.c0.y * s, -m.c1.x * s, m.c0.x * s);
}

template<typename T>
inline mat2<T> mix (const mat2<T>& x, const mat2<T>& y, T a) {
  return x + (y - x) * a;
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const mat2<T>& matrix) {
  return out << '(' << matrix.c0 << ", " << matrix.c1 << ')';
}

typedef mat2<float> mat2f;
typedef mat2<double> mat2d;

template<typename T>
struct mat3 {
  vec3<T> c0, c1, c2;

  static mat3 identity () {
    return mat3(1);
  }

  static mat3 translate (const vec2<T>& t) {
    return mat3(
      1, 0, 0,
      0, 1, 0,
      t.x, t.y, 1);
  }

  static mat3 rotate (T r) {
    return translate_rotate(vec2<T>(), r);
  }

  static mat3 scale (const vec2<T>& s) {
    return mat3(
      s.x, 0, 0,
      0, s.y, 0,
      0, 0, 1);
  }

  static mat3 translate_rotate (const vec2<T>& t, T r);
  static mat3 translate_rotate_scale (const vec2<T>& t, T r, const vec2<T>& s);

  explicit mat3 (T v = 0) : c0(v, 0, 0), c1(0, v, 0), c2(0, 0, v) {}
  mat3 (const vec3<T>& c0, const vec3<T>& c1, const vec3<T>& c2) : c0(c0), c1(c1), c2(c2) {}
  mat3 (T m00, T m01, T m02, T m10, T m11, T m12, T m20, T m21, T m22) :
    c0(m00, m01, m02), c1(m10, m11, m12), c2(m20, m21, m22) {}

  explicit operator mat2<T> () const {
    return mat2<T>(vec2<T>(c0), vec2<T>(c1));
  }

  vec3<T>& operator[] (size_t index) {
    return reinterpret_cast<vec3<T>*>(this)[index];
  }

  const vec3<T>& operator[] (size_t index) const {
    return reinterpret_cast<const vec3<T>*>(this)[index];
  }

  bool operator== (const mat3& other) const {
    return c0 == other.c0 && c1 == other.c1 && c2 == other.c2;
  }

  bool operator!= (const mat3& other) const {
    return c0 != other.c0 || c1 != other.c1 || c2 != other.c2;
  }

  mat3 operator+ (const mat3& other) const {
    return mat3(c0 + other.c0, c1 + other.c1, c2 + other.c2);
  }

  mat3 operator- (const mat3& other) const {
    return mat3(c0 - other.c0, c1 - other.c1, c2 - other.c2);
  }

  mat3 operator+ () const {
    return mat3(+c0, +c1, +c2);
  }

  mat3 operator- () const {
    return mat3(-c0, -c1, -c2);
  }

  mat3 operator* (T scalar) const {
    return mat3(c0 * scalar, c1 * scalar, c2 * scalar);
  }

  vec2<T> operator* (const vec2<T>& v) const {
    return vec2<T>(c0.x*v.x + c1.x*v.y + c2.x, c0.y*v.x + c1.y*v.y + c2.y);
  }

  vec3<T> operator* (const vec3<T>& v) const {
    return vec3<T>(c0.x*v.x + c1.x*v.y + c2.x*v.z, c0.y*v.x + c1.y*v.y + c2.y*v.z, c0.z*v.x + c1.z*v.y + c2.z*v.z);
  }

  mat3 operator* (const mat3& o) const;

  mat3 operator/ (T scalar) const {
    return *this * (1 / scalar);
  }

  mat3& operator+= (const mat3& other) {
    c0 += other.c0;
    c1 += other.c1;
    c2 += other.c2;
    return *this;
  }

  mat3& operator-= (const mat3& other) {
    c0 -= other.c0;
    c1 -= other.c1;
    c2 -= other.c2;
    return *this;
  }

  mat3& operator*= (T scalar) {
    c0 *= scalar;
    c1 *= scalar;
    c2 *= scalar;
    return *this;
  }

  mat3& operator/= (T scalar) {
    return *this *= (1 / scalar);
  }
};

template<typename S, typename T>
inline mat3<T> operator* (S scalar, const mat3<T>& matrix) {
  return mat3<T>(scalar * matrix.c0, scalar * matrix.c1, scalar * matrix.c2);
}

template<typename T>
inline vec3<T> operator* (const vec3<T>& v, const mat3<T>& m) {
  return vec3<T>(dot(v, m.c0), dot(v, m.c1), dot(v, m.c2));
}

template<typename T>
inline mat3<T> transpose (const mat3<T>& m) {
  return mat3<T>(m.c0.x, m.c1.x, m.c2.x, m.c0.y, m.c1.y, m.c2.y, m.c0.z, m.c1.z, m.c2.z);
}

template<typename T>
inline T determinant (const mat3<T>& m) {
  return
    m.c0.x*(m.c1.y*m.c2.z - m.c2.y*m.c1.z) -
    m.c1.x*(m.c0.y*m.c2.z - m.c2.y*m.c0.z) +
    m.c2.x*(m.c0.y*m.c1.z - m.c1.y*m.c0.z);
}

template<typename T>
mat3<T> inverse (const mat3<T>& m);

template<typename T>
inline mat3<T> mix (const mat3<T>& x, const mat3<T>& y, T a) {
  return x + (y - x) * a;
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const mat3<T>& matrix) {
  return out << '(' << matrix.c0 << ", " << matrix.c1 << ", " << matrix.c2 << ')';
}

typedef mat3<float> mat3f;
typedef mat3<double> mat3d;

template<typename T>
class plane;

template<typename T>
struct mat4x3 {
  vec3<T> c0, c1, c2, c3;

  static mat4x3 identity () {
    return mat4x3(1);
  }

  static mat4x3 translate (const vec3<T>& v) {
    // https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/glTranslate.xml
    return mat4x3(
      1, 0, 0,
      0, 1, 0,
      0, 0, 1,
      v.x, v.y, v.z);
  }

  static mat4x3 rotate (T angle, const vec3<T>& axis);

  static mat4x3 rotate (const euler<T>& euler) {
    return translate_rotate(vec3<T>(), euler);
  }

  static mat4x3 rotate (const quat<T>& quat) {
    return translate_rotate(vec3<T>(), quat);
  }

  static mat4x3 scale (const vec3<T>& v) {
    // https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/glScale.xml
    return mat4x3(
      v.x, 0, 0,
      0, v.y, 0,
      0, 0, v.z,
      0, 0, 0);
  }

  static mat4x3 translate_rotate (const vec3<T>& t, const euler<T>& r);
  static mat4x3 translate_rotate (const vec3<T>& t, const quat<T>& r);
  static mat4x3 translate_rotate_scale (const vec3<T>& t, const euler<T>& r, const vec3<T>& s);
  static mat4x3 translate_rotate_scale (const vec3<T>& t, const quat<T>& r, const vec3<T>& s);

  static mat4x3 reflect (const plane<T>& p);

  explicit mat4x3 (T v = 0) : c0(v, 0, 0), c1(0, v, 0), c2(0, 0, v), c3(0, 0, 0) {}
  mat4x3 (const vec3<T>& c0, const vec3<T>& c1, const vec3<T>& c2, const vec3<T>& c3) : c0(c0), c1(c1), c2(c2), c3(c3) {}
  mat4x3 (T m00, T m01, T m02, T m10, T m11, T m12, T m20, T m21, T m22, T m30, T m31, T m32) :
    c0(m00, m01, m02), c1(m10, m11, m12), c2(m20, m21, m22), c3(m30, m31, m32) {}

  explicit operator mat3<T> () const {
    return mat3<T>(c0, c1, c2);
  }

  vec3<T>& operator[] (size_t index) {
    return reinterpret_cast<vec3<T>*>(this)[index];
  }

  const vec3<T>& operator[] (size_t index) const {
    return reinterpret_cast<const vec3<T>*>(this)[index];
  }

  bool operator== (const mat4x3& other) const {
    return c0 == other.c0 && c1 == other.c1 && c2 == other.c2 && c3 == other.c3;
  }

  bool operator!= (const mat4x3& other) const {
    return c0 != other.c0 || c1 != other.c1 || c2 != other.c2 || c3 != other.c3;
  }

  mat4x3 operator+ (const mat4x3& other) const {
    return mat4x3(c0 + other.c0, c1 + other.c1, c2 + other.c2, c3 + other.c3);
  }

  mat4x3 operator- (const mat4x3& other) const {
    return mat4x3(c0 - other.c0, c1 - other.c1, c2 - other.c2, c3 - other.c3);
  }

  mat4x3 operator+ () const {
    return mat4x3(+c0, +c1, +c2, +c3);
  }

  mat4x3 operator- () const {
    return mat4x3(-c0, -c1, -c2, -c3);
  }

  mat4x3 operator* (T scalar) const {
    return mat4x3(c0 * scalar, c1 * scalar, c2 * scalar, c3 * scalar);
  }

  vec3<T> transform_vector (const vec3<T>& v) const {
    return vec3<T>(
      c0.x*v.x + c1.x*v.y + c2.x*v.z,
      c0.y*v.x + c1.y*v.y + c2.y*v.z,
      c0.z*v.x + c1.z*v.y + c2.z*v.z);
  }

  vec3<T> transform_normal (const vec3<T>& n) const {
    return normalize(inverse(transpose(mat3<T>(*this))) * n);
  }

  vec3<T> operator* (const vec3<T>& v) const {
    return vec3<T>(
      c0.x*v.x + c1.x*v.y + c2.x*v.z + c3.x,
      c0.y*v.x + c1.y*v.y + c2.y*v.z + c3.y,
      c0.z*v.x + c1.z*v.y + c2.z*v.z + c3.z);
  }

  vec4<T> operator* (const vec4<T>& v) const {
    return vec4<T>(
      c0.x*v.x + c1.x*v.y + c2.x*v.z + c3.x*v.w,
      c0.y*v.x + c1.y*v.y + c2.y*v.z + c3.y*v.w,
      c0.z*v.x + c1.z*v.y + c2.z*v.z + c3.z*v.w,
      v.w);
  }

  mat4x3 operator* (const mat4x3& o) const;

  mat4x3 operator/ (T scalar) const {
    return *this * (1 / scalar);
  }

  mat4x3& operator+= (const mat4x3& other) {
    c0 += other.c0;
    c1 += other.c1;
    c2 += other.c2;
    c3 += other.c3;
    return *this;
  }

  mat4x3& operator-= (const mat4x3& other) {
    c0 -= other.c0;
    c1 -= other.c1;
    c2 -= other.c2;
    c3 -= other.c3;
    return *this;
  }

  mat4x3& operator*= (T scalar) {
    c0 *= scalar;
    c1 *= scalar;
    c2 *= scalar;
    c3 *= scalar;
    return *this;
  }

  mat4x3& operator/= (T scalar) {
    return *this *= (1 / scalar);
  }
};

template<typename S, typename T>
inline mat4x3<T> operator* (S scalar, const mat4x3<T>& matrix) {
  return mat4x3<T>(scalar * matrix.c0, scalar * matrix.c1, scalar * matrix.c2, scalar * matrix.c3);
}

template<typename T>
inline vec4<T> operator* (const vec4<T>& v, const mat4x3<T>& m) {
  return vec4<T>(
    v.x*m.c0.x + v.y*m.c0.y + v.z*m.c0.z,
    v.x*m.c1.x + v.y*m.c1.y + v.z*m.c1.z,
    v.x*m.c2.x + v.y*m.c2.y + v.z*m.c2.z,
    v.x*m.c3.x + v.y*m.c3.y + v.z*m.c3.z + v.w);
}

template<typename T>
T determinant (const mat4x3<T>& m);

template<typename T>
mat4x3<T> inverse (const mat4x3<T>& m);

template<typename T>
inline mat4x3<T> mix (const mat4x3<T>& x, const mat4x3<T>& y, T a) {
  return x + (y - x) * a;
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const mat4x3<T>& matrix) {
  return out << '(' << matrix.c0 << ", " << matrix.c1 << ", " << matrix.c2 << ", " << matrix.c3 << ')';
}

typedef mat4x3<float> mat4x3f;
typedef mat4x3<double> mat4x3d;

template<typename T>
struct mat4 {
  vec4<T> c0, c1, c2, c3;

  static mat4 identity () {
    return mat4(1);
  }

  static mat4 translate (const vec3<T>& v) {
    return mat4(mat4x3<T>::translate(v));
  }

  static mat4 rotate (T angle, const vec3<T>& axis) {
    return mat4(mat4x3<T>::rotate(angle, axis));
  }

  static mat4 rotate (const euler<T>& euler) {
    return mat4(mat4x3<T>::rotate(euler));
  }

  static mat4 scale (const vec3<T>& v) {
    return mat4(mat4x3<T>::scale(v));
  }

  static mat4 rotate (const quat<T>& quat) {
    return mat4(mat4x3<T>::rotate(quat));
  }

  static mat4 translate_rotate (const vec3<T>& t, const euler<T>& r) {
    return mat4(mat4x3<T>::translate_rotate(t, r));
  }

  static mat4 translate_rotate (const vec3<T>& t, const quat<T>& r) {
    return mat4(mat4x3<T>::translate_rotate(t, r));
  }

  static mat4 translate_rotate_scale (const vec3<T>& t, const euler<T>& r, const vec3<T>& s) {
    return mat4(mat4x3<T>::translate_rotate_scale(t, r, s));
  }

  static mat4 translate_rotate_scale (const vec3<T>& t, const quat<T>& r, const vec3<T>& s) {
    return mat4(mat4x3<T>::translate_rotate_scale(t, r, s));
  }

  static mat4 reflect (const plane<T>& plane) {
    return mat4(mat4x3<T>::reflect(plane));
  }

  static mat4 ortho (T left, T right, T bottom, T top, T nearVal, T farVal);
  static mat4 frustum (T left, T right, T bottom, T top, T nearVal, T farVal);
  static mat4 perspective (T fovy, T aspect, T zNear, T zFar);

  explicit mat4 (T v = 0) : c0(v, 0, 0, 0), c1(0, v, 0, 0), c2(0, 0, v, 0), c3(0, 0, 0, v) {}
  mat4 (const vec4<T>& c0, const vec4<T>& c1, const vec4<T>& c2, const vec4<T>& c3) : c0(c0), c1(c1), c2(c2), c3(c3) {}
  mat4 (T m00, T m01, T m02, T m03, T m10, T m11, T m12, T m13, T m20, T m21, T m22, T m23, T m30, T m31, T m32, T m33) :
    c0(m00, m01, m02, m03), c1(m10, m11, m12, m13), c2(m20, m21, m22, m23), c3(m30, m31, m32, m33) {}
  explicit mat4 (const mat4x3<T>& m) : c0(m.c0, 0), c1(m.c1, 0), c2(m.c2, 0), c3(m.c3, 1) {}

  explicit operator mat4x3<T> () const {
    return mat4x3<T>(vec3<T>(c0), vec3<T>(c1), vec3<T>(c2), vec3<T>(c3));
  }

  explicit operator mat3<T> () const {
    return mat3<T>(vec3<T>(c0), vec3<T>(c1), vec3<T>(c2));
  }

  vec4<T>& operator[] (size_t index) {
    return reinterpret_cast<vec4<T>*>(this)[index];
  }

  const vec4<T>& operator[] (size_t index) const {
    return reinterpret_cast<const vec4<T>*>(this)[index];
  }

  bool operator== (const mat4& other) const {
    return c0 == other.c0 && c1 == other.c1 && c2 == other.c2 && c3 == other.c3;
  }

  bool operator!= (const mat4& other) const {
    return c0 != other.c0 || c1 != other.c1 || c2 != other.c2 || c3 != other.c3;
  }

  mat4 operator+ (const mat4& other) const {
    return mat4(c0 + other.c0, c1 + other.c1, c2 + other.c2, c3 + other.c3);
  }

  mat4 operator- (const mat4& other) const {
    return mat4(c0 - other.c0, c1 - other.c1, c2 - other.c2, c3 - other.c3);
  }

  mat4 operator+ () const {
    return mat4(+c0, +c1, +c2, +c3);
  }

  mat4 operator- () const {
    return mat4(-c0, -c1, -c2, -c3);
  }

  mat4 operator* (T scalar) const {
    return mat4(c0 * scalar, c1 * scalar, c2 * scalar, c3 * scalar);
  }

  vec3<T> transform_vector (const vec3<T>& v) const {
    return vec3<T>(
      c0.x*v.x + c1.x*v.y + c2.x*v.z,
      c0.y*v.x + c1.y*v.y + c2.y*v.z,
      c0.z*v.x + c1.z*v.y + c2.z*v.z);
  }

  vec3<T> transform_normal (const vec3<T>& n) const {
    return normalize(inverse(transpose(mat3<T>(*this))) * n);
  }

  vec3<T> operator* (const vec3<T>& v) const {
    return vec3<T>(
      c0.x*v.x + c1.x*v.y + c2.x*v.z + c3.x,
      c0.y*v.x + c1.y*v.y + c2.y*v.z + c3.y,
      c0.z*v.x + c1.z*v.y + c2.z*v.z + c3.z);
  }

  vec4<T> operator* (const vec4<T>& v) const {
    return vec4<T>(
      c0.x*v.x + c1.x*v.y + c2.x*v.z + c3.x*v.w,
      c0.y*v.x + c1.y*v.y + c2.y*v.z + c3.y*v.w,
      c0.z*v.x + c1.z*v.y + c2.z*v.z + c3.z*v.w,
      c0.w*v.x + c1.w*v.y + c2.w*v.z + c3.w*v.w);
  }

  mat4 operator* (const mat4& o) const;

  mat4 operator/ (T scalar) const {
    return *this * (1 / scalar);
  }

  mat4& operator+= (const mat4& other) {
    c0 += other.c0;
    c1 += other.c1;
    c2 += other.c2;
    c3 += other.c3;
    return *this;
  }

  mat4& operator-= (const mat4& other) {
    c0 -= other.c0;
    c1 -= other.c1;
    c2 -= other.c2;
    c3 -= other.c3;
    return *this;
  }

  mat4& operator*= (T scalar) {
    c0 *= scalar;
    c1 *= scalar;
    c2 *= scalar;
    c3 *= scalar;
    return *this;
  }

  mat4& operator/= (T scalar) {
    return *this *= (1 / scalar);
  }
};

template<typename S, typename T>
inline mat4<T> operator* (S scalar, const mat4<T>& matrix) {
  return mat4<T>(scalar * matrix.c0, scalar * matrix.c1, scalar * matrix.c2, scalar * matrix.c3);
}

template<typename T>
inline vec4<T> operator* (const vec4<T>& v, const mat4<T>& m) {
  return vec4<T>(dot(v, m.c0), dot(v, m.c1), dot(v, m.c2), dot(v, m.c3));
}

template<typename T>
inline mat4<T> transpose (const mat4<T>& m) {
  return mat4<T>(
    m.c0.x, m.c1.x, m.c2.x, m.c3.x,
    m.c0.y, m.c1.y, m.c2.y, m.c3.y,
    m.c0.z, m.c1.z, m.c2.z, m.c3.z,
    m.c0.w, m.c1.w, m.c2.w, m.c3.w);
}

template<typename T>
T determinant (const mat4<T>& m);

template<typename T>
mat4<T> inverse (const mat4<T>& m);

template<typename T>
inline mat4<T> mix (const mat4<T>& x, const mat4<T>& y, T a) {
  return x + (y - x) * a;
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const mat4<T>& matrix) {
  return out << '(' << matrix.c0 << ", " << matrix.c1 << ", " << matrix.c2 << ", " << matrix.c3 << ')';
}

typedef mat4<float> mat4f;
typedef mat4<double> mat4d;

enum class transform_type : unsigned { identity, trs, affine, general };
enum class rotation_type : unsigned { euler, quat };
enum class scale_type : unsigned { unit, uniform, nonuniform };

template<typename T>
class transform {
public:

  transform () : type_(transform_type::identity) {}
  transform (const vec3<T>& translation, const euler<T>& euler) :
    type_(transform_type::trs), rotation_type_(rotation_type::euler), scale_type_(scale_type::unit),
    trs_{translation, {vkm::quat<T>(euler.x, euler.y, euler.z, 0)}} {}
  transform (const vec3<T>& translation, const quat<T>& quat) :
    type_(transform_type::trs), rotation_type_(rotation_type::quat), scale_type_(scale_type::unit),
    trs_{translation, {quat}} {}
  transform (const vec3<T>& translation, const euler<T>& euler, T scale) :
    type_(transform_type::trs), rotation_type_(rotation_type::euler), scale_type_(scale_type::uniform),
    trs_{translation, {vkm::quat<T>(euler.x, euler.y, euler.z, 0)}, {vec3<T>(scale, 0, 0)}} {}
  transform (const vec3<T>& translation, const quat<T>& quat, T scale) :
    type_(transform_type::trs), rotation_type_(rotation_type::quat), scale_type_(scale_type::uniform),
    trs_{translation, {quat}, {vec3<T>(scale, 0, 0)}} {}
  transform (const vec3<T>& translation, const euler<T>& euler, const vec3<T>& scale) :
    type_(transform_type::trs), rotation_type_(rotation_type::euler), scale_type_(scale_type::nonuniform),
    trs_{translation, {vkm::quat<T>(euler.x, euler.y, euler.z, 0)}, {scale}} {}
  transform (const vec3<T>& translation, const quat<T>& quat, const vec3<T>& scale) :
    type_(transform_type::trs), rotation_type_(rotation_type::quat), scale_type_(scale_type::nonuniform),
    trs_{translation, {quat}, {scale}} {}
  explicit transform (const mat4x3<T>& affine_matrix) : type_(transform_type::affine), affine_matrix_(affine_matrix) {}
  explicit transform (const mat4<T>& general_matrix) : type_(transform_type::general), general_matrix_(general_matrix) {}

  explicit operator mat4x3<T> () const;
  explicit operator mat4<T> () const;

  bool operator== (const transform& other) const;

  bool operator!= (const transform& other) const {
    return !(*this == other);
  }

  transform_type type () const {
    return type_;
  }

  vkm::rotation_type rotation_type () const {
    return rotation_type_;
  }

  vkm::scale_type scale_type () const {
    return scale_type_;
  }

  void identity () {
    type_ = transform_type::identity;
  }

  vec3<T>& translation () {
    set_type_(transform_type::trs);
    return trs_.translation_;
  }

  const vec3<T>& translation () const {
    return trs_.translation_;
  }

  vkm::euler<T>& euler () {
    set_rotation_type_(rotation_type::euler);
    return trs_.euler_;
  }

  const vkm::euler<T>& euler () const {
    return trs_.euler_;
  }

  vkm::quat<T>& quat () {
    set_rotation_type_(rotation_type::quat);
    return trs_.quat_;
  }

  const vkm::quat<T>& quat () const {
    return trs_.quat_;
  }

  void unit_scale () {
    set_scale_type_(scale_type::unit);
  }

  T& uniform_scale () {
    set_scale_type_(scale_type::uniform);
    return trs_.uniform_scale_;
  }

  T uniform_scale () const {
    return trs_.uniform_scale_;
  }

  vec3<T>& nonuniform_scale () {
    set_scale_type_(scale_type::nonuniform);
    return trs_.nonuniform_scale_;
  }

  const vec3<T>& nonuniform_scale () const {
    return trs_.nonuniform_scale_;
  }

  mat4x3<T>& affine_matrix () {
    set_type_(transform_type::affine);
    return affine_matrix_;
  }

  const mat4x3<T>& affine_matrix () const {
    return affine_matrix_;
  }

  mat4<T>& general_matrix () {
    set_type_(transform_type::general);
    return general_matrix_;
  }

  const mat4<T>& general_matrix () const {
    return general_matrix_;
  }

  vec3<T> extract_translation () const;
  vkm::quat<T> extract_rotation () const;
  vec3<T> extract_scale () const;
  T extract_uniform_scale () const;

  vec3<T> transform_vector (const vec3<T>& v) const;
  vec3<T> transform_normal (const vec3<T>& n) const;

  vec3<T> operator* (const vec3<T>& v) const;
  vec4<T> operator* (const vec4<T>& v) const;

  transform<T> operator* (const transform<T>& other) const;

private:

  void set_type_ (transform_type type);
  void set_rotation_type_ (vkm::rotation_type type);
  void set_scale_type_ (vkm::scale_type type);

  transform_type type_ : 2;
  vkm::rotation_type rotation_type_ : 1;
  vkm::scale_type scale_type_ : 2;
  union {
    mat4x3<T> affine_matrix_;
    mat4<T> general_matrix_;
    struct {
      vec3<T> translation_;
      union {
        vkm::quat<T> quat_;
        vkm::euler<T> euler_;
      };
      union {
        vec3<T> nonuniform_scale_;
        T uniform_scale_;
      };
    } trs_;
  };
};

template<typename T>
transform<T> inverse (const transform<T>& transform);

template<typename T>
transform<T> mix (const transform<T>& x, const transform<T>& y, T a);

template<typename T>
std::ostream& operator<< (std::ostream& out, const transform<T>& transform);

typedef transform<float> transformf;
typedef transform<double> transformd;

}

#endif // VKM_MATH_H
