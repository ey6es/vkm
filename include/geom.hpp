#ifndef VKM_GEOM_H
#define VKM_GEOM_H

#include "math.hpp"

namespace vkm {

template<typename T>
struct rect {
  vec2<T> min, max;

  rect () : min(), max() {}
  rect (const vec2<T>& min, const vec2<T>& max) : min(min), max(max) {}

  bool operator== (const rect& other) const {
    return min == other.min && max == other.max;
  }

  bool operator!= (const rect& other) const {
    return min != other.min || max != other.max;
  }

  bool intersects (const rect& other) const {
    return
      other.max.x >= min.x && other.min.x <= max.x &&
      other.max.y >= min.y && other.min.y <= max.y;
  }

  bool contains (const rect& other) const {
    return
      other.min.x >= min.x && other.max.x <= max.x &&
      other.min.y >= min.y && other.max.y <= max.y;
  }
};

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const rect<T>& rect) {
  return out << '(' << rect.min << ", " << rect.max << ')';
}

typedef rect<float> rectf;
typedef rect<double> rectd;

template<typename T>
struct box {
  vec3<T> min, max;

  template<typename ForwardIterator>
  static box bounds (ForwardIterator first, ForwardIterator last) {
    box bounds(*first, *first);
    for (ForwardIterator it = ++first; it != last; it++) {
      bounds.add(*it);
    }
    return bounds;
  }

  box () : min(), max() {}
  box (const vec3<T>& min, const vec3<T>& max) : min(min), max(max) {}

  bool operator== (const box& other) const {
    return min == other.min && max == other.max;
  }

  bool operator!= (const box& other) const {
    return min != other.min || max != other.max;
  }

  void add (const vec3<T>& point) {
    min = vkm::min(min, point);
    max = vkm::max(max, point);
  }

  vec3<T> corner (int index) const {
    return vec3<T>(
      index & 1 ? max.x : min.x,
      index & 2 ? max.y : min.y,
      index & 4 ? max.z : min.z);
  }

  bool intersects (const box& other) const {
    return
      other.max.x >= min.x && other.min.x <= max.x &&
      other.max.y >= min.y && other.min.y <= max.y &&
      other.max.z >= min.z && other.min.z <= max.z;
  }

  bool contains (const box& other) const {
    return
      other.min.x >= min.x && other.max.x <= max.x &&
      other.min.y >= min.y && other.max.y <= max.y &&
      other.min.z >= min.z && other.max.z <= max.z;
  }
};

template<typename T>
box<T> operator* (const mat4x3<T>& mat, const box<T>& box);

template<typename T>
inline box<T> operator* (const transform<T>& transform, const box<T>& box) {
  return mat4x3<T>(transform) * box;
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const box<T>& box) {
  return out << '(' << box.min << ", " << box.max << ')';
}

typedef box<float> boxf;
typedef box<double> boxd;

template<typename T>
struct cuboid {
  mat4x3<T> matrix;

  explicit cuboid (T s = 0) : matrix(s) {}
  explicit cuboid (const mat4x3<T>& matrix) : matrix(matrix) {}

  vec3<T> corner (int index) const {
    return matrix * vec3<T>(
      index & 1 ? 0.5 : -0.5,
      index & 2 ? 0.5 : -0.5,
      index & 4 ? 0.5 : -0.5);
  }
};

template<typename T>
inline cuboid<T> operator* (const mat4x3<T>& mat, const cuboid<T>& cuboid) {
  return vkm::cuboid<T>(mat * cuboid.matrix);
}

template<typename T>
inline cuboid<T> operator* (const transform<T>& transform, const cuboid<T>& cuboid) {
  return mat4x3<T>(transform) * cuboid;
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const cuboid<T>& cuboid) {
  return out << '(' << cuboid.matrix << ')';
}

typedef cuboid<float> cuboidf;
typedef cuboid<double> cuboidd;

enum class intersection_type { none, intersects, contains };
enum class slab_orientation_type { below, within, above };

template<typename T>
struct plane {
  T a, b, c, d;

  template<typename ForwardIterator>
  static plane slab (const vec3<T>& normal, ForwardIterator first, ForwardIterator last) {
    T min_dp = dot(normal, *first);
    T max_dp = min_dp;
    for (ForwardIterator it = ++first; it != last; it++) {
      T dp = dot(normal, *it);
      min_dp = std::min(min_dp, dp);
      max_dp = std::max(max_dp, dp);
    }
    T width = max_dp - min_dp;
    if (width == 0) return plane(); // degenerate
    T scale = 1 / width;
    return plane(normal.x * scale, normal.y * scale, normal.z * scale, -min_dp * scale);
  }

  plane () : a(), b(), c(), d() {}
  plane (T a, T b, T c, T d) : a(a), b(b), c(c), d(d) {}
  plane (const vec3<T>& normal, T d) : a(normal.x), b(normal.y), c(normal.z), d(d) {}
  plane (const vec3<T>& point, const vec3<T>& normal) : a(normal.x), b(normal.y), c(normal.z), d(-dot(normal, point)) {}
  plane (const vec3<T>& a, const vec3<T>& b, const vec3<T>& c) : plane(a, normalize(cross(b - a, c - a))) {}

  vec3<T> normal () const {
    return vec3<T>(a, b, c);
  }

  plane reverse () const {
    return plane(-a, -b, -c, -d);
  }

  T signed_distance (const vec3<T>& point) const {
    return a*point.x + b*point.y + c*point.z + d;
  }

  T distance (const vec3<T>& point) const {
    return std::abs(signed_distance(point));
  }

  slab_orientation_type slab_orientation (const vec3<T>& point) const {
    T dist = signed_distance(point);
    if (dist < 0) return slab_orientation_type::below;
    if (dist > 1) return slab_orientation_type::above;
    return slab_orientation_type::within;
  }

  intersection_type half_space_intersects (const box<T>& b) const;

  // Tests for intersection with the "slab" formed by this plane.  The slab consists of all points above the plane up to
  // 1/length of the normal.
  intersection_type slab_intersects (const box<T>& b) const;
};

template<typename T>
inline plane<T> operator* (const mat4x3<T>& mat, const plane<T>& plane) {
  return vkm::plane<T>(mat * (-plane.d * plane.normal()), mat.transform_normal(plane.normal()));
}

template<typename T>
inline plane<T> operator* (const transform<T>& transform, const plane<T>& plane) {
  return vkm::plane<T>(transform * (-plane.d * plane.normal()), transform.transform_normal(plane.normal()));
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const plane<T>& plane) {
  return out << '(' << plane.a << ", " << plane.b << ", " << plane.c << ", " << plane.d << ')';
}

typedef plane<float> planef;
typedef plane<double> planed;

template<typename T>
struct triangle {
  vec3<T> v0, v1, v2;

  triangle () : v0(), v1(), v2() {}
  triangle (const vec3<T>& v0, const vec3<T>& v1, const vec3<T>& v2) : v0(v0), v1(v1), v2(v2) {}

  vec3<T> normal () const {
    return normalize(cross(v1 - v0, v2 - v0));
  }

  triangle reverse () const {
    return triangle(v2, v1, v0);
  }
};

template<typename T>
inline triangle<T> operator* (const mat4x3<T>& mat, const triangle<T>& triangle) {
  return vkm::triangle<T>(mat * triangle.v0, mat * triangle.v1, mat * triangle.v2);
}

template<typename T>
inline triangle<T> operator* (const transform<T>& transform, const triangle<T>& triangle) {
  return mat4x3<T>(transform) * triangle;
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const triangle<T>& triangle) {
  return out << '(' << triangle.v0 << ", " << triangle.v1 << ", " << triangle.v2 << ')';
}

typedef triangle<float> trianglef;
typedef triangle<double> triangled;

template<typename T>
struct sphere {
  vec3<T> center;
  T radius;

  sphere () : center(), radius() {}
  sphere (const vec3<T>& center, T radius) : center(center), radius(radius) {}

  bool intersects (const sphere& other) const {
    return distance(center, other.center) <= radius + other.radius;
  }

  bool contains (const sphere& other) const {
    return distance(center, other.center) <= radius - other.radius;
  }
};

template<typename T>
inline sphere<T> operator* (const mat4x3<T>& mat, const sphere<T>& sphere) {
  T scale = std::abs(std::cbrt(dot(cross(mat.c0, mat.c1), mat.c2)));
  return vkm::sphere<T>(mat * sphere.center, sphere.radius * scale);
}

template<typename T>
inline sphere<T> operator* (const transform<T>& transform, const sphere<T>& sphere) {
  return vkm::sphere<T>(transform * sphere.center, sphere.radius * std::abs(transform.extract_uniform_scale()));
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const sphere<T>& sphere) {
  return out << '(' << sphere.center << ", " << sphere.radius << ')';
}

typedef sphere<float> spheref;
typedef sphere<double> sphered;

template<typename T>
struct capsule {
  vec3<T> start;
  vec3<T> end;
  T radius;

  capsule () : start(), end(), radius() {}
  capsule (const vec3<T>& start, const vec3<T>& end, T radius) : start(start), end(end), radius(radius) {}
};

template<typename T>
inline capsule<T> operator* (const mat4x3<T>& mat, const capsule<T>& capsule) {
  T scale = std::abs(std::cbrt(dot(cross(mat.c0, mat.c1), mat.c2)));
  return vkm::capsule<T>(mat * capsule.start, mat * capsule.end, capsule.radius * scale);
}

template<typename T>
inline capsule<T> operator* (const transform<T>& transform, const capsule<T>& capsule) {
  return vkm::capsule<T>(transform * capsule.start, transform * capsule.end,
      capsule.radius * std::abs(transform.extract_uniform_scale()));
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const capsule<T>& capsule) {
  return out << '(' << capsule.start << ", " << capsule.end << ", " << capsule.radius << ')';
}

typedef capsule<float> capsulef;
typedef capsule<double> capsuled;

template<typename T>
struct ray {
  vec3<T> origin, dir;

  ray () : origin(), dir() {}
  ray (const vec3<T>& origin, const vec3<T>& dir) : origin(origin), dir(dir) {}

  vec3<T> at (T a) const {
    return origin + dir * a;
  }

  bool intersects (const box<T>& box, T* where = nullptr) const;
  bool intersects (const cuboid<T>& cuboid, T* where = nullptr, vec3<T>* normal = nullptr) const;

  // Note: only intersects the front side (as determined by the plane normal).
  bool intersects (const plane<T>& plane, T* where = nullptr) const;

  // Note: only intersects front side (as determined by CCW order).
  bool intersects (const triangle<T>& triangle, T* where = nullptr) const;

  bool intersects (const sphere<T>& sphere, T* where = nullptr) const;
  bool intersects (const capsule<T>& capsule, T* where = nullptr, vec3<T>* normal = nullptr) const;
};

template<typename T>
inline ray<T> operator* (const mat4x3<T>& mat, const ray<T>& ray) {
  return vkm::ray<T>(mat * ray.origin, mat.transform_vector(ray.dir));
}

template<typename T>
inline ray<T> operator* (const transform<T>& transform, const ray<T>& ray) {
  return vkm::ray<T>(transform * ray.origin, transform.transform_vector(ray.dir));
}

template<typename T>
inline std::ostream& operator<< (std::ostream& out, const ray<T>& ray) {
  return out << '(' << ray.origin << ", " << ray.dir << ')';
}

typedef ray<float> rayf;
typedef ray<double> rayd;

template<typename T>
class frustum {
public:

  // Initializes the frustum with the inverse of the view-projection matrix and the w values for the near and far clip planes
  // (which will be 1/1 for ortho and nearVal/farVal for perspective).
  void init (const mat4<T>& inv_view_proj, T near_w, T far_w);

  intersection_type intersects (const box<T>& b) const;

private:

  vec3<T> points_[8];
  box<T> bounds_;
  plane<T> plane_slabs_[5];
  plane<T> cross_product_slabs_[18];
};

typedef frustum<float> frustumf;
typedef frustum<double> frustumd;

}

#endif // VKM_GEOM_H
