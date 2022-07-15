#include "geom.hpp"

namespace vkm {

template<typename T>
box<T> operator* (const mat4x3<T>& mat, const box<T>& box) {
  vec3<T> first(mat * box.corner(0));
  vkm::box<T> bounds(first, first);
  for (int i = 1; i < 8; i++) {
    bounds.add(mat * box.corner(i));
  }
  return bounds;
}

template box<float> operator* (const mat4x3<float>& mat, const box<float>& box);
template box<double> operator* (const mat4x3<double>& mat, const box<double>& box);

template<typename T>
intersection_type plane<T>::half_space_intersects (const box<T>& b) const {
  // in order to return none/contains, all points must be on the same side as the first
  bool inside = signed_distance(b.corner(0)) >= 0;
  for (int i = 1; i < 8; i++) {
    if ((signed_distance(b.corner(i)) >= 0) != inside) return intersection_type::intersects;
  }
  return inside ? intersection_type::contains : intersection_type::none;
}

template intersection_type plane<float>::half_space_intersects (const box<float>& b) const;
template intersection_type plane<double>::half_space_intersects (const box<double>& b) const;

template<typename T>
intersection_type plane<T>::slab_intersects (const box<T>& b) const {
  // in order to return none/contains, all points must be in the same orientation as the first
  slab_orientation_type orientation = slab_orientation(b.corner(0));
  for (int i = 1; i < 8; i++) {
    if (slab_orientation(b.corner(i)) != orientation) return intersection_type::intersects;
  }
  return orientation == slab_orientation_type::within ? intersection_type::contains : intersection_type::none;
}

template intersection_type plane<float>::slab_intersects (const box<float>& b) const;
template intersection_type plane<double>::slab_intersects (const box<double>& b) const;

template<typename T>
bool ray<T>::intersects (const box<T>& box, T* where) const {
  int inside_count = 0;

  if (origin.x < box.min.x) {
    if (dir.x > 0) {
      T a = (box.min.x - origin.x) / dir.x;
      if (is_within(origin.y + a*dir.y, box.min.y, box.max.y) && is_within(origin.z + a*dir.z, box.min.z, box.max.z)) {
        if (where) *where = a;
        return true;
      }
    }
  } else if (origin.x > box.max.x) {
    if (dir.x < 0) {
      T a = (box.max.x - origin.x) / dir.x;
      if (is_within(origin.y + a*dir.y, box.min.y, box.max.y) && is_within(origin.z + a*dir.z, box.min.z, box.max.z)) {
        if (where) *where = a;
        return true;
      }
    }
  } else inside_count++;

  if (origin.y < box.min.y) {
    if (dir.y > 0) {
      T a = (box.min.y - origin.y) / dir.y;
      if (is_within(origin.x + a*dir.x, box.min.x, box.max.x) && is_within(origin.z + a*dir.z, box.min.z, box.max.z)) {
        if (where) *where = a;
        return true;
      }
    }
  } else if (origin.y > box.max.y) {
    if (dir.y < 0) {
      T a = (box.max.y - origin.y) / dir.y;
      if (is_within(origin.x + a*dir.x, box.min.x, box.max.x) && is_within(origin.z + a*dir.z, box.min.z, box.max.z)) {
        if (where) *where = a;
        return true;
      }
    }
  } else inside_count++;

  if (origin.z < box.min.z) {
    if (dir.z > 0) {
      T a = (box.min.z - origin.z) / dir.z;
      if (is_within(origin.x + a*dir.x, box.min.x, box.max.x) && is_within(origin.y + a*dir.y, box.min.y, box.max.y)) {
        if (where) *where = a;
        return true;
      }
    }
  } else if (origin.z > box.max.z) {
    if (dir.z < 0) {
      T a = (box.max.z - origin.z) / dir.z;
      if (is_within(origin.x + a*dir.x, box.min.x, box.max.x) && is_within(origin.y + a*dir.y, box.min.y, box.max.y)) {
        if (where) *where = a;
        return true;
      }
    }
  } else inside_count++;

  if (inside_count == 3) {
    if (where) *where = 0;
    return true;
  }

  return false;
}

template bool ray<float>::intersects (const box<float>& box, float* where) const;
template bool ray<double>::intersects (const box<double>& box, double* where) const;

template<typename T>
bool ray<T>::intersects (const cuboid<T>& box, T* where, vec3<T>* normal) const {
  // TODO
  return false;
}

template bool ray<float>::intersects (const cuboid<float>& cuboid, float* where, vec3<float>* normal) const;
template bool ray<double>::intersects (const cuboid<double>& cuboid, double* where, vec3<double>* normal) const;

template<typename T>
bool ray<T>::intersects (const plane<T>& plane, T* where) const {
  T divisor = dot(plane.normal(), dir);
  if (divisor == 0) return false;

  T a = -plane.signed_distance(origin) / divisor;
  if (a < 0) return false;
  if (where) *where = a;
  return true;
}

template bool ray<float>::intersects (const plane<float>& plane, float* where) const;
template bool ray<double>::intersects (const plane<double>& plane, double* where) const;

template<typename T>
bool ray<T>::intersects (const triangle<T>& triangle, T* where) const {
  vec3<T> abc = cross(triangle.v1 - triangle.v0, triangle.v2 - triangle.v0);
  T divisor = dot(abc, dir);
  if (divisor == 0) return false;

  T d = -dot(abc, triangle.v0);
  T a = -(dot(abc, origin) + d) / divisor;

  // TODO

  return false;
}

template bool ray<float>::intersects (const triangle<float>& plane, float* where) const;
template bool ray<double>::intersects (const triangle<double>& plane, double* where) const;

template<typename T>
bool ray<T>::intersects (const sphere<T>& sphere, T* where) const {
  // TODO
  return false;
}

template bool ray<float>::intersects (const sphere<float>& sphere, float* where) const;
template bool ray<double>::intersects (const sphere<double>& sphere, double* where) const;

template<typename T>
bool ray<T>::intersects (const capsule<T>& capsule, T* where, vec3<T>* normal) const {
  // TODO
  return false;
}

template bool ray<float>::intersects (const capsule<float>& capsule, float* where, vec3<float>* normal) const;
template bool ray<double>::intersects (const capsule<double>& capsule, double* where, vec3<double>* normal) const;

template<typename T>
void frustum<T>::init (const mat4<T>& inv_view_proj, T near_w, T far_w) {
  for (int i = 0; i < 8; i++) {
    points_[i] = vec3<T>(inv_view_proj * (i & 4 ?
      vec4<T>(i & 1 ? far_w : -far_w, i & 2 ? far_w : -far_w, far_w, far_w) :
      vec4<T>(i & 1 ? near_w : -near_w, i & 2 ? near_w : -near_w, -near_w, near_w)));
  }

  bounds_ = box<T>::bounds(points_, points_ + 8);

  vec3<T> edges[] = {
    points_[1] - points_[0],
    points_[2] - points_[0],
    points_[4] - points_[0],
    points_[5] - points_[1],
    points_[6] - points_[2],
    points_[7] - points_[3]};

  auto create_slab = [&](const vec3<T>& e1, const vec3<T>& e2) {
    return plane<T>::slab(cross(e1, e2), points_, points_ + 8);
  };

  plane_slabs_[0] = create_slab(edges[1], edges[0]);
  plane_slabs_[1] = create_slab(edges[0], edges[3]);
  plane_slabs_[2] = create_slab(edges[2], edges[1]);
  plane_slabs_[3] = create_slab(edges[4], edges[0]);
  plane_slabs_[4] = create_slab(edges[1], edges[5]);

  plane<T>* current_slab = cross_product_slabs_;
  for (const vec3<T>& edge : edges) {
    for (int axis = 0; axis < 3; axis++) {
      vec3<T> axis_edge;
      axis_edge[axis] = 1;
      *current_slab++ = create_slab(edge, axis_edge);
    }
  }
}

template void frustum<float>::init (const mat4<float>& inv_view_proj, float near_val, float far_val);
template void frustum<double>::init (const mat4<double>& inv_view_proj, double near_val, double far_val);

template<typename T>
intersection_type frustum<T>::intersects (const box<T>& b) const {
  // cheap initial bounds check
  if (!bounds_.intersects(b)) return intersection_type::none;

  // check box against slabs formed by planes of frustum
  int contained_count = 0;
  for (const plane<T>& plane_slab : plane_slabs_) {
    switch (plane_slab.slab_intersects(b)) {
      case intersection_type::none:
        return intersection_type::none;

      case intersection_type::contains:
        contained_count++;
        break;
    }
  }
  if (contained_count == 5) return intersection_type::contains;

  // check points of frustum against box planes
  for (int axis = 0; axis < 3; axis++) {
    T min = b.min[axis];
    bool any_outside_min = false;
    for (const vec3<T>& point : points_) {
      if (point[axis] >= min) goto not_outside_min;
      any_outside_min = true;
    }
    return intersection_type::none;

    not_outside_min:

    if (any_outside_min) continue; // at least one point is definitely inside the max

    T max = b.max[axis];
    for (const vec3<T>& point : points_) {
      if (point[axis] <= max) goto not_outside_max;
    }
    return intersection_type::none;

    not_outside_max: ;
  }

  // check box against cross product slabs
  for (const plane<T>& cross_product_slab : cross_product_slabs_) {
    if (cross_product_slab.slab_intersects(b) == intersection_type::none) return intersection_type::none;
  }

  return intersection_type::intersects;
}

template intersection_type frustum<float>::intersects (const box<float>& b) const;
template intersection_type frustum<double>::intersects (const box<double>& b) const;

}
