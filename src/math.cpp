#include "math.hpp"
#include "geom.hpp"

namespace vkm {

template<typename T>
vec3<T> euler<T>::operator* (const vec3<T>& vec) const {
  T sx = std::sin(x);
  T cx = std::cos(x);
  T sy = std::sin(y);
  T cy = std::cos(y);
  T sz = std::sin(z);
  T cz = std::cos(z);
  T czsy = cz*sy;
  T szsy = sz*sy;
  return vec3<T>(
    vec.x*cz*cy + vec.y*(czsy*sx - sz*cx) + vec.z*(czsy*cx + sz*sx),
    vec.x*sz*cy + vec.y*(szsy*sx + cz*cx) + vec.z*(szsy*cx - cz*sx),
    -vec.x*sy + vec.y*cy*sx + vec.z*cy*cx);
}

template vec3<float> euler<float>::operator* (const vec3<float>& vec) const;
template vec3<double> euler<double>::operator* (const vec3<double>& vec) const;

template<typename T>
quat<T>::quat (T angle, const vec3<T>& axis) {
  T ha = angle / 2;
  T sinha = std::sin(ha);
  x = axis.x * sinha;
  y = axis.y * sinha;
  z = axis.z * sinha;
  w = std::cos(ha);
}

template quat<float>::quat (float angle, const vec3<float>& axis);
template quat<double>::quat (double angle, const vec3<double>& axis);

template<typename T>
quat<T>::quat (const euler<T>& euler) {
  T hx = euler.x / 2;
  T hy = euler.y / 2;
  T hz = euler.z / 2;
  T sx = std::sin(hx);
  T cx = std::cos(hx);
  T sy = std::sin(hy);
  T cy = std::cos(hy);
  T sz = std::sin(hz);
  T cz = std::cos(hz);
  T cysx = cy * sx;
  T cxsy = cx * sy;
  T cycx = cy * cx;
  T sysx = sy * sx;
  x = cz*cysx - sz*cxsy;
  y = cz*cxsy + sz*cysx;
  z = sz*cycx - cz*sysx;
  w = cz*cycx + sz*sysx;
}

template quat<float>::quat (const euler<float>& euler);
template quat<double>::quat (const euler<double>& euler);

template<typename T>
quat<T>::operator euler<T> () const {
  // https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
  T xx = x * x;
  T yy = y * y;
  T zz = z * z;
  T xy = x * y;
  T xz = x * z;
  T xw = x * w;
  T yz = y * z;
  T yw = y * w;
  T zw = z * w;
  T xx2 = xx + xx;
  T yy2 = yy + yy;
  T zz2 = zz + zz;
  T xy2 = xy + xy;
  T xz2 = xz + xz;
  T xw2 = xw + xw;
  T yz2 = yz + yz;
  T yw2 = yw + yw;
  T zw2 = zw + zw;
  T m02 = xz2 - yw2;
  return euler<T>(
    std::atan2(yz2 + xw2, 1 - xx2 - yy2),
    std::atan2(-m02, std::sqrt(1 - m02*m02)),
    std::atan2(xy2 + zw2, 1 - yy2 - zz2));
}

template quat<float>::operator euler<float> () const;
template quat<double>::operator euler<double> () const;

template<typename T>
vec3<T> quat<T>::operator* (const vec3<T>& vec) const {
  // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Performance_comparisons
  T x2 = x + x;
  T y2 = y + y;
  T z2 = z + z;
  T tx = y*vec.z - z*vec.y + w*vec.x;
  T ty = z*vec.x - x*vec.z + w*vec.y;
  T tz = x*vec.y - y*vec.x + w*vec.z;
  return vec3<T>(
    vec.x + y2*tz - z2*ty,
    vec.y + z2*tx - x2*tz,
    vec.z + x2*ty - y2*tx);
}

template vec3<float> quat<float>::operator* (const vec3<float>& vec) const;
template vec3<double> quat<double>::operator* (const vec3<double>& vec) const;

template<typename T>
quat<T> mix (const quat<T>& x, const quat<T>& y, T a) {
  // https://blog.magnum.graphics/backstage/the-unnecessarily-short-ways-to-do-a-quaternion-slerp/
  T d = dot(x, y);
  quat<T> xp = x;
  if (d < 0) {
    d = -d;
    xp = -x;
  }
  if (d == 1) return x;
  T theta = std::acos(d);
  return (std::sin((1 - a) * theta)*xp + std::sin(a * theta)*y) / std::sin(theta);
}

template quat<float> mix (const quat<float>& x, const quat<float>& y, float a);
template quat<double> mix (const quat<double>& x, const quat<double>& y, double a);

template<typename T>
mat3<T> mat3<T>::translate_rotate (const vec2<T>& t, T r) {
  T cosa = std::cos(r);
  T sina = std::sin(r);
  return mat3(
    cosa, sina, 0,
    -sina, cosa, 0,
    t.x, t.y, 1);
}

template mat3<float> mat3<float>::translate_rotate (const vec2<float>& t, float r);
template mat3<double> mat3<double>::translate_rotate (const vec2<double>& t, double r);

template<typename T>
mat3<T> mat3<T>::translate_rotate_scale (const vec2<T>& t, T r, const vec2<T>& s) {
  T cosa = std::cos(r);
  T sina = std::sin(r);
  return mat3(
    cosa * s.x, sina * s.x, 0,
    -sina * s.y, cosa * s.y, 0,
    t.x, t.y, 1);
}

template mat3<float> mat3<float>::translate_rotate_scale (const vec2<float>& t, float r, const vec2<float>& s);
template mat3<double> mat3<double>::translate_rotate_scale (const vec2<double>& t, double r, const vec2<double>& s);

template<typename T>
mat3<T> mat3<T>::operator* (const mat3<T>& o) const {
  return mat3(
    c0.x*o.c0.x + c1.x*o.c0.y + c2.x*o.c0.z,
    c0.y*o.c0.x + c1.y*o.c0.y + c2.y*o.c0.z,
    c0.z*o.c0.x + c1.z*o.c0.y + c2.z*o.c0.z,

    c0.x*o.c1.x + c1.x*o.c1.y + c2.x*o.c1.z,
    c0.y*o.c1.x + c1.y*o.c1.y + c2.y*o.c1.z,
    c0.z*o.c1.x + c1.z*o.c1.y + c2.z*o.c1.z,

    c0.x*o.c2.x + c1.x*o.c2.y + c2.x*o.c2.z,
    c0.y*o.c2.x + c1.y*o.c2.y + c2.y*o.c2.z,
    c0.z*o.c2.x + c1.z*o.c2.y + c2.z*o.c2.z);
}

template mat3<float> mat3<float>::operator* (const mat3<float>& o) const;
template mat3<double> mat3<double>::operator* (const mat3<double>& o) const;

template<typename T>
mat3<T> inverse (const mat3<T>& m) {
  // https://en.wikipedia.org/wiki/Invertible_matrix#Inversion_of_3_%C3%97_3_matrices
  T a = m.c1.y*m.c2.z - m.c2.y*m.c1.z;
  T b = m.c2.y*m.c0.z - m.c0.y*m.c2.z;
  T c = m.c0.y*m.c1.z - m.c1.y*m.c0.z;
  T s = 1 / (m.c0.x*a + m.c1.x*b + m.c2.x*c);
  return mat3<T>(
    a * s, b * s, c * s,
    (m.c2.x*m.c1.z - m.c1.x*m.c2.z) * s, (m.c0.x*m.c2.z - m.c2.x*m.c0.z) * s, (m.c1.x*m.c0.z - m.c0.x*m.c1.z) * s,
    (m.c1.x*m.c2.y - m.c2.x*m.c1.y) * s, (m.c2.x*m.c0.y - m.c0.x*m.c2.y) * s, (m.c0.x*m.c1.y - m.c1.x*m.c0.y) * s);
}

template mat3<float> inverse (const mat3<float>& m);
template mat3<double> inverse (const mat3<double>& m);

template<typename T>
mat4x3<T> mat4x3<T>::rotate (T angle, const vec3<T>& axis) {
  // https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/glRotate.xml
  T c = std::cos(angle);
  T cc = 1 - c;
  T s = std::sin(angle);
  T xcc = axis.x * cc;
  T ycc = axis.y * cc;
  T zcc = axis.z * cc;
  T xycc = xcc * axis.y;
  T xzcc = xcc * axis.z;
  T yzcc = ycc * axis.z;
  T xs = axis.x * s;
  T ys = axis.y * s;
  T zs = axis.z * s;
  return mat4x3(
    axis.x*xcc + c, xycc + zs, xzcc - ys,
    xycc - zs, axis.y*ycc + c, yzcc + xs,
    xzcc + ys, yzcc - xs, axis.z*zcc + c,
    0, 0, 0);
}

template mat4x3<float> mat4x3<float>::rotate (float angle, const vec3<float>& axis);
template mat4x3<double> mat4x3<double>::rotate (double angle, const vec3<double>& axis);

template<typename T>
mat4x3<T> mat4x3<T>::translate_rotate (const vec3<T>& t, const euler<T>& r) {
  T sx = std::sin(r.x);
  T cx = std::cos(r.x);
  T sy = std::sin(r.y);
  T cy = std::cos(r.y);
  T sz = std::sin(r.z);
  T cz = std::cos(r.z);
  T czsy = cz*sy;
  T szsy = sz*sy;
  return mat4x3(
    cz*cy, sz*cy, -sy,
    czsy*sx - sz*cx, szsy*sx + cz*cx, cy*sx,
    czsy*cx + sz*sx, szsy*cx - cz*sx, cy*cx,
    t.x, t.y, t.z);
}

template mat4x3<float> mat4x3<float>::translate_rotate (const vec3<float>& t, const euler<float>& r);
template mat4x3<double> mat4x3<double>::translate_rotate (const vec3<double>& t, const euler<double>& r);

template<typename T>
mat4x3<T> mat4x3<T>::translate_rotate (const vec3<T>& t, const quat<T>& r) {
  // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
  T xx = r.x * r.x;
  T yy = r.y * r.y;
  T zz = r.z * r.z;
  T xy = r.x * r.y;
  T xz = r.x * r.z;
  T xw = r.x * r.w;
  T yz = r.y * r.z;
  T yw = r.y * r.w;
  T zw = r.z * r.w;
  T xx2 = xx + xx;
  T yy2 = yy + yy;
  T zz2 = zz + zz;
  T xy2 = xy + xy;
  T xz2 = xz + xz;
  T xw2 = xw + xw;
  T yz2 = yz + yz;
  T yw2 = yw + yw;
  T zw2 = zw + zw;
  return mat4x3(
    1 - yy2 - zz2, xy2 + zw2, xz2 - yw2,
    xy2 - zw2, 1 - xx2 - zz2, yz2 + xw2,
    xz2 + yw2, yz2 - xw2, 1 - xx2 - yy2,
    t.x, t.y, t.z);
}

template mat4x3<float> mat4x3<float>::translate_rotate (const vec3<float>& t, const quat<float>& r);
template mat4x3<double> mat4x3<double>::translate_rotate (const vec3<double>& t, const quat<double>& r);

template<typename T>
mat4x3<T> mat4x3<T>::translate_rotate_scale (const vec3<T>& t, const euler<T>& r, const vec3<T>& s) {
  T sx = std::sin(r.x);
  T cx = std::cos(r.x);
  T sy = std::sin(r.y);
  T cy = std::cos(r.y);
  T sz = std::sin(r.z);
  T cz = std::cos(r.z);
  T czsy = cz*sy;
  T szsy = sz*sy;
  return mat4x3(
    vec3<T>(cz*cy, sz*cy, -sy) * s.x,
    vec3<T>(czsy*sx - sz*cx, szsy*sx + cz*cx, cy*sx) * s.y,
    vec3<T>(czsy*cx + sz*sx, szsy*cx - cz*sx, cy*cx) * s.z,
    t);
}

template mat4x3<float> mat4x3<float>::translate_rotate_scale (const vec3<float>& t, const euler<float>& r, const vec3<float>& s);
template mat4x3<double> mat4x3<double>::translate_rotate_scale (const vec3<double>& t, const euler<double>& r, const vec3<double>& s);

template<typename T>
mat4x3<T> mat4x3<T>::translate_rotate_scale (const vec3<T>& t, const quat<T>& r, const vec3<T>& s) {
  // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix
  T xx = r.x * r.x;
  T yy = r.y * r.y;
  T zz = r.z * r.z;
  T xy = r.x * r.y;
  T xz = r.x * r.z;
  T xw = r.x * r.w;
  T yz = r.y * r.z;
  T yw = r.y * r.w;
  T zw = r.z * r.w;
  T xx2 = xx + xx;
  T yy2 = yy + yy;
  T zz2 = zz + zz;
  T xy2 = xy + xy;
  T xz2 = xz + xz;
  T xw2 = xw + xw;
  T yz2 = yz + yz;
  T yw2 = yw + yw;
  T zw2 = zw + zw;
  return mat4x3(
    vec3<T>(1 - yy2 - zz2, xy2 + zw2, xz2 - yw2) * s.x,
    vec3<T>(xy2 - zw2, 1 - xx2 - zz2, yz2 + xw2) * s.y,
    vec3<T>(xz2 + yw2, yz2 - xw2, 1 - xx2 - yy2) * s.z,
    t);
}

template mat4x3<float> mat4x3<float>::translate_rotate_scale (const vec3<float>& t, const quat<float>& r, const vec3<float>& s);
template mat4x3<double> mat4x3<double>::translate_rotate_scale (const vec3<double>& t, const quat<double>& r, const vec3<double>& s);

template<typename T>
mat4x3<T> mat4x3<T>::reflect (const plane<T>& p) {
  T na2 = -(p.a + p.a);
  T nb2 = -(p.b + p.b);
  T nc2 = -(p.c + p.c);
  return mat4x3(
    1 + na2 * p.a, nb2 * p.a, nc2 * p.a,
    na2 * p.b, 1 + nb2 * p.b, nc2 * p.b,
    na2 * p.c, nb2 * p.c, 1 + nc2 * p.c,
    na2 * p.d, nb2 * p.d, nc2 * p.d);
}

template mat4x3<float> mat4x3<float>::reflect (const plane<float>& p);
template mat4x3<double> mat4x3<double>::reflect (const plane<double>& p);

template<typename T>
mat4x3<T> mat4x3<T>::operator* (const mat4x3<T>& o) const {
  return mat4x3(
    c0.x*o.c0.x + c1.x*o.c0.y + c2.x*o.c0.z,
    c0.y*o.c0.x + c1.y*o.c0.y + c2.y*o.c0.z,
    c0.z*o.c0.x + c1.z*o.c0.y + c2.z*o.c0.z,

    c0.x*o.c1.x + c1.x*o.c1.y + c2.x*o.c1.z,
    c0.y*o.c1.x + c1.y*o.c1.y + c2.y*o.c1.z,
    c0.z*o.c1.x + c1.z*o.c1.y + c2.z*o.c1.z,

    c0.x*o.c2.x + c1.x*o.c2.y + c2.x*o.c2.z,
    c0.y*o.c2.x + c1.y*o.c2.y + c2.y*o.c2.z,
    c0.z*o.c2.x + c1.z*o.c2.y + c2.z*o.c2.z,

    c0.x*o.c3.x + c1.x*o.c3.y + c2.x*o.c3.z + c3.x,
    c0.y*o.c3.x + c1.y*o.c3.y + c2.y*o.c3.z + c3.y,
    c0.z*o.c3.x + c1.z*o.c3.y + c2.z*o.c3.z + c3.z);
}

template mat4x3<float> mat4x3<float>::operator* (const mat4x3<float>& o) const;
template mat4x3<double> mat4x3<double>::operator* (const mat4x3<double>& o) const;

template<typename T>
T determinant (const mat4x3<T>& m) {
  return
    m.c0.x * (m.c1.y*m.c2.z - m.c2.y*m.c1.z) -
    m.c1.x * (m.c0.y*m.c2.z - m.c2.y*m.c0.z) +
    m.c2.x * (m.c0.y*m.c1.z - m.c1.y*m.c0.z);
}

template float determinant<float> (const mat4x3<float>& m);
template double determinant<double> (const mat4x3<double>& m);

template<typename T>
mat4x3<T> inverse (const mat4x3<T>& m) {
  T a = m.c1.y*m.c2.z - m.c2.y*m.c1.z;
  T b = -(m.c0.y*m.c2.z - m.c2.y*m.c0.z);
  T c = m.c0.y*m.c1.z - m.c1.y*m.c0.z;
  T s = 1 / (m.c0.x*a + m.c1.x*b + m.c2.x*c);

  T yz23 = m.c2.y*m.c3.z - m.c3.y*m.c2.z;
  T yz13 = m.c1.y*m.c3.z - m.c3.y*m.c1.z;
  T yz12 = m.c1.y*m.c2.z - m.c2.y*m.c1.z;
  T yz03 = m.c0.y*m.c3.z - m.c3.y*m.c0.z;
  T yz02 = m.c0.y*m.c2.z - m.c2.y*m.c0.z;
  T yz01 = m.c0.y*m.c1.z - m.c1.y*m.c0.z;

  return mat4x3<T>(
    a * s, b * s, c * s,

    -(m.c1.x*m.c2.z - m.c2.x*m.c1.z) * s,
    (m.c0.x*m.c2.z - m.c2.x*m.c0.z) * s,
    -(m.c0.x*m.c1.z - m.c1.x*m.c0.z) * s,

    (m.c1.x*m.c2.y - m.c2.x*m.c1.y) * s,
    -(m.c0.x*m.c2.y - m.c2.x*m.c0.y) * s,
    (m.c0.x*m.c1.y - m.c1.x*m.c0.y) * s,

    -(m.c1.x*yz23 - m.c2.x*yz13 + m.c3.x*yz12) * s,
    (m.c0.x*yz23 - m.c2.x*yz03 + m.c3.x*yz02) * s,
    -(m.c0.x*yz13 - m.c1.x*yz03 + m.c3.x*yz01) * s);
}

template mat4x3<float> inverse<float> (const mat4x3<float>& m);
template mat4x3<double> inverse<double> (const mat4x3<double>& m);

template<typename T>
mat4<T> mat4<T>::ortho (T left, T right, T bottom, T top, T nearVal, T farVal) {
  // https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/glOrtho.xml
  T tx = -(right + left) / (right - left);
  T ty = -(top + bottom) / (top - bottom);
  T tz = -(farVal + nearVal) / (farVal - nearVal);
  return mat4(
    2 / (right - left), 0, 0, 0,
    0, 2 / (top - bottom), 0, 0,
    0, 0, -2 / (farVal - nearVal), 0,
    tx, ty, tz, 1);
}

template mat4<float> mat4<float>::ortho (float left, float right, float bottom, float top, float nearVal, float farVal);
template mat4<double> mat4<double>::ortho (double left, double right, double bottom, double top, double nearVal, double farVal);

template<typename T>
mat4<T> mat4<T>::frustum (T left, T right, T bottom, T top, T nearVal, T farVal) {
  // https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/glFrustum.xml
  T a = (right + left) / (right - left);
  T b = (top + bottom) / (top - bottom);
  T c = -(farVal + nearVal) / (farVal - nearVal);
  T d = -2*farVal*nearVal / (farVal - nearVal);
  return mat4(
    2 * nearVal / (right - left), 0, 0, 0,
    0, 2 * nearVal / (top - bottom), 0, 0,
    a, b, c, -1,
    0, 0, d, 0);
}

template mat4<float> mat4<float>::frustum (float left, float right, float bottom, float top, float nearVal, float farVal);
template mat4<double> mat4<double>::frustum (double left, double right, double bottom, double top, double nearVal, double farVal);

template<typename T>
mat4<T> mat4<T>::perspective (T fovy, T aspect, T zNear, T zFar) {
  // https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/gluPerspective.xml
  T f = 1 / std::tan(fovy / 2);
  return mat4(
    f / aspect, 0, 0, 0,
    0, f, 0, 0,
    0, 0, (zFar + zNear) / (zNear - zFar), -1,
    0, 0, 2*zFar*zNear / (zNear - zFar), 0);
}

template mat4<float> mat4<float>::perspective (float fovy, float aspect, float zNear, float zFar);
template mat4<double> mat4<double>::perspective (double fovy, double aspect, double zNear, double zFar);

template<typename T>
mat4<T> mat4<T>::operator* (const mat4<T>& o) const {
  return mat4(
    c0.x*o.c0.x + c1.x*o.c0.y + c2.x*o.c0.z + c3.x*o.c0.w,
    c0.y*o.c0.x + c1.y*o.c0.y + c2.y*o.c0.z + c3.y*o.c0.w,
    c0.z*o.c0.x + c1.z*o.c0.y + c2.z*o.c0.z + c3.z*o.c0.w,
    c0.w*o.c0.x + c1.w*o.c0.y + c2.w*o.c0.z + c3.w*o.c0.w,

    c0.x*o.c1.x + c1.x*o.c1.y + c2.x*o.c1.z + c3.x*o.c1.w,
    c0.y*o.c1.x + c1.y*o.c1.y + c2.y*o.c1.z + c3.y*o.c1.w,
    c0.z*o.c1.x + c1.z*o.c1.y + c2.z*o.c1.z + c3.z*o.c1.w,
    c0.w*o.c1.x + c1.w*o.c1.y + c2.w*o.c1.z + c3.w*o.c1.w,

    c0.x*o.c2.x + c1.x*o.c2.y + c2.x*o.c2.z + c3.x*o.c2.w,
    c0.y*o.c2.x + c1.y*o.c2.y + c2.y*o.c2.z + c3.y*o.c2.w,
    c0.z*o.c2.x + c1.z*o.c2.y + c2.z*o.c2.z + c3.z*o.c2.w,
    c0.w*o.c2.x + c1.w*o.c2.y + c2.w*o.c2.z + c3.w*o.c2.w,

    c0.x*o.c3.x + c1.x*o.c3.y + c2.x*o.c3.z + c3.x*o.c3.w,
    c0.y*o.c3.x + c1.y*o.c3.y + c2.y*o.c3.z + c3.y*o.c3.w,
    c0.z*o.c3.x + c1.z*o.c3.y + c2.z*o.c3.z + c3.z*o.c3.w,
    c0.w*o.c3.x + c1.w*o.c3.y + c2.w*o.c3.z + c3.w*o.c3.w);
}

template mat4<float> mat4<float>::operator* (const mat4<float>& o) const;
template mat4<double> mat4<double>::operator* (const mat4<double>& o) const;

template<typename T>
T determinant (const mat4<T>& m) {
  T zw23 = m.c2.z*m.c3.w - m.c3.z*m.c2.w;
  T zw13 = m.c1.z*m.c3.w - m.c3.z*m.c1.w;
  T zw12 = m.c1.z*m.c2.w - m.c2.z*m.c1.w;
  T zw03 = m.c0.z*m.c3.w - m.c3.z*m.c0.w;
  T zw02 = m.c0.z*m.c2.w - m.c2.z*m.c0.w;
  T zw01 = m.c0.z*m.c1.w - m.c1.z*m.c0.w;
  return
    m.c0.x * (m.c1.y*zw23 - m.c2.y*zw13 + m.c3.y*zw12) -
    m.c1.x * (m.c0.y*zw23 - m.c2.y*zw03 + m.c3.y*zw02) +
    m.c2.x * (m.c0.y*zw13 - m.c1.y*zw03 + m.c3.y*zw01) -
    m.c3.x * (m.c0.y*zw12 - m.c1.y*zw02 + m.c2.y*zw01);
}

template float determinant<float> (const mat4<float>& m);
template double determinant<double> (const mat4<double>& m);

template<typename T>
mat4<T> inverse (const mat4<T>& m) {
  T zw23 = m.c2.z*m.c3.w - m.c3.z*m.c2.w;
  T zw13 = m.c1.z*m.c3.w - m.c3.z*m.c1.w;
  T zw12 = m.c1.z*m.c2.w - m.c2.z*m.c1.w;
  T zw03 = m.c0.z*m.c3.w - m.c3.z*m.c0.w;
  T zw02 = m.c0.z*m.c2.w - m.c2.z*m.c0.w;
  T zw01 = m.c0.z*m.c1.w - m.c1.z*m.c0.w;

  T a = m.c1.y*zw23 - m.c2.y*zw13 + m.c3.y*zw12;
  T b = -(m.c0.y*zw23 - m.c2.y*zw03 + m.c3.y*zw02);
  T c = m.c0.y*zw13 - m.c1.y*zw03 + m.c3.y*zw01;
  T d = -(m.c0.y*zw12 - m.c1.y*zw02 + m.c2.y*zw01);
  T s = 1 / (m.c0.x*a + m.c1.x*b + m.c2.x*c + m.c3.x*d);

  T yw23 = m.c2.y*m.c3.w - m.c3.y*m.c2.w;
  T yw13 = m.c1.y*m.c3.w - m.c3.y*m.c1.w;
  T yw12 = m.c1.y*m.c2.w - m.c2.y*m.c1.w;
  T yw03 = m.c0.y*m.c3.w - m.c3.y*m.c0.w;
  T yw02 = m.c0.y*m.c2.w - m.c2.y*m.c0.w;
  T yw01 = m.c0.y*m.c1.w - m.c1.y*m.c0.w;

  T yz23 = m.c2.y*m.c3.z - m.c3.y*m.c2.z;
  T yz13 = m.c1.y*m.c3.z - m.c3.y*m.c1.z;
  T yz12 = m.c1.y*m.c2.z - m.c2.y*m.c1.z;
  T yz03 = m.c0.y*m.c3.z - m.c3.y*m.c0.z;
  T yz02 = m.c0.y*m.c2.z - m.c2.y*m.c0.z;
  T yz01 = m.c0.y*m.c1.z - m.c1.y*m.c0.z;

  return mat4<T>(
    a * s, b * s, c * s, d * s,

    -(m.c1.x*zw23 - m.c2.x*zw13 + m.c3.x*zw12) * s,
    (m.c0.x*zw23 - m.c2.x*zw03 + m.c3.x*zw02) * s,
    -(m.c0.x*zw13 - m.c1.x*zw03 + m.c3.x*zw01) * s,
    (m.c0.x*zw12 - m.c1.x*zw02 + m.c2.x*zw01) * s,

    (m.c1.x*yw23 - m.c2.x*yw13 + m.c3.x*yw12) * s,
    -(m.c0.x*yw23 - m.c2.x*yw03 + m.c3.x*yw02) * s,
    (m.c0.x*yw13 - m.c1.x*yw03 + m.c3.x*yw01) * s,
    -(m.c0.x*yw12 - m.c1.x*yw02 + m.c2.x*yw01) * s,

    -(m.c1.x*yz23 - m.c2.x*yz13 + m.c3.x*yz12) * s,
    (m.c0.x*yz23 - m.c2.x*yz03 + m.c3.x*yz02) * s,
    -(m.c0.x*yz13 - m.c1.x*yz03 + m.c3.x*yz01) * s,
    (m.c0.x*yz12 - m.c1.x*yz02 + m.c2.x*yz01) * s);
}

template mat4<float> inverse<float> (const mat4<float>& m);
template mat4<double> inverse<double> (const mat4<double>& m);

template<typename T>
transform<T>::operator mat4x3<T> () const {
  switch (type_) {
    default:
    case transform_type::identity:
      return mat4x3<T>::identity();

    case transform_type::trs:
      switch (scale_type_) {
        default:
        case scale_type::unit:
          return (rotation_type_ == rotation_type::euler)
            ? mat4x3<T>::translate_rotate(trs_.translation_, trs_.euler_)
            : mat4x3<T>::translate_rotate(trs_.translation_, trs_.quat_);
        case scale_type::uniform:
          return (rotation_type_ == rotation_type::euler)
            ? mat4x3<T>::translate_rotate_scale(trs_.translation_, trs_.euler_, vec3<T>(trs_.uniform_scale_))
            : mat4x3<T>::translate_rotate_scale(trs_.translation_, trs_.quat_, vec3<T>(trs_.uniform_scale_));
        case scale_type::nonuniform:
          return (rotation_type_ == rotation_type::euler)
            ? mat4x3<T>::translate_rotate_scale(trs_.translation_, trs_.euler_, trs_.nonuniform_scale_)
            : mat4x3<T>::translate_rotate_scale(trs_.translation_, trs_.quat_, trs_.nonuniform_scale_);
      }
    case transform_type::affine:
      return affine_matrix_;

    case transform_type::general:
      return mat4x3<T>(general_matrix_);
  }
}

template transform<float>::operator mat4x3<float> () const;
template transform<double>::operator mat4x3<double> () const;

template<typename T>
transform<T>::operator mat4<T> () const {
  switch (type_) {
    default:
    case transform_type::identity:
      return mat4<T>::identity();

    case transform_type::trs:
      switch (scale_type_) {
        default:
        case scale_type::unit:
          return (rotation_type_ == rotation_type::euler)
            ? mat4<T>::translate_rotate(trs_.translation_, trs_.euler_)
            : mat4<T>::translate_rotate(trs_.translation_, trs_.quat_);
        case scale_type::uniform:
          return (rotation_type_ == rotation_type::euler)
            ? mat4<T>::translate_rotate_scale(trs_.translation_, trs_.euler_, vec3<T>(trs_.uniform_scale_))
            : mat4<T>::translate_rotate_scale(trs_.translation_, trs_.quat_, vec3<T>(trs_.uniform_scale_));
        case scale_type::nonuniform:
          return (rotation_type_ == rotation_type::euler)
            ? mat4<T>::translate_rotate_scale(trs_.translation_, trs_.euler_, trs_.nonuniform_scale_)
            : mat4<T>::translate_rotate_scale(trs_.translation_, trs_.quat_, trs_.nonuniform_scale_);
      }
    case transform_type::affine:
      return mat4<T>(affine_matrix_);

    case transform_type::general:
      return general_matrix_;
  }
}

template transform<float>::operator mat4<float> () const;
template transform<double>::operator mat4<double> () const;

template<typename T>
bool transform<T>::operator== (const transform& other) const {
  switch (std::max(type(), other.type())) {
    default:
    case transform_type::identity:
      return true;

    case transform_type::trs:
      if (extract_translation() != other.extract_translation() || extract_rotation() != other.extract_rotation()) return false;
      switch (std::max(scale_type(), other.scale_type())) {
        case scale_type::unit: return true;
        case scale_type::uniform: return extract_uniform_scale() == other.extract_uniform_scale();
        case scale_type::nonuniform: return extract_scale() == other.extract_scale();
      }
    case transform_type::affine:
      return mat4x3<T>(*this) == mat4x3<T>(other);

    case transform_type::general:
      return mat4<T>(*this) == mat4<T>(other);
  }
}

template bool transform<float>::operator== (const transform<float>& other) const;
template bool transform<double>::operator== (const transform<double>& other) const;

template<typename T>
vec3<T> transform<T>::extract_translation () const {
  switch (type_) {
    default: case transform_type::identity: return vec3<T>();
    case transform_type::trs: return trs_.translation_;
    case transform_type::affine: return affine_matrix_.c3;
    case transform_type::general: return vec3<T>(general_matrix_.c3);
  }
}

template vec3<float> transform<float>::extract_translation () const;
template vec3<double> transform<double>::extract_translation () const;

template<typename T>
quat<T> transform<T>::extract_rotation () const {
  switch (type_) {
    case transform_type::identity: return vkm::quat<T>();
    case transform_type::trs: return (rotation_type_ == rotation_type::euler) ? vkm::quat<T>(trs_.euler_) : trs_.quat_;
  }
  vec3<T> c0, c1, c2;
  if (type_ == transform_type::affine) {
    c0 = affine_matrix_.c0;
    c1 = affine_matrix_.c1;
    c2 = affine_matrix_.c2;
  } else {
    c0 = vec3<T>(general_matrix_.c0);
    c1 = vec3<T>(general_matrix_.c1);
    c2 = vec3<T>(general_matrix_.c2);
  }
  // https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
  c0 = normalize(c0);
  c1 = normalize(c1);
  c2 = normalize(c2);
  T t = c0.x + c1.y + c2.z;
  T r = std::sqrt(1 + t);
  T s = 1 / (2*r);
  return vkm::quat<T>((c1.z - c2.y) * s, (c2.x - c0.z) * s, (c0.y - c1.x) * s, r / 2);
}

template quat<float> transform<float>::extract_rotation () const;
template quat<double> transform<double>::extract_rotation () const;

template<typename T>
vec3<T> transform<T>::extract_scale () const {
  switch (type_) {
    default:
    case transform_type::identity:
      return vec3<T>(1);

    case transform_type::trs:
      switch (scale_type_) {
        default: case scale_type::unit: return vec3<T>(1);
        case scale_type::uniform: return vec3<T>(trs_.uniform_scale_);
        case scale_type::nonuniform: return trs_.nonuniform_scale_;
      }
    case transform_type::affine:
      return vec3<T>(length(affine_matrix_.c0), length(affine_matrix_.c1), length(affine_matrix_.c2));

    case transform_type::general:
      return vec3<T>(length(vec3<T>(general_matrix_.c0)), length(vec3<T>(general_matrix_.c1)),
        length(vec3<T>(general_matrix_.c2)));
  }
}

template vec3<float> transform<float>::extract_scale () const;
template vec3<double> transform<double>::extract_scale () const;

template<typename T>
T transform<T>::extract_uniform_scale () const {
  switch (type_) {
    default:
    case transform_type::identity:
      return 1;

    case transform_type::trs:
      switch (scale_type_) {
        default: case scale_type::unit:
          return 1;

        case scale_type::uniform:
          return trs_.uniform_scale_;

        case scale_type::nonuniform:
          return std::cbrt(trs_.nonuniform_scale_.x * trs_.nonuniform_scale_.y * trs_.nonuniform_scale_.z);
      }
    case transform_type::affine:
      return std::cbrt(dot(cross(affine_matrix_.c0, affine_matrix_.c1), affine_matrix_.c2));

    case transform_type::general:
      return std::cbrt(dot(cross(vec3<T>(general_matrix_.c0), vec3<T>(general_matrix_.c1)), vec3<T>(general_matrix_.c2)));
  }
}

template float transform<float>::extract_uniform_scale () const;
template double transform<double>::extract_uniform_scale () const;

template<typename T>
vec3<T> transform<T>::transform_vector (const vec3<T>& v) const {
  switch (type_) {
    default:
    case transform_type::identity:
      return v;

    case transform_type::trs: {
      vec3<T> scaled = v;
      switch (scale_type_) {
        case scale_type::uniform:
          scaled *= trs_.uniform_scale_;
          break;

        case scale_type::nonuniform:
          scaled *= trs_.nonuniform_scale_;
          break;
      }
      switch (rotation_type_) {
        default: case rotation_type::euler: return trs_.euler_ * scaled;
        case rotation_type::quat: return trs_.quat_ * scaled;
      }
    }
    case transform_type::affine:
      return affine_matrix_.transform_vector(v);

    case transform_type::general:
      return general_matrix_.transform_vector(v);
  }
}

template vec3<float> transform<float>::transform_vector (const vec3<float>& v) const;
template vec3<double> transform<double>::transform_vector (const vec3<double>& v) const;

template<typename T>
vec3<T> transform<T>::transform_normal (const vec3<T>& n) const {
  switch (type_) {
    default:
    case transform_type::identity:
      return n;

    case transform_type::trs: {
      vec3<T> scaled = n;
      switch (scale_type_) {
        case scale_type::nonuniform:
          scaled /= trs_.nonuniform_scale_;
          break;
      }
      switch (rotation_type_) {
        default: case rotation_type::euler: return normalize(trs_.euler_ * scaled);
        case rotation_type::quat: return normalize(trs_.quat_ * scaled);
      }
    }
    case transform_type::affine:
      return affine_matrix_.transform_normal(n);

    case transform_type::general:
      return general_matrix_.transform_normal(n);
  }
}

template vec3<float> transform<float>::transform_normal (const vec3<float>& n) const;
template vec3<double> transform<double>::transform_normal (const vec3<double>& n) const;

template<typename T>
vec3<T> transform<T>::operator* (const vec3<T>& v) const {
  switch (type_) {
    default:
    case transform_type::identity:
      return v;

    case transform_type::trs: {
      vec3<T> scaled_rotated = v;
      switch (scale_type_) {
        case scale_type::uniform:
          scaled_rotated *= trs_.uniform_scale_;
          break;

        case scale_type::nonuniform:
          scaled_rotated *= trs_.nonuniform_scale_;
          break;
      }
      switch (rotation_type_) {
        case rotation_type::euler:
          scaled_rotated = trs_.euler_ * scaled_rotated;
          break;

        case rotation_type::quat:
          scaled_rotated = trs_.quat_ * scaled_rotated;
          break;
      }
      return scaled_rotated + trs_.translation_;
    }
    case transform_type::affine:
      return affine_matrix_ * v;

    case transform_type::general:
      return general_matrix_ * v;
  }
}

template vec3<float> transform<float>::operator* (const vec3<float>& v) const;
template vec3<double> transform<double>::operator* (const vec3<double>& v) const;

template<typename T>
vec4<T> transform<T>::operator* (const vec4<T>& v) const {
  switch (type_) {
    default:
    case transform_type::identity:
      return v;

    case transform_type::trs: {
      vec3<T> scaled_rotated = vec3<T>(v);
      switch (scale_type_) {
        case scale_type::uniform:
          scaled_rotated *= trs_.uniform_scale_;
          break;

        case scale_type::nonuniform:
          scaled_rotated *= trs_.nonuniform_scale_;
          break;
      }
      switch (rotation_type_) {
        case rotation_type::euler:
          scaled_rotated = trs_.euler_ * scaled_rotated;
          break;

        case rotation_type::quat:
          scaled_rotated = trs_.quat_ * scaled_rotated;
          break;
      }
      return vec4<T>(scaled_rotated + v.w * trs_.translation_, v.w);
    }
    case transform_type::affine:
      return affine_matrix_ * v;

    case transform_type::general:
      return general_matrix_ * v;
  }
}

template vec4<float> transform<float>::operator* (const vec4<float>& v) const;
template vec4<double> transform<double>::operator* (const vec4<double>& v) const;

template<typename T>
transform<T> transform<T>::operator* (const transform<T>& other) const {
  switch (std::max(type(), other.type())) {
    default:
    case transform_type::identity:
      return transform();

    case transform_type::trs:
      switch (std::max(scale_type(), other.scale_type())) {
        default:
        case scale_type::unit: {
          vkm::quat<T> rotation = extract_rotation();
          return transform(
            extract_translation() + rotation * other.extract_translation(),
            rotation * other.extract_rotation());
        }
        case scale_type::uniform: {
          vkm::quat<T> rotation = extract_rotation();
          T scale = extract_uniform_scale();
          return transform(
            extract_translation() + rotation * other.extract_translation() * scale,
            rotation * other.extract_rotation(),
            scale * other.extract_uniform_scale());
        }
        case scale_type::nonuniform:
          return transform(mat4x3<T>(*this) * mat4x3<T>(other));
      }
    case transform_type::affine:
      return transform(mat4x3<T>(*this) * mat4x3<T>(other));

    case transform_type::general:
      return transform(mat4<T>(*this) * mat4<T>(other));
  }
}

template transform<float> transform<float>::operator* (const transform<float>& other) const;
template transform<double> transform<double>::operator* (const transform<double>& other) const;

template<typename T>
void transform<T>::set_type_ (transform_type type) {
  if (type_ == type) return;
  switch (type) {
    case transform_type::trs: {
      vec3<T> translation = extract_translation();
      vkm::quat<T> rotation = extract_rotation();
      vec3<T> scale = extract_scale();
      trs_.translation_ = translation;
      trs_.quat_ = rotation;
      rotation_type_ = rotation_type::quat;
      if (scale.x == scale.y && scale.y == scale.z) {
        if (scale.x == 1) scale_type_ = scale_type::unit;
        else {
          trs_.uniform_scale_ = scale.x;
          scale_type_ = scale_type::uniform;
        }
      } else {
        trs_.nonuniform_scale_ = scale;
        scale_type_ = scale_type::nonuniform;
      }
      break;
    }
    case transform_type::affine:
      affine_matrix_ = mat4x3<T>(*this);
      break;

    case transform_type::general:
      general_matrix_ = mat4<T>(*this);
      break;
  }
  type_ = type;
}

template void transform<float>::set_type_ (transform_type type);
template void transform<double>::set_type_ (transform_type type);

template<typename T>
void transform<T>::set_rotation_type_ (vkm::rotation_type type) {
  set_type_(transform_type::trs);
  if (rotation_type_ == type) return;
  switch (type) {
    case rotation_type::euler:
      trs_.euler_ = vkm::euler<T>(trs_.quat_);
      break;

    case rotation_type::quat:
      trs_.quat_ = vkm::quat<T>(trs_.euler_);
      break;
  }
  rotation_type_ = type;
}

template void transform<float>::set_rotation_type_ (vkm::rotation_type type);
template void transform<double>::set_rotation_type_ (vkm::rotation_type type);

template<typename T>
void transform<T>::set_scale_type_ (vkm::scale_type type) {
  set_type_(transform_type::trs);
  if (scale_type_ == type) return;
  switch (type) {
    case scale_type::uniform:
      trs_.uniform_scale_ = (scale_type_ == scale_type::unit)
        ? 1
        : std::cbrt(trs_.nonuniform_scale_.x * trs_.nonuniform_scale_.y * trs_.nonuniform_scale_.z);
      break;

    case scale_type::nonuniform:
      trs_.nonuniform_scale_ = vec3<T>(scale_type_ == scale_type::unit ? 1 : trs_.uniform_scale_);
      break;
  }
  scale_type_ = type;
}

template void transform<float>::set_scale_type_ (vkm::scale_type type);
template void transform<double>::set_scale_type_ (vkm::scale_type type);

template<typename T>
transform<T> inverse (const transform<T>& transform) {
  switch (transform.type()) {
    default:
    case transform_type::identity:
      return vkm::transform<T>();

    case transform_type::trs:
      switch (transform.scale_type()) {
        default:
        case scale_type::unit: {
          quat<T> rotation = inverse(transform.extract_rotation());
          return vkm::transform<T>(rotation * -transform.translation(), rotation);
        }
        case scale_type::uniform: {
          quat<T> rotation = inverse(transform.extract_rotation());
          T scale = 1 / transform.uniform_scale();
          return vkm::transform<T>(rotation * transform.translation() * -scale, rotation, scale);
        }
        case scale_type::nonuniform:
          return vkm::transform<T>(inverse(mat4x3<T>(transform)));
      }
    case transform_type::affine:
      return vkm::transform<T>(inverse(transform.affine_matrix()));

    case transform_type::general:
      return vkm::transform<T>(inverse(transform.general_matrix()));
  }
}

template transform<float> inverse (const transform<float>& transform);
template transform<double> inverse (const transform<double>& transform);

template<typename T>
transform<T> mix (const transform<T>& x, const transform<T>& y, T a) {
  switch (std::max(x.type(), y.type())) {
    default:
    case transform_type::identity:
      return transform<T>();

    case transform_type::trs:
      switch (std::max(x.scale_type(), y.scale_type())) {
        default:
        case scale_type::unit:
          return transform<T>(
            mix(x.extract_translation(), y.extract_translation(), a),
            mix(x.extract_rotation(), y.extract_rotation(), a));

        case scale_type::uniform:
          return transform<T>(
            mix(x.extract_translation(), y.extract_translation(), a),
            mix(x.extract_rotation(), y.extract_rotation(), a),
            mix(x.extract_uniform_scale(), y.extract_uniform_scale(), a));

        case scale_type::nonuniform:
          return transform<T>(
            mix(x.extract_translation(), y.extract_translation(), a),
            mix(x.extract_rotation(), y.extract_rotation(), a),
            mix(x.extract_scale(), y.extract_scale(), a));
      }
    case transform_type::affine:
      return transform<T>(mix(mat4x3<T>(x), mat4x3<T>(y), a));

    case transform_type::general:
      return transform<T>(mix(mat4<T>(x), mat4<T>(y), a));
  }
}

template transform<float> mix (const transform<float>& x, const transform<float>& y, float a);
template transform<double> mix (const transform<double>& x, const transform<double>& y, double a);

template<typename T>
std::ostream& operator<< (std::ostream& out, const transform<T>& transform) {
  out << '(';
  switch (transform.type()) {
    case transform_type::trs:
      out << transform.translation() << ", ";
      switch (transform.rotation_type()) {
        case rotation_type::euler:
          out << transform.euler();
          break;

        case rotation_type::quat:
          out << transform.quat();
          break;
      }
      switch (transform.scale_type()) {
        case scale_type::uniform:
          out << ", " << transform.uniform_scale();
          break;

        case scale_type::nonuniform:
          out << ", " << transform.nonuniform_scale();
          break;
      }
      break;

    case transform_type::affine:
      out << transform.affine_matrix();
      break;

    case transform_type::general:
      out << transform.general_matrix();
      break;
  }
  return out << ')';
}

template std::ostream& operator<< (std::ostream& out, const transform<float>& transform);
template std::ostream& operator<< (std::ostream& out, const transform<double>& transform);

}
