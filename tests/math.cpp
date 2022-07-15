#include "test.hpp"

namespace vkm {

TEST_CASE("scalars can be manipulated", "[scalar]") {
  REQUIRE_THAT(radians(0.0f), ApproxEquals(0.0f));
  REQUIRE_THAT(radians(180.0f), ApproxEquals(float(M_PI)));
  REQUIRE_THAT(radians(0.0), ApproxEquals(0.0));
  REQUIRE_THAT(radians(180.0), ApproxEquals(M_PI));

  REQUIRE_THAT(mix(0.0f, 10.0f, 0.0f), ApproxEquals(0.0f));
  REQUIRE_THAT(mix(0.0f, 10.0f, 1.0f), ApproxEquals(10.0f));
  REQUIRE_THAT(mix(0.0f, 10.0f, 0.5f), ApproxEquals(5.0f));

  REQUIRE_THAT(mix(0.0, 10.0, 0.0), ApproxEquals(0.0));
  REQUIRE_THAT(mix(0.0, 10.0, 1.0), ApproxEquals(10.0));
  REQUIRE_THAT(mix(0.0, 10.0, 0.5), ApproxEquals(5.0));

  REQUIRE(is_within(0.5, 0.0, 1.0));
  REQUIRE_FALSE(is_within(0.5, 0.0, 0.25));
}

TEST_CASE("vec2s can be manipulated", "[vec2]") {
  vec2f valuef;
  REQUIRE_THAT(valuef, ApproxEquals(vec2f(0, 0)));
  REQUIRE_THAT(valuef += vec2f(1, 0), ApproxEquals(vec2f(1, 0)));
  REQUIRE_THAT(valuef *= 0.5f, ApproxEquals(vec2f(0.5, 0)));
  REQUIRE_THAT(valuef = valuef + vec2f(0, 1), ApproxEquals(vec2f(0.5, 1)));

  vec2d valued;
  REQUIRE_THAT(valued, ApproxEquals(vec2d(0, 0)));
  REQUIRE_THAT(valued += vec2d(1, 0), ApproxEquals(vec2d(1, 0)));
  REQUIRE_THAT(valued *= 0.5, ApproxEquals(vec2d(0.5, 0)));
  REQUIRE_THAT(valued = valued + vec2d(0, 1), ApproxEquals(vec2d(0.5, 1)));

  REQUIRE_THAT(dot(vec2f(0, 1), vec2f(1, 0)), ApproxEquals(0.0f));
  REQUIRE_THAT(dot(vec2f(1, 1), vec2f(1, 1)), ApproxEquals(2.0f));
  REQUIRE_THAT(length(vec2f(0, 0)), ApproxEquals(0.0f));
  REQUIRE_THAT(length(vec2f(1, 0)), ApproxEquals(1.0f));
  REQUIRE_THAT(length(vec2f(1, 1)), ApproxEquals(std::sqrt(2.0f)));
  REQUIRE_THAT(distance(vec2f(1, 1), vec2f(2, 2)), ApproxEquals(std::sqrt(2.0f)));
  REQUIRE_THAT(normalize(vec2f(2, 0)), ApproxEquals(vec2f(1, 0)));
  REQUIRE_THAT(mix(vec2f(0, 0), vec2f(2, 2), 0.5f), ApproxEquals(vec2f(1, 1)));
}

TEST_CASE("vec3s can be manipulated", "[vec3]") {
  vec3f valuef;
  REQUIRE_THAT(valuef, ApproxEquals(vec3f(0, 0, 0)));
  REQUIRE_THAT(valuef += vec3f(1, 0, 0), ApproxEquals(vec3f(1, 0, 0)));
  REQUIRE_THAT(valuef *= 0.5f, ApproxEquals(vec3f(0.5, 0, 0)));
  REQUIRE_THAT(valuef = valuef + vec3f(0, 1, 1), ApproxEquals(vec3f(0.5, 1, 1)));

  vec3d valued;
  REQUIRE_THAT(valued, ApproxEquals(vec3d(0, 0, 0)));
  REQUIRE_THAT(valued += vec3d(1, 0, 0), ApproxEquals(vec3d(1, 0, 0)));
  REQUIRE_THAT(valued *= 0.5, ApproxEquals(vec3d(0.5, 0, 0)));
  REQUIRE_THAT(valued = valued + vec3d(0, 1, 1), ApproxEquals(vec3d(0.5, 1, 1)));

  REQUIRE_THAT(dot(vec3f(0, 1, 0), vec3f(1, 0, 0)), ApproxEquals(0.0f));
  REQUIRE_THAT(dot(vec3f(1, 1, 1), vec3f(1, 1, 1)), ApproxEquals(3.0f));
  REQUIRE_THAT(length(vec3f(0, 0, 0)), ApproxEquals(0.0f));
  REQUIRE_THAT(length(vec3f(1, 0, 0)), ApproxEquals(1.0f));
  REQUIRE_THAT(length(vec3f(1, 1, 1)), ApproxEquals(std::sqrt(3.0f)));
  REQUIRE_THAT(distance(vec3f(1, 1, 1), vec3f(2, 2, 2)), ApproxEquals(std::sqrt(3.0f)));
  REQUIRE_THAT(normalize(vec3f(2, 0, 0)), ApproxEquals(vec3f(1, 0, 0)));
  REQUIRE_THAT(mix(vec3f(0, 0, 0), vec3f(2, 2, 2), 0.5f), ApproxEquals(vec3f(1, 1, 1)));

  REQUIRE_THAT(cross(vec3f(1, 0, 0), vec3f(0, 1, 0)), ApproxEquals(vec3f(0, 0, 1)));
}

TEST_CASE("vec4s can be manipulated", "[vec4]") {
  vec4f valuef;
  REQUIRE_THAT(valuef, ApproxEquals(vec4f(0, 0, 0, 0)));
  REQUIRE_THAT(valuef += vec4f(1, 0, 0, 0), ApproxEquals(vec4f(1, 0, 0, 0)));
  REQUIRE_THAT(valuef *= 0.5f, ApproxEquals(vec4f(0.5, 0, 0, 0)));
  REQUIRE_THAT(valuef = valuef + vec4f(0, 1, 1, 1), ApproxEquals(vec4f(0.5, 1, 1, 1)));

  vec4d valued;
  REQUIRE_THAT(valued, ApproxEquals(vec4d(0, 0, 0, 0)));
  REQUIRE_THAT(valued += vec4d(1, 0, 0, 0), ApproxEquals(vec4d(1, 0, 0, 0)));
  REQUIRE_THAT(valued *= 0.5, ApproxEquals(vec4d(0.5, 0, 0, 0)));
  REQUIRE_THAT(valued = valued + vec4d(0, 1, 1, 1), ApproxEquals(vec4d(0.5, 1, 1, 1)));

  REQUIRE_THAT(dot(vec4f(0, 1, 0, 0), vec4f(1, 0, 0, 0)), ApproxEquals(0.0f));
  REQUIRE_THAT(dot(vec4f(1, 1, 1, 1), vec4f(1, 1, 1, 1)), ApproxEquals(4.0f));
  REQUIRE_THAT(length(vec4f(0, 0, 0, 0)), ApproxEquals(0.0f));
  REQUIRE_THAT(length(vec4f(1, 0, 0, 0)), ApproxEquals(1.0f));
  REQUIRE_THAT(length(vec4f(1, 1, 1, 1)), ApproxEquals(std::sqrt(4.0f)));
  REQUIRE_THAT(distance(vec4f(1, 1, 1, 1), vec4f(2, 2, 2, 2)), ApproxEquals(std::sqrt(4.0f)));
  REQUIRE_THAT(normalize(vec4f(2, 0, 0, 0)), ApproxEquals(vec4f(1, 0, 0, 0)));
  REQUIRE_THAT(mix(vec4f(0, 0, 0, 0), vec4f(2, 2, 2, 2), 0.5f), ApproxEquals(vec4f(1, 1, 1, 1)));
}

TEST_CASE("eulers can be manipulated", "[euler]") {
  eulerf valuef;
  REQUIRE_THAT(valuef, ApproxEquals(eulerf(0, 0, 0)));
  REQUIRE_THAT(valuef += eulerf(1, 0, 0), ApproxEquals(eulerf(1, 0, 0)));
  REQUIRE_THAT(valuef *= 0.5f, ApproxEquals(eulerf(0.5, 0, 0)));
  REQUIRE_THAT(valuef = valuef + eulerf(0, 1, 1), ApproxEquals(eulerf(0.5, 1, 1)));

  REQUIRE_THAT(eulerf(M_PI_2, M_PI_2, M_PI_2) * vec3f(1, 0, 0), ApproxEquals(vec3f(0, 0, -1)));
  REQUIRE_THAT(eulerf(M_PI_2, M_PI_2, M_PI_2) * vec3f(0, 1, 0), ApproxEquals(vec3f(0, 1, 0)));
}

TEST_CASE("quats can be manipulated", "[quat]") {
  REQUIRE_THAT(quatf(), ApproxEquals(quatf(0, 0, 0, 1)));
  REQUIRE_THAT(quatf(0, vec3f(1, 0, 0)), ApproxEquals(quatf()));
  REQUIRE_THAT(quatf(eulerf(0, 0, 0)), ApproxEquals(quatf()));
  REQUIRE_THAT(quatf(M_PI_2, vec3f(1, 0, 0)), ApproxEquals(quatf(eulerf(M_PI_2, 0, 0))));

  REQUIRE_THAT(eulerf(quatf(eulerf(M_PI_2, 0, 0))), ApproxEquals(eulerf(M_PI_2, 0, 0)));
  REQUIRE_THAT(quatf(eulerf(M_PI_2, 0, 0)) * vec3f(0, 1, 0), ApproxEquals(vec3f(0, 0, 1)));
  REQUIRE_THAT(quatf(eulerf(0, M_PI_2, 0)) * quatf(eulerf(M_PI_2, 0, 0)), ApproxEquals(quatf(eulerf(M_PI_2, M_PI_2, 0))));

  REQUIRE_THAT(inverse(quatf()), ApproxEquals(quatf()));
  REQUIRE_THAT(inverse(quatf(eulerf(M_PI_2, 0, 0))), ApproxEquals(quatf(eulerf(-M_PI_2, 0, 0))));

  REQUIRE_THAT(mix(quatf(), quatf(), 0.0f), ApproxEquals(quatf()));
  REQUIRE_THAT(mix(quatf(eulerf()), quatf(eulerf(M_PI_2, 0, 0)), 0.0f), ApproxEquals(quatf()));
  REQUIRE_THAT(mix(quatf(eulerf()), quatf(eulerf(M_PI_2, 0, 0)), 1.0f), ApproxEquals(quatf(eulerf(M_PI_2, 0, 0))));
  REQUIRE_THAT(mix(quatf(eulerf()), quatf(eulerf(M_PI_2, 0, 0)), 0.5f), ApproxEquals(quatf(eulerf(M_PI_4, 0, 0))));
}

TEST_CASE("mat2s can be manipulated", "[mat2]") {
  REQUIRE_THAT(mat2f::identity(), ApproxEquals(mat2f(1)));
  REQUIRE_THAT(mat2f::identity() * vec2f(1, 2), ApproxEquals(vec2f(1, 2)));
  REQUIRE_THAT(mat2f(2) * vec2f(1, 2), ApproxEquals(vec2f(2, 4)));
  REQUIRE_THAT(mat2f(2) * mat2f(2), ApproxEquals(mat2f(4)));
  REQUIRE_THAT(inverse(mat2f(2)), ApproxEquals(mat2f(0.5)));
}

TEST_CASE("mat3s can be manipulated", "[mat3]") {
  REQUIRE_THAT(mat3f::identity(), ApproxEquals(mat3f(1)));
  REQUIRE_THAT(mat3f::identity() * vec2f(1, 2), ApproxEquals(vec2f(1, 2)));
  REQUIRE_THAT(mat3f::translate(vec2f(2, 4)) * vec2f(2, 2), ApproxEquals(vec2f(4, 6)));
  REQUIRE_THAT(mat3f::rotate(M_PI_2) * vec2f(1, 0), ApproxEquals(vec2f(0, 1)));
  REQUIRE_THAT(mat3f::scale(vec2f(2, 4)) * vec2f(2, 2), ApproxEquals(vec2f(4, 8)));
  REQUIRE_THAT(mat3f::translate_rotate(vec2f(2, 4), M_PI_2) * vec2f(1, 0), ApproxEquals(vec2f(2, 5)));
  REQUIRE_THAT(mat3f::translate_rotate_scale(vec2f(2, 4), M_PI_2, vec2f(2, 4)) * vec2f(1, 0), ApproxEquals(vec2f(2, 6)));

  REQUIRE_THAT(mat3f::translate_rotate(vec2f(2, 4), M_PI_2),
    ApproxEquals(mat3f::translate(vec2f(2, 4)) * mat3f::rotate(M_PI_2)));
  REQUIRE_THAT(mat3f::translate_rotate_scale(vec2f(2, 4), M_PI_2, vec2f(2, 4)),
    ApproxEquals(mat3f::translate(vec2f(2, 4)) * mat3f::rotate(M_PI_2) * mat3f::scale(vec2f(2, 4))));

  REQUIRE_THAT(inverse(mat3f::translate(vec2f(2, 4))), ApproxEquals(mat3f::translate(vec2f(-2, -4))));
  REQUIRE_THAT(inverse(mat3f::rotate(M_PI_2)), ApproxEquals(mat3f::rotate(-M_PI_2)));
  REQUIRE_THAT(inverse(mat3f::scale(vec2f(2, 4))), ApproxEquals(mat3f::scale(vec2f(0.5, 0.25))));
}

TEST_CASE("mat4x3s can be manipulated", "[mat4x3]") {
  REQUIRE_THAT(mat4x3f::identity(), ApproxEquals(mat4x3f(1)));
  REQUIRE_THAT(mat4x3f::identity() * vec3f(1, 2, 3), ApproxEquals(vec3f(1, 2, 3)));
  REQUIRE_THAT(mat4x3f::translate(vec3f(1, 2, 3)) * vec3f(1, 2, 3), ApproxEquals(vec3f(2, 4, 6)));
  REQUIRE_THAT(mat4x3f::rotate(M_PI_2, vec3f(1, 0, 0)) * vec3f(0, 1, 0), ApproxEquals(vec3f(0, 0, 1)));
  REQUIRE_THAT(mat4x3f::rotate(eulerf(M_PI_2, M_PI_2, M_PI_2)) * vec3f(1, 0, 0), ApproxEquals(vec3f(0, 0, -1)));
  REQUIRE_THAT(mat4x3f::rotate(quatf(M_PI_2, vec3f(0, 1, 0))) * vec3f(1, 0, 0), ApproxEquals(vec3f(0, 0, -1)));
  REQUIRE_THAT(mat4x3f::scale(vec3f(2, 3, 4)) * vec3f(2, 3, 4), ApproxEquals(vec3f(4, 9, 16)));
  REQUIRE_THAT(mat4x3f::translate_rotate(vec3f(1, 2, 3), eulerf(M_PI_2, M_PI_2, M_PI_2)) * vec3f(1, 0, 0),
    ApproxEquals(vec3f(1, 2, 2)));
  REQUIRE_THAT(mat4x3f::translate_rotate(vec3f(1, 2, 3), quatf(M_PI_2, vec3f(0, 1, 0))) * vec3f(1, 0, 0),
    ApproxEquals(vec3f(1, 2, 2)));
  REQUIRE_THAT(mat4x3f::translate_rotate_scale(vec3f(1, 2, 3), eulerf(M_PI_2, M_PI_2, M_PI_2), vec3f(2, 3, 4)) * vec3f(1, 0, 0),
    ApproxEquals(vec3f(1, 2, 1)));
  REQUIRE_THAT(mat4x3f::translate_rotate_scale(vec3f(1, 2, 3), quatf(M_PI_2, vec3f(0, 1, 0)), vec3f(2, 3, 4)) * vec3f(1, 0, 0),
    ApproxEquals(vec3f(1, 2, 1)));

  REQUIRE_THAT(mat4x3f::translate_rotate(vec3f(1, 2, 3), eulerf(M_PI_2, M_PI_2, M_PI_2)) * vec4f(1, 0, 0, 1),
    ApproxEquals(vec4f(1, 2, 2, 1)));
  REQUIRE_THAT(mat4x3f::translate_rotate(vec3f(1, 2, 3), eulerf(M_PI_2, M_PI_2, M_PI_2)) * vec4f(1, 0, 0, 0),
    ApproxEquals(vec4f(0, 0, -1, 0)));

  REQUIRE_THAT(mat4x3f::translate(vec3f(1, 2, 3)) * mat4x3f::rotate(eulerf(M_PI_2, M_PI_2, M_PI_2)),
    ApproxEquals(mat4x3f::translate_rotate(vec3f(1, 2, 3), eulerf(M_PI_2, M_PI_2, M_PI_2))));

  REQUIRE_THAT(inverse(mat4x3f::translate(vec3f(1, 2, 3))), ApproxEquals(mat4x3f::translate(vec3f(-1, -2, -3))));
  REQUIRE_THAT(inverse(mat4x3f::rotate(M_PI_2, vec3f(1, 0, 0))), ApproxEquals(mat4x3f::rotate(-M_PI_2, vec3f(1, 0, 0))));
  REQUIRE_THAT(inverse(mat4x3f::scale(vec3f(1, 2, 3))), ApproxEquals(mat4x3f::scale(vec3f(1, 1/2.0, 1/3.0))));

  REQUIRE_THAT(mat4x3f::reflect(planef(0, 1, 0, -1)) * vec3f(1, 2, 3), ApproxEquals(vec3f(1, 0, 3)));
  REQUIRE_THAT(mat4x3f::reflect(planef(1, 0, 0, -1)) * vec3f(5, 2, 3), ApproxEquals(vec3f(-3, 2, 3)));
}

TEST_CASE("mat4s can be manipulated", "[mat4]") {
  REQUIRE_THAT(mat4f::identity(), ApproxEquals(mat4f(1)));
  REQUIRE_THAT(mat4f::identity() * vec3f(1, 2, 3), ApproxEquals(vec3f(1, 2, 3)));
  REQUIRE_THAT(mat4f::translate(vec3f(1, 2, 3)) * vec3f(1, 2, 3), ApproxEquals(vec3f(2, 4, 6)));
  REQUIRE_THAT(mat4f::rotate(M_PI_2, vec3f(1, 0, 0)) * vec3f(0, 1, 0), ApproxEquals(vec3f(0, 0, 1)));
  REQUIRE_THAT(mat4f::rotate(eulerf(M_PI_2, M_PI_2, M_PI_2)) * vec3f(1, 0, 0), ApproxEquals(vec3f(0, 0, -1)));
  REQUIRE_THAT(mat4f::rotate(quatf(M_PI_2, vec3f(0, 1, 0))) * vec3f(1, 0, 0), ApproxEquals(vec3f(0, 0, -1)));
  REQUIRE_THAT(mat4f::scale(vec3f(2, 3, 4)) * vec3f(2, 3, 4), ApproxEquals(vec3f(4, 9, 16)));
  REQUIRE_THAT(mat4f::translate_rotate(vec3f(1, 2, 3), eulerf(M_PI_2, M_PI_2, M_PI_2)) * vec3f(1, 0, 0),
    ApproxEquals(vec3f(1, 2, 2)));
  REQUIRE_THAT(mat4f::translate_rotate(vec3f(1, 2, 3), quatf(M_PI_2, vec3f(0, 1, 0))) * vec3f(1, 0, 0),
    ApproxEquals(vec3f(1, 2, 2)));
  REQUIRE_THAT(mat4f::translate_rotate_scale(vec3f(1, 2, 3), eulerf(M_PI_2, M_PI_2, M_PI_2), vec3f(2, 3, 4)) * vec3f(1, 0, 0),
    ApproxEquals(vec3f(1, 2, 1)));
  REQUIRE_THAT(mat4f::translate_rotate_scale(vec3f(1, 2, 3), quatf(M_PI_2, vec3f(0, 1, 0)), vec3f(2, 3, 4)) * vec3f(1, 0, 0),
    ApproxEquals(vec3f(1, 2, 1)));

  REQUIRE_THAT(mat4f::translate_rotate(vec3f(1, 2, 3), eulerf(M_PI_2, M_PI_2, M_PI_2)) * vec4f(1, 0, 0, 1),
    ApproxEquals(vec4f(1, 2, 2, 1)));
  REQUIRE_THAT(mat4f::translate_rotate(vec3f(1, 2, 3), eulerf(M_PI_2, M_PI_2, M_PI_2)) * vec4f(1, 0, 0, 0),
    ApproxEquals(vec4f(0, 0, -1, 0)));

  REQUIRE_THAT(mat4f::translate(vec3f(1, 2, 3)) * mat4f::rotate(eulerf(M_PI_2, M_PI_2, M_PI_2)),
    ApproxEquals(mat4f::translate_rotate(vec3f(1, 2, 3), eulerf(M_PI_2, M_PI_2, M_PI_2))));

  REQUIRE_THAT(inverse(mat4f::translate(vec3f(1, 2, 3))), ApproxEquals(mat4f::translate(vec3f(-1, -2, -3))));
  REQUIRE_THAT(inverse(mat4f::rotate(M_PI_2, vec3f(1, 0, 0))), ApproxEquals(mat4f::rotate(-M_PI_2, vec3f(1, 0, 0))));
  REQUIRE_THAT(inverse(mat4f::scale(vec3f(1, 2, 3))), ApproxEquals(mat4f::scale(vec3f(1, 1/2.0, 1/3.0))));

  REQUIRE_THAT(mat4f::ortho(-2, 2, -2, 2, 1, 4) * vec4f(2, 2, -4, 1), ApproxEquals(vec4f(1, 1, 1, 1)));
  REQUIRE_THAT(mat4f::frustum(-2, 2, -2, 2, 1, 4) * vec4f(2, 2, -4, 1), ApproxEquals(vec4f(1, 1, 4, 4)));
  REQUIRE_THAT(mat4f::perspective(M_PI_2, 1, 1, 4), ApproxEquals(mat4f::frustum(-1, 1, -1, 1, 1, 4)));
}

TEST_CASE("transforms can be manipulated", "[transform]") {
  REQUIRE_THAT(transformf() * vec3f(1, 2, 3), ApproxEquals(vec3f(1, 2, 3)));
  REQUIRE_THAT(mat4f(transformf()), ApproxEquals(mat4f(1)));
  REQUIRE_THAT(transformf(vec3f(1, 2, 3), eulerf(M_PI_2, 0, 0)) * vec3f(1, 2, 3), ApproxEquals(vec3f(2, -1, 5)));
  REQUIRE_THAT(transformf(vec3f(1, 2, 3), quatf(M_PI_2, vec3f(0, 1, 0))) * vec3f(1, 2, 3), ApproxEquals(vec3f(4, 4, 2)));
  REQUIRE_THAT(transformf(vec3f(1, 2, 3), eulerf(M_PI_2, 0, 0), 2) * vec3f(1, 2, 3),
    ApproxEquals(vec3f(3, -4, 7)));
  REQUIRE_THAT(transformf(vec3f(1, 2, 3), quatf(M_PI_2, vec3f(0, 1, 0)), 2) * vec3f(1, 2, 3),
    ApproxEquals(vec3f(7, 6, 1)));
  REQUIRE_THAT(transformf(vec3f(1, 2, 3), eulerf(M_PI_2, 0, 0), vec3f(2, 3, 4)) * vec3f(1, 2, 3),
    ApproxEquals(vec3f(3, -10, 9)));
  REQUIRE_THAT(transformf(vec3f(1, 2, 3), quatf(M_PI_2, vec3f(0, 1, 0)), vec3f(2, 3, 4)) * vec3f(1, 2, 3),
    ApproxEquals(vec3f(13, 8, 1)));
  REQUIRE_THAT(transformf(
    mat4x3f::translate_rotate_scale(vec3f(1, 2, 3), quatf(M_PI_2, vec3f(0, 1, 0)), vec3f(2, 3, 4))) * vec3f(1, 2, 3),
    ApproxEquals(vec3f(13, 8, 1)));
  REQUIRE_THAT(transformf(
    mat4f::translate_rotate_scale(vec3f(1, 2, 3), quatf(M_PI_2, vec3f(0, 1, 0)), vec3f(2, 3, 4))) * vec3f(1, 2, 3),
    ApproxEquals(vec3f(13, 8, 1)));
  REQUIRE_THAT(mat4x3f(transformf(vec3f(1, 2, 3), eulerf(M_PI_2, 0, 0), 2)),
    ApproxEquals(mat4x3f::translate_rotate_scale(vec3f(1, 2, 3), eulerf(M_PI_2, 0, 0), vec3f(2))));
  REQUIRE_THAT(mat4f(transformf(vec3f(1, 2, 3), eulerf(M_PI_2, 0, 0), 2)),
    ApproxEquals(mat4f::translate_rotate_scale(vec3f(1, 2, 3), eulerf(M_PI_2, 0, 0), vec3f(2))));

  transformf trs(vec3f(1, 2, 3), eulerf(M_PI_2, 0, 0), 2);
  REQUIRE_THAT(trs.extract_translation(), ApproxEquals(vec3f(1, 2, 3)));
  REQUIRE_THAT(trs.extract_rotation(), ApproxEquals(quatf(M_PI_2, vec3f(1, 0, 0))));
  REQUIRE_THAT(trs.extract_uniform_scale(), ApproxEquals(2.0f));
  REQUIRE_THAT(trs.extract_scale(), ApproxEquals(vec3f(2, 2, 2)));

  trs.quat() = quatf(M_PI_2, vec3f(0, 1, 0));
  REQUIRE_THAT(trs.euler(), ApproxEquals(eulerf(0, M_PI_2, 0)));

  REQUIRE_THAT(transformf(vec3f(1, 2, 3), eulerf(0, M_PI_2, 0)) * transformf(vec3f(1, 2, 3), eulerf(0, 0, M_PI_2)),
    ApproxEquals(transformf(mat4x3f::translate_rotate(vec3f(1, 2, 3), eulerf(0, M_PI_2, 0)) *
      mat4x3f::translate_rotate(vec3f(1, 2, 3), eulerf(0, 0, M_PI_2)))));

  REQUIRE_THAT(inverse(transformf(vec3f(1, 2, 3), eulerf(0, M_PI_2, 0))),
    ApproxEquals(transformf(vec3f(3, -2, -1), eulerf(0, -M_PI_2, 0))));

  REQUIRE_THAT(mix(transformf(vec3f(0, 0, 0), eulerf(0, 0, 0)), transformf(vec3f(2, 2, 2), eulerf(0, M_PI_2, 0)), 0.5f),
    ApproxEquals(transformf(vec3f(1, 1, 1), eulerf(0, M_PI_4, 0))));
}

}
