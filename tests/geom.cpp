#include <vector>

#include "test.hpp"

namespace vkm {

TEST_CASE("rects can be manipulated", "[rect]") {
  REQUIRE(rectf({0, 0}, {1, 1}) == rectf({0, 0}, {1, 1}));
  REQUIRE_FALSE(rectf({0, 0}, {1, 1}) != rectf({0, 0}, {1, 1}));
  REQUIRE_FALSE(rectf({0, 0}, {1, 1}) == rectf({0, 0}, {2, 2}));
  REQUIRE(rectf({0, 0}, {1, 1}) != rectf({0, 0}, {2, 2}));

  REQUIRE(rectf({0, 0}, {1, 1}).intersects(rectf({0.5, 0.5}, {1.5, 1.5})));
  REQUIRE_FALSE(rectf({0, 0}, {1, 1}).intersects(rectf({1.5, 1.5}, {2.5, 2.5})));
  REQUIRE(rectf({0, 0}, {1, 1}).contains(rectf({0.5, 0.5}, {0.75, 0.75})));
  REQUIRE_FALSE(rectf({0, 0}, {1, 1}).contains(rectf({0.5, 0.5}, {1.75, 1.75})));
  REQUIRE_FALSE(rectf({0, 0}, {1, 1}).contains(rectf({1.5, 1.5}, {1.75, 1.75})));
}

TEST_CASE("boxes can be manipulated", "[box]") {
  REQUIRE(boxf({0, 0, 0}, {1, 1, 1}) == boxf({0, 0, 0}, {1, 1, 1}));
  REQUIRE_FALSE(boxf({0, 0, 0}, {1, 1, 1}) != boxf({0, 0, 0}, {1, 1, 1}));
  REQUIRE_FALSE(boxf({0, 0, 0}, {1, 1, 1}) == boxf({0, 0, 0}, {2, 2, 2}));
  REQUIRE(boxf({0, 0, 0}, {1, 1, 1}) != boxf({0, 0, 0}, {2, 2, 2}));

  REQUIRE(boxf({0, 0, 0}, {1, 1, 1}).intersects(boxf({0.5, 0.5, 0.5}, {1.5, 1.5, 1.5})));
  REQUIRE_FALSE(boxf({0, 0, 0}, {1, 1, 1}).intersects(boxf({1.5, 1.5, 1.5}, {2.5, 2.5, 2.5})));
  REQUIRE(boxf({0, 0, 0}, {1, 1, 1}).contains(boxf({0.5, 0.5, 0.5}, {0.75, 0.75, 0.75})));
  REQUIRE_FALSE(boxf({0, 0, 0}, {1, 1, 1}).contains(boxf({0.5, 0.5, 0.5}, {1.75, 1.75, 1.75})));
  REQUIRE_FALSE(boxf({0, 0, 0}, {1, 1, 1}).contains(boxf({1.5, 1.5, 1.5}, {1.75, 1.75, 1.75})));

  std::vector<vec3f> points{{0, 0, 0}, {-1, -2, -3}, {1, 2, 3}};
  boxf bounds = boxf::bounds(points.begin(), points.end());
  REQUIRE(bounds == boxf({-1, -2, -3}, {1, 2, 3}));
  REQUIRE(bounds.corner(0) == vec3f(-1, -2, -3));
  REQUIRE(bounds.corner(1) == vec3f(1, -2, -3));
  REQUIRE(bounds.corner(7) == vec3f(1, 2, 3));

  REQUIRE_THAT(transformf({1, 2, 3}, {0, M_PI_2, 0}, 2) * bounds, ApproxEquals(boxf({-5, -2, 1}, {7, 6, 5})));
}

TEST_CASE("cuboids can be manipulated", "[cuboid]") {
  cuboidf cuboid(mat4x3f::translate_rotate_scale({1, 2, 3}, {0, M_PI_2, 0}, {1, 2, 3}));
  REQUIRE_THAT(cuboid.corner(0), ApproxEquals(vec3f(-0.5, 1, 3.5)));
  REQUIRE_THAT(cuboid.corner(1), ApproxEquals(vec3f(-0.5, 1, 2.5)));
  REQUIRE_THAT(cuboid.corner(7), ApproxEquals(vec3f(2.5, 3, 2.5)));

  REQUIRE_THAT(inverse(cuboid.matrix) * cuboid, ApproxEquals(cuboidf(1)));
}

TEST_CASE("planes can be manipulated", "[plane]") {
  std::vector<vec3f> points{{0, 0, 0}, {-1, -2, -3}, {1, 2, 3}};
  planef slab = planef::slab({0, 1, 0}, points.begin(), points.end());

  REQUIRE(slab.slab_orientation({0, 2, 0}) == slab_orientation_type::within);
  REQUIRE(slab.slab_orientation({0, -3, 0}) == slab_orientation_type::below);
  REQUIRE(slab.slab_orientation({0, 3, 0}) == slab_orientation_type::above);

  REQUIRE(planef(0, 1, 0, -1).signed_distance({0, 1, 0}) == 0);
  REQUIRE(planef(0, 1, 0, -1).signed_distance({0, 2, 0}) == 1);
  REQUIRE(planef(0, 1, 0, -1).signed_distance({0, 0, 0}) == -1);

  REQUIRE(planef(0, 1, 0, -1).reverse().signed_distance({0, 1, 0}) == 0);
  REQUIRE(planef(0, 1, 0, -1).reverse().signed_distance({0, 2, 0}) == -1);
  REQUIRE(planef(0, 1, 0, -1).reverse().signed_distance({0, 0, 0}) == 1);

  REQUIRE(planef(0, 1, 0, -1).distance({0, 1, 0}) == 0);
  REQUIRE(planef(0, 1, 0, -1).distance({0, 2, 0}) == 1);
  REQUIRE(planef(0, 1, 0, -1).distance({0, 0, 0}) == 1);

  REQUIRE(planef(0, 1, 0, -1).half_space_intersects(boxf({0, -1, 0}, {1, 0, 1})) == intersection_type::none);
  REQUIRE(planef(0, 1, 0, -1).half_space_intersects(boxf({0, 0, 0}, {1, 2, 1})) == intersection_type::intersects);
  REQUIRE(planef(0, 1, 0, -1).half_space_intersects(boxf({0, 1, 0}, {1, 2, 1})) == intersection_type::contains);

  REQUIRE(slab.slab_intersects(boxf({0, -4, 0}, {1, -3, 1})) == intersection_type::none);
  REQUIRE(slab.slab_intersects(boxf({0, 0, 0}, {1, 4, 1})) == intersection_type::intersects);
  REQUIRE(slab.slab_intersects(boxf({0, -1, 0}, {1, 2, 1})) == intersection_type::contains);

  REQUIRE_THAT(transformf({1, 2, 3}, {0, M_PI_2, 0}, 2) * planef(0, 1, 0, -1), ApproxEquals(planef(0, 1, 0, -4)));
}

TEST_CASE("triangles can be manipulated", "[triangle]") {
  trianglef triangle({0, 0, 0}, {1, 1, 0}, {0, 1, 0});
  REQUIRE_THAT(triangle.normal(), ApproxEquals(vec3f(0, 0, 1)));
  REQUIRE_THAT(triangle.reverse().normal(), ApproxEquals(vec3f(0, 0, -1)));
  REQUIRE_THAT((mat4x3f::rotate(eulerf(0, M_PI_2, 0)) * triangle).normal(), ApproxEquals(vec3f(1, 0, 0)));
}

TEST_CASE("spheres can be manipulated", "[sphere]") {
  spheref sphere({1, 2, 3}, 2);
  REQUIRE_THAT(transformf({1, 2, 3}, {0, M_PI_2, 0}, 2) * sphere, ApproxEquals(spheref({7, 6, 1}, 4)));
}

TEST_CASE("capsules can be manipulated", "[capsule]") {
  capsulef capsule({0, 0, 0}, {1, 1, 0}, 2);
  REQUIRE_THAT(transformf({1, 2, 3}, {0, M_PI_2, 0}, 2) * capsule, ApproxEquals(capsulef({1, 2, 3}, {1, 4, 1}, 4)));
}

TEST_CASE("rays can be manipulated", "[ray]") {
  rayf ray({0, 0, 0}, {1, 1, 0});
  REQUIRE_THAT(ray.at(0.5), ApproxEquals(vec3f(0.5, 0.5, 0)));
  REQUIRE_THAT(transformf({1, 2, 3}, {0, M_PI_2, 0}, 2) * ray, ApproxEquals(rayf({1, 2, 3}, {0, 2, -2})));

  boxf box({1, 1, 1}, {2, 2, 2});
  float where;
  REQUIRE((rayf({1.5, 1.5, 1.5}, {1, 0, 0}).intersects(box, &where) && where == 0));
  REQUIRE((rayf({0, 0, 0}, {1, 1, 1}).intersects(box, &where) && where == 1));
  REQUIRE((rayf({0.5, 1.5, 1.5}, {1, 0, 0}).intersects(box, &where) && where == 0.5));
  REQUIRE_FALSE((rayf({0.5, 2.5, 1.5}, {1, 0, 0}).intersects(box)));


}

TEST_CASE("frusta can be manipulated", "[frustum]") {
}

}
