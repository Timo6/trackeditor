#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE Spline
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include "Spline.h"

using namespace Vamos_Geometry;

struct Empty_Fixture
{
  Spline spline;
};

BOOST_AUTO_TEST_CASE (interpolate_empty)
{
  Empty_Fixture f;
  BOOST_CHECK_EQUAL (f.spline.size (), size_t (0));
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 0.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 0.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (-1.0), 0.0);
}

BOOST_AUTO_TEST_CASE (load_point)
{
  Empty_Fixture f;
  f.spline.load (Two_Vector (1.0, 2.0));
  BOOST_CHECK_EQUAL (f.spline.size (), size_t (1));
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (-1.0), 2.0);
}

BOOST_AUTO_TEST_CASE (load_xy_point)
{
  Empty_Fixture f;
  std::vector <Two_Vector> points;
  f.spline.load (1.0, 2.0);
  BOOST_CHECK_EQUAL (f.spline.size (), size_t (1));
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (-1.0), 2.0);
}

BOOST_AUTO_TEST_CASE (test_load_points)
{
  Empty_Fixture f;
  std::vector <Two_Vector> points;
  points.push_back (Two_Vector (1.0, 2.0));
  points.push_back (Two_Vector (2.0, 3.0));
  f.spline.load (points);
  BOOST_CHECK_EQUAL (f.spline.size (), size_t (2));
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.0), 3.0);
}

struct Two_Point_Fixture
{
  Two_Point_Fixture ()
  {
    spline.load (Two_Vector (1.0, 2.0));
    spline.load (Two_Vector (2.0, 3.0));
  }
  Spline spline;
};

struct Two_Point_Vector_Fixture
{
  Two_Point_Vector_Fixture ()
  {
    std::vector <Two_Vector> points;
    points.push_back (Two_Vector (1.0, 2.0));
    points.push_back (Two_Vector (2.0, 3.0));
    spline.load (points);
  }
  Spline spline;
};

BOOST_AUTO_TEST_CASE (interpolate)
{
  Two_Point_Fixture f;
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.5), 3.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.1), 2.028);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.5), 2.5);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.0), 3.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.5), 2.0);
}

BOOST_AUTO_TEST_CASE (interpolate_vector)
{
  Two_Point_Vector_Fixture f;
  BOOST_CHECK_EQUAL (f.spline.interpolate (0.5), 3.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.1), 2.028);
  BOOST_CHECK_EQUAL (f.spline.interpolate (1.5), 2.5);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.0), 3.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.5), 2.0);
}

BOOST_AUTO_TEST_CASE (clear)
{
  Two_Point_Fixture f;
  BOOST_CHECK_EQUAL (f.spline.size (), size_t (2));
  f.spline.clear ();
  BOOST_CHECK_EQUAL (f.spline.size (), size_t (0));
}

BOOST_AUTO_TEST_CASE (slopes)
{
  Two_Point_Fixture f;
  BOOST_CHECK_EQUAL (f.spline.slope (1.0), 0.0);
  BOOST_CHECK_EQUAL (f.spline.slope (2.0), 0.0);
  f.spline.set_slope (-1.0, 1.0);
  BOOST_CHECK_CLOSE (f.spline.slope (1.0), -1.0, 1e-4);
  BOOST_CHECK_CLOSE (f.spline.slope (2.0), 1.0, 1e-4);
}

BOOST_AUTO_TEST_CASE (second_derivatives)
{
  Two_Point_Fixture f;
  BOOST_CHECK_EQUAL (f.spline.second_derivative (1.0), 6.0);
  BOOST_CHECK_EQUAL (f.spline.second_derivative (1.5), 0.0);
  BOOST_CHECK_EQUAL (f.spline.second_derivative (2.0), -6.0);
}

BOOST_AUTO_TEST_CASE (remove_greater)
{
  Two_Point_Fixture f;
  f.spline.remove_greater (1.0);
  BOOST_CHECK_EQUAL (f.spline.size (), size_t (1));
}

BOOST_AUTO_TEST_CASE (scale)
{
  Two_Point_Fixture f;
  f.spline.scale (2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (2.0), 2.0);
  BOOST_CHECK_EQUAL (f.spline.interpolate (4.0), 3.0);
}

BOOST_AUTO_TEST_CASE (normal)
{
  Two_Point_Fixture f;
  BOOST_CHECK_CLOSE (f.spline.normal (0.5).x, 0.97619, 1e-3);
  BOOST_CHECK_CLOSE (f.spline.normal (0.5).y, 0.21693, 1e-3);
  BOOST_CHECK_EQUAL (f.spline.normal (1.0).x, 0.0);
  BOOST_CHECK_EQUAL (f.spline.normal (1.0).y, 1.0);
  BOOST_CHECK_CLOSE (f.spline.normal (1.1).x, -0.47515, 1e-3);
  BOOST_CHECK_CLOSE (f.spline.normal (1.1).y, 0.87991, 1e-3);
  BOOST_CHECK_CLOSE (f.spline.normal (1.5).x, -0.83205, 1e-3);
  BOOST_CHECK_CLOSE (f.spline.normal (1.5).y, 0.55470, 1e-3);
  BOOST_CHECK_EQUAL (f.spline.normal (2.0).x, 0.0);
  BOOST_CHECK_EQUAL (f.spline.normal (2.0).y, 1.0);
  BOOST_CHECK_CLOSE (f.spline.normal (3.0).x, 0.99655, 1e-3);
  BOOST_CHECK_CLOSE (f.spline.normal (3.0).y, 0.083045, 1e-3);
}

BOOST_AUTO_TEST_CASE (one_segment_path)
{
  Spline_Path path;
  path.load (Two_Vector (1.0, 2.0));
  path.load (Two_Vector (2.0, 3.0));

  BOOST_CHECK_EQUAL (path.interpolate (0.5), 3.0);
  BOOST_CHECK_EQUAL (path.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (path.interpolate (1.1), 2.028);
  BOOST_CHECK_EQUAL (path.interpolate (1.5), 2.5);
  BOOST_CHECK_EQUAL (path.interpolate (2.0), 3.0);
  BOOST_CHECK_EQUAL (path.interpolate (2.5), 2.0);
}

BOOST_AUTO_TEST_CASE (one_vector_segment_path)
{
  Spline_Path path;
  std::vector <Two_Vector> points;
  points.push_back (Two_Vector (1.0, 2.0));
  points.push_back (Two_Vector (2.0, 3.0));
  path.load (points);

  BOOST_CHECK_EQUAL (path.interpolate (0.5), 3.0);
  BOOST_CHECK_EQUAL (path.interpolate (1.0), 2.0);
  BOOST_CHECK_EQUAL (path.interpolate (1.1), 2.028);
  BOOST_CHECK_EQUAL (path.interpolate (1.5), 2.5);
  BOOST_CHECK_EQUAL (path.interpolate (2.0), 3.0);
  BOOST_CHECK_EQUAL (path.interpolate (2.5), 2.0);
}

BOOST_AUTO_TEST_CASE (two_segment_path)
{
  Spline first;
  Spline second;

  Spline_Path path;
  std::vector <Two_Vector> points;
  points.push_back (Two_Vector (1.0, 2.0));
  points.push_back (Two_Vector (2.0, 3.0));
  first.load (points);
  path.load (points);

  points.clear ();
  points.push_back (Two_Vector (3.0, 4.0));
  second.load (first [1]);
  second.load (points);
  path.load (points);

  BOOST_CHECK_EQUAL (path.interpolate (0.5), first.interpolate (0.5));
  BOOST_CHECK_EQUAL (path.interpolate (1.0), first.interpolate (1.0));
  BOOST_CHECK_EQUAL (path.interpolate (1.1), first.interpolate (1.1));
  BOOST_CHECK_EQUAL (path.interpolate (1.5), first.interpolate (1.5));
  BOOST_CHECK_EQUAL (path.interpolate (2.0), first.interpolate (2.0));
  BOOST_CHECK_EQUAL (path.interpolate (2.0), second.interpolate (2.0));
  BOOST_CHECK_EQUAL (path.interpolate (2.1), second.interpolate (2.1));
  BOOST_CHECK_EQUAL (path.interpolate (2.5), second.interpolate (2.5));
  BOOST_CHECK_EQUAL (path.interpolate (3.0), second.interpolate (3.0));
  BOOST_CHECK_EQUAL (path.interpolate (3.5), second.interpolate (3.5));
}
