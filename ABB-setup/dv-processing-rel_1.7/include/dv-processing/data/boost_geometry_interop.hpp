#pragma once

#include "bounding_box_base.hpp"
#include "event_base.hpp"
#include "timed_keypoint_base.hpp"

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometry.hpp>
#include <opencv2/core.hpp>

// This header contains interoperability primitives for the boost geometry library
// By registering these types, they can be used directly with algorithms available in the
// Boost.Geometry library

// DV type interoperability
BOOST_GEOMETRY_REGISTER_POINT_2D_CONST(dv::Point2f, float, boost::geometry::cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_POINT_2D_CONST(dv::TimedKeyPoint, float, boost::geometry::cs::cartesian, pt.x(), pt.y())
BOOST_GEOMETRY_REGISTER_POINT_2D_CONST(dv::Event, int16_t, boost::geometry::cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_BOX_2D_4VALUES(dv::BoundingBox, dv::Point2f, topLeftX, topLeftY, bottomRightX, bottomRightY)

// OpenCV type interoperability
BOOST_GEOMETRY_REGISTER_POINT_2D(cv::Point2f, float, boost::geometry::cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_POINT_2D(cv::Point2d, double, boost::geometry::cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_POINT_2D(cv::Point2i, int, boost::geometry::cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_POINT_2D(cv::Point2l, long, boost::geometry::cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_BOX(cv::Rect2f, cv::Point2f, tl(), br())
BOOST_GEOMETRY_REGISTER_BOX(cv::Rect2i, cv::Point2i, tl(), br())
BOOST_GEOMETRY_REGISTER_BOX(cv::Rect2d, cv::Point2d, tl(), br())
