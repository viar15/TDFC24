/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Copyright 2010 Michael Smith, all rights reserved.

// Derived closely from:
/****************************************
* 2D Vector Classes
* By Bill Perone (billperone@yahoo.com)
* Original: 9-16-2002
* Revised: 19-11-2003
*          18-12-2003
*          06-06-2004
*
* Copyright 2003, This code is provided "as is" and you can use it freely as long as
* credit is given to Bill Perone in the application it is used in
****************************************/
#pragma once

#include <math.h>

template <typename T>
struct Vector2
{
    T x, y;

    // trivial ctor
    constexpr Vector2<T>()
        : x(0)
        , y(0) {}

    // setting ctor
    constexpr Vector2<T>(const T x0, const T y0)
        : x(x0)
        , y(y0) {}

    // function call operator
    void operator ()(const T x0, const T y0)
    {
        x= x0; y= y0;
    }

    // test for equality
    bool operator ==(const Vector2<T> &v) const;

    // test for inequality
    bool operator !=(const Vector2<T> &v) const;

    // negation
    Vector2<T> operator -(void) const;

    // addition
    Vector2<T> operator +(const Vector2<T> &v) const;

    // subtraction
    Vector2<T> operator -(const Vector2<T> &v) const;

    // uniform scaling
    Vector2<T> operator *(const T num) const;

    // uniform scaling
    Vector2<T> operator  /(const T num) const;

    // addition
    Vector2<T> &operator +=(const Vector2<T> &v);

    // subtraction
    Vector2<T> &operator -=(const Vector2<T> &v);

    // uniform scaling
    Vector2<T> &operator *=(const T num);

    // uniform scaling
    Vector2<T> &operator /=(const T num);

    // dot product
    T operator *(const Vector2<T> &v) const;

    // cross product
    T operator %(const Vector2<T> &v) const;

    // computes the angle between this vector and another vector
    // returns 0 if the vectors are parallel, and M_PI if they are antiparallel
    float angle(const Vector2<T> &v2) const;

    // computes the angle of this vector in radians, from 0 to 2pi,
    // from a unit vector(1,0); a (1,1) vector's angle is +M_PI/4
    float angle(void) const;

    // check if any elements are NAN
    bool is_nan(void) const;

    // check if any elements are infinity
    bool is_inf(void) const;

    // check if all elements are zero
    bool is_zero(void) const { return (fabsf(x) < FLT_EPSILON) && (fabsf(y) < FLT_EPSILON); }

    // allow a vector2 to be used as an array, 0 indexed
    T & operator[](uint8_t i) {
        T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 2);
#endif
        return _v[i];
    }

    const T & operator[](uint8_t i) const {
        const T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 2);
#endif
        return _v[i];
    }
    
    // zero the vector
    void zero()
    {
        x = y = 0;
    }

    // gets the length of this vector squared
    float length_squared() const;

    // gets the length of this vector
    float length(void) const;

    // normalizes this vector
    void normalize();

    // returns the normalized vector
    Vector2<T> normalized() const;

    // reflects this vector about n
    void reflect(const Vector2<T> &n);

    // projects this vector onto v
    void project(const Vector2<T> &v);

    // returns this vector projected onto v
    Vector2<T> projected(const Vector2<T> &v);

    // given a position p1 and a velocity v1 produce a vector
    // perpendicular to v1 maximising distance from p1
    static Vector2<T> perpendicular(const Vector2<T> &pos_delta, const Vector2<T> &v1);

    /*
     * Returns the point closest to p on the line segment (v,w).
     *
     * Comments and implementation taken from
     * http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     */
    static Vector2<T> closest_point(const Vector2<T> &p, const Vector2<T> &v, const Vector2<T> &w);

    /*
     * Returns the point closest to p on the line segment (0,w).
     *
     * this is a simplification of closest point with a general segment, with v=(0,0)
     */
    static Vector2<T> closest_point(const Vector2<T> &p, const Vector2<T> &w);

    // w1 and w2 define a line segment
    // p is a point
    // returns the square of the closest distance between the line segment and the point
    static float closest_distance_between_line_and_point_squared(const Vector2<T> &w1,
                                                                 const Vector2<T> &w2,
                                                                 const Vector2<T> &p);

    // w1 and w2 define a line segment
    // p is a point
    // returns the closest distance between the line segment and the point
    static float closest_distance_between_line_and_point(const Vector2<T> &w1,
                                                         const Vector2<T> &w2,
                                                         const Vector2<T> &p);

    // a1->a2 and b2->v2 define two line segments
    // returns the square of the closest distance between the two line segments
    static float closest_distance_between_lines_squared(const Vector2<T> &a1,
                                                        const Vector2<T> &a2,
                                                        const Vector2<T> &b1,
                                                        const Vector2<T> &b2);

    // w defines a line segment from the origin
    // p is a point
    // returns the square of the closest distance between the radial and the point
    static float closest_distance_between_radial_and_point_squared(const Vector2<T> &w,
                                                                   const Vector2<T> &p);

    // w defines a line segment from the origin
    // p is a point
    // returns the closest distance between the radial and the point
    static float closest_distance_between_radial_and_point(const Vector2<T> &w,
                                                           const Vector2<T> &p);

    // find the intersection between two line segments
    // returns true if they intersect, false if they do not
    // the point of intersection is returned in the intersection argument
    static bool segment_intersection(const Vector2<T>& seg1_start, const Vector2<T>& seg1_end, const Vector2<T>& seg2_start, const Vector2<T>& seg2_end, Vector2<T>& intersection);

    // find the intersection between a line segment and a circle
    // returns true if they intersect and intersection argument is updated with intersection closest to seg_start
    static bool circle_segment_intersection(const Vector2<T>& seg_start, const Vector2<T>& seg_end, const Vector2<T>& circle_center, float radius, Vector2<T>& intersection);

    static bool point_on_segment(const Vector2<T>& point,
                                 const Vector2<T>& seg_start,
                                 const Vector2<T>& seg_end) {
        const float expected_run = seg_end.x-seg_start.x;
        const float intersection_run = point.x-seg_start.x;
        // check slopes are identical:
        if (fabsf(expected_run) < FLT_EPSILON) {
            if (fabsf(intersection_run) > FLT_EPSILON) {
                return false;
            }
        } else {
            const float expected_slope = (seg_end.y-seg_start.y)/expected_run;
            const float intersection_slope = (point.y-seg_start.y)/intersection_run;
            if (fabsf(expected_slope - intersection_slope) > FLT_EPSILON) {
                return false;
            }
        }
        // check for presence in bounding box
        if (seg_start.x < seg_end.x) {
            if (point.x < seg_start.x || point.x > seg_end.x) {
                return false;
            }
        } else {
            if (point.x < seg_end.x || point.x > seg_start.x) {
                return false;
            }
        }
        if (seg_start.y < seg_end.y) {
            if (point.y < seg_start.y || point.y > seg_end.y) {
                return false;
            }
        } else {
            if (point.y < seg_end.y || point.y > seg_start.y) {
                return false;
            }
        }
        return true;
    }
};

typedef Vector2<int16_t>        Vector2i;
typedef Vector2<uint16_t>       Vector2ui;
typedef Vector2<int32_t>        Vector2l;
typedef Vector2<uint32_t>       Vector2ul;
typedef Vector2<float>          Vector2f;
