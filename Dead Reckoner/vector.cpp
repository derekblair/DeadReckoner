// vector.cpp
//
// Copyright (c) 2015 Iwe Labs
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "quaternion.h"
#include "vector.h"

Vector::Vector() : x(0),y(0),z(0) {}
Vector::Vector(FLOAT a, FLOAT b, FLOAT c) : x(a),y(b),z(c) {}
Vector::Vector(const Vector& v) : x(v.x),y(v.y),z(v.z) {}

void Vector::rotateBy(const Quaternion& q) {
    Quaternion p = Quaternion(*this);
    Quaternion b = q;
    Quaternion c = q;
    c.conjugate();
    b.multiply(p);
    b.multiply(c);
    x = b.x;
    y = b.y;
    z = b.z;
}

void Vector::add(const Vector& v) {
    x+=v.x;
    y+=v.y;
    z+=v.z;
}

void Vector::sub(const Vector& v) {
    x-=v.x;
    y-=v.y;
    z-=v.z;
}

void Vector::negate() {
    x=-x;
    y=-y;
    z=-z;
}

void Vector::zero() {
    x=0;
    y=0;
    z=0;
}

void Vector::scalarMultiply(FLOAT k) {
    x*=k;
    y*=k;
    z*=k;
}

bool Vector::normalize() {
    FLOAT k = norm();
    if(k == 0) return false;
    scalarMultiply(1/k);
    return true;
}

FLOAT Vector::norm() const {
    return sqrt(x*x+y*y+z*z);
}

FLOAT Vector::distance(const Vector& v) const {
    Vector vn = Vector(x,y,z);
    vn.sub(v);
    return vn.norm();
}

FLOAT Vector::dotProduct(const Vector& v) const {
    return x*v.x + y*v.y + z*v.z;
}

bool Vector::equals(const Vector& v) const {
    return ((x == v.x) && (y == v.y) && (z == v.z));
}
