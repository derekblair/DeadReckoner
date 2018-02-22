// quaternion.cpp
//
// Copyright (c) 2018 Derek Blair
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

#include "vector.h"
#include "quaternion.h"

Quaternion::Quaternion() : x(0),y(0),z(0),w(1) {}

Quaternion::Quaternion(FLOAT a, FLOAT b, FLOAT c, FLOAT d) : x(a),y(b),z(c),w(d) {}

Quaternion::Quaternion(const Quaternion& q) {
    w=q.w;
    x=q.x;
    y=q.y;
    z=q.z;
}

Quaternion::Quaternion(const Vector& v) : x(v.x),y(v.y),z(v.z),w(0.0f) {}

Quaternion::Quaternion(const Vector& axis, FLOAT theta) {
    Vector v = axis;
    v.normalize();
	v.scalarMultiply(sin(theta/2));
	x = v.x;
	y = v.y;
	z = v.z;
	w = cos(theta/2);
}

void Quaternion::conjugate() {
    x=-x;
    y=-y;
    z=-z;
}

void Quaternion::multiply(const Quaternion& q) {
    FLOAT nw = w*q.w - x*q.x - y*q.y - z*q.z;
    FLOAT nx = w*q.x + x*q.w + y* q.z - z*q.y;
    FLOAT ny = w*q.y - x*q.z + y*q.w + z*q.x;
    FLOAT nz = w*q.z + x*q.y - y*q.x + z*q.w;
    w=nw;
    x=nx;
    y=ny;
    z=nz;
}

FLOAT Quaternion::norm() const {
    return sqrt(x*x+y*y+z+z+w*w);
}

// rotation about Z-axis, aka 'attitude'
FLOAT Quaternion::getPitch() const {
    FLOAT test = x*y + z*w;
    if (test > 0.4999) // singularity at north pole
        return M_PI/2;
    if (test < -0.4999) // singularity at north pole
        return -M_PI/2;
    return  asin(2*test);
}

// rotation about Y-axis, aka 'heading'
FLOAT Quaternion::getYaw() const {
    FLOAT test = x*y + z*w;
    if (test > 0.4999) // singularity at north pole
        return 2 * atan2(x,w);
    if (test < -0.4999)  // singularity at south pole
        return -2 * atan2(x,w);
    return atan2(2*y*w-2*x*z , 1 - 2*(y*y + z*z));
}

// rotation about X-axis, aka 'bank'
FLOAT Quaternion::getRoll() const {
    FLOAT test = x*y + z*w;
    if (test > 0.4999 || test < -0.4999) // singularity at a pole
        return 0;
    return atan2(2*(x*w-y*z) , 1 - 2*(x*x+z*z));
}

bool Quaternion::equals(const Quaternion& q) const {
    return ((x == q.x) && (y == q.y) && (z == q.z) && (w == q.w));
}





