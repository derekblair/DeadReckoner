
#ifndef DEAD_RECKONER_H_
#define DEAD_RECKONER_H_

// deadreckoner.h
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

#include "datatypes.h"
#include "vector.h"
#include "quaternion.h"

struct SensorReading {
    Quaternion orientation;
    Vector acceleration;
};

class DeadReckoner {
public:
	DeadReckoner();
	~DeadReckoner();
   void updatePositionAndHeadingBasedOnSensorData(SensorReading *readings, UINT length, FLOAT samplePeriod);
   Vector position;
   FLOAT initialHeading;
   FLOAT currentHeading;
};



#endif /*DEAD_RECKONER_H_*/
