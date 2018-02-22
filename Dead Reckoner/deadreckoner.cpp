// deadreckoner.cpp
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

#include "deadreckoner.h"

static FLOAT max(FLOAT *values, UINT start, UINT end);
static FLOAT min(FLOAT *values, UINT start, UINT end);
static void computeLocalMeans(FLOAT *data,FLOAT *targ,UINT size, UINT window);
static void lowPassFilter(FLOAT *data,FLOAT *targ,FLOAT p,FLOAT tc,UINT size);
static Quaternion reflectToProper(const Quaternion& q);
static FLOAT weinbergStrideLength(UINT stance, FLOAT *norms, UINT length, UINT window, FLOAT period);


static const FLOAT thresh1 = 2.0f;
static const FLOAT thresh2 = 1.0f;
static const FLOAT weinbergK = 0.6;

DeadReckoner::DeadReckoner() {
	position = Vector(0,0,0);
	initialHeading = currentHeading = 0.0f;
}

DeadReckoner::~DeadReckoner() {
}


void DeadReckoner::updatePositionAndHeadingBasedOnSensorData(SensorReading *readings, UINT length, FLOAT samplePeriod) {
    
    const UINT sampleWindow = 30*0.005/samplePeriod;
    const UINT searchSize = (UINT)(1/samplePeriod)*2; // 2s of readings
   
    FLOAT *tempBuffer = new FLOAT[4*searchSize];
    FLOAT *accelNorms = tempBuffer;
    FLOAT *b1 = tempBuffer+searchSize;
    FLOAT *b2 = tempBuffer+2*searchSize;
    FLOAT *localMeans = tempBuffer+3*searchSize;
    
    UINT start = 0;
    
    while (start <= length - searchSize) {
    
        for (UINT i = 0; i < searchSize ; i++) {
            accelNorms[i] = (readings[start+i]).acceleration.norm();
        }
        
        computeLocalMeans(accelNorms,localMeans,searchSize,sampleWindow);
        
        //Test thresholds
        for (UINT i = 2*sampleWindow; i <  searchSize - 2*sampleWindow ; i++) {
            FLOAT sumOfDifferenceSquares = 0;
            for (UINT  j = i - sampleWindow; j <= i + sampleWindow ; j++) {
                FLOAT d = (accelNorms[j] - localMeans[j]);
                sumOfDifferenceSquares += d*d;
            }
            FLOAT sdev = sqrt(sumOfDifferenceSquares / (2*sampleWindow + 1));
            b1[i] = sdev > thresh1 ? thresh1 : 0;
            b2[i] = sdev < thresh2 ? thresh2 : 0;
        }
        
        UINT stance = 0;
        
        for (UINT i = 2*sampleWindow + 1; i < searchSize - 2*sampleWindow; i++) {
            if (b1[i-1] > b1[i] && max(b2,i,i+sampleWindow) == thresh2) {
                stance = i; 
                break;
            }
        }
        // A step was detected.
        if (stance > 0) {
            FLOAT sl = weinbergStrideLength(stance,accelNorms,searchSize,sampleWindow,samplePeriod);
            Quaternion creading = (readings[stance+start]).orientation;
            Quaternion reading = reflectToProper(creading);
            FLOAT dHeading = reading.getYaw();
            currentHeading = initialHeading + dHeading;
            Vector dPos = Vector(cos(currentHeading),sin(currentHeading),0);
            dPos.scalarMultiply(sl);
            position.add(dPos);
            start += stance - sampleWindow;
        } else start += searchSize-4*sampleWindow-1;
    }
    delete[] tempBuffer;
}
		

// Numerical Sub-Routines. Side-Effect Free. Purely Functional

void computeLocalMeans(FLOAT *data, FLOAT *targ, UINT size, UINT window) {
	FLOAT temp;
	for (UINT i = window; i <  size - window; i++) {
        temp = 0;
        for (UINT j = i - window ; j <= i + window ; j++) {
            temp += data[j];
        }
		targ[i] = temp / (2*window + 1);
    }
}

void lowPassFilter(FLOAT *data, FLOAT *targ, FLOAT p, FLOAT tc, UINT size) {
	FLOAT alpha = p/(tc+p);
	targ[0] = data[0];
	for (UINT i = 1; i < size; i++) {
		targ[i] = alpha * data[i] + (1-alpha)*targ[i-1];
	}
}

FLOAT max(FLOAT *values, UINT start, UINT end) {
    UINT c;
    FLOAT max;
    max = values[start];
    for (c = start + 1 ; c <= end ; c++) {
        max = (max < values[c] ? values[c] : max);
    }
    return max;
}

FLOAT min(FLOAT *values, UINT start, UINT end) {
    UINT c;
    FLOAT min;
    min = values[start];
    for (c = start; c <= end; c++) {
        min = (min > values[c] ? values[c] : min);
    }
    return min;
}


static FLOAT weinbergStrideLength(UINT stance, FLOAT *norms, UINT length, UINT window, FLOAT period) {
    UINT b = stance - window;
    UINT a = stance + window;
    FLOAT *lpa = new FLOAT[length];
    lowPassFilter(norms,lpa,period,period*2,length);
    FLOAT result = weinbergK * sqrt(sqrt(max(lpa,b,a) - min(lpa,b,a)));
    delete [] lpa;
    return result;
}

Quaternion reflectToProper(const Quaternion& q) {
    return Quaternion(-q.y,-q.x,-q.z,q.w);
}




