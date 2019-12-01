/**********************************************************************************************
 * FABRIK 2D inverse kinematics solver - Version 1.0.3
 * by Henrik Söderlund <henrik.a.soderlund@gmail.com>
 *
 * Copyright (c) 2018 Henrik Söderlund

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 **********************************************************************************************/

#include <math.h>
#include "FABRIK2D.h"
#include <misc.h>

namespace {
Message msg;
}
/* Fabrik2D(numJoints, lengths)
 * inputs: numJoints, lengths
 *
 * creates the chain to be used for the inverse kinematics solver
 */
Fabrik2D::Fabrik2D(int numJoints, const uint8_t * lengths)
{

  this->numJoints = numJoints;
  createChain(lengths);

  this->tolerance = 1; // 1mm tolerance default
}

/* createChain(lengths)
 * inputs: lengths
 *
 * length size should always be one lesser than the number of joints
 */
void Fabrik2D::createChain(const uint8_t * lengths)
{
	chain.joints[0].x = 0;
	chain.joints[0].y = 0;
	chain.joints[0].angle = 0;

	int sumLengths = 0;
	for (int i = 1; i < this->numJoints; i++)
	{
		sumLengths = sumLengths + lengths[i-1];
		chain.joints[i].x = 0;
		chain.joints[i].y = sumLengths;
		chain.joints[i].angle = 0;
	}

	this->chain = chain;
}

void Fabrik2D::printChain ()
{
	for(int i = 0; i < numJoints - 1; ++i) {
		msg << "FAB: j " << i << ":" << chain.joints[i].x << chain.joints[i].y << (uint16_t)(chain.joints[i].angle*180)/3.14 << Message::endl;
	}
}

/* solve(x, y, lengths)
 * inputs: x and y positions of target, lengths between each joint
 *
 * solves the inverse kinematics of the stored chain to reach the target
 */
bool Fabrik2D::solve(float x, float y, const uint8_t * lengths)
{
	// Distance between root and target (root is always 0,0)
	int dist = sqrt(x*x+y*y);

	// Total length of chain
	int totalLength = 0;
	for (int i = 0; i < this->numJoints-1; i++)
	{
		totalLength = totalLength + lengths[i];
	}

	// Check whether the target is within reach
	if (dist > totalLength)
	{
	   // The target is unreachable

	   for (int i = 0; i < this->numJoints-1; i++)
	   {
		   // Find the distance r_i between the target (x,y) and the joint i position (jx,jy)
		   float jx = chain.joints[i].x;
		   float jy = chain.joints[i].y;
		   float r_i = distance(jx,jy,x,y);
		   float lambda_i = ((float)lengths[i])/r_i;

		   // Find the new joint positions
		   chain.joints[i+1].x = (float)((1-lambda_i)*jx + lambda_i*x);
		   chain.joints[i+1].y = (float)((1-lambda_i)*jy + lambda_i*y);
	   }

	   return false;
	}
	else
	{
		// The target is reachable; this, set as (bx,by) the initial position of the joint i
		float bx = chain.joints[0].x;
		float by = chain.joints[0].y;

		// Check whether the distance between the end effector joint n (ex,ey) and the target is
		// greater than a tolerance
		float ex = chain.joints[this->numJoints-1].x;
		float ey = chain.joints[this->numJoints-1].y;
		float dif = distance(ex,ey,x,y);

		float prevDif = 0;
		float tolerance = this->tolerance;
		while (dif > tolerance)
		{

			if (prevDif == dif)
				tolerance *= 2;

			prevDif = dif;

			msg << "Fab: forward" << Message::endl;
			// STAGE 1: FORWARD REACHING
			// Set the end effector as target
			chain.joints[this->numJoints-1].x = x;
			chain.joints[this->numJoints-1].y = y;

			for (int i = numJoints-2; i >= 0; i--)
			{

				// Find the distance r_i between the new joint position i+1 (nx,ny)
				// and the joint i (jx,jy)
				float jx = chain.joints[i].x;
				float jy = chain.joints[i].y;
				float nx = chain.joints[i+1].x;
				float ny = chain.joints[i+1].y;
				float r_i = distance(jx,jy,nx,ny);
				float lambda_i = ((float)lengths[i])/r_i;

				// Find the new joint positions
				chain.joints[i].x = (float)((1-lambda_i)*nx + lambda_i*jx);
				chain.joints[i].y = (float)((1-lambda_i)*ny + lambda_i*jy);
				printChain();
			}

			msg << "Fab: backward" << Message::endl;
			// STAGE 2: BACKWARD REACHING
			// Set the root at its initial position
			chain.joints[0].x = bx;
			chain.joints[0].y = by;

			for (int i = 0; i < numJoints-1; i++)
			{

				// Find the distance r_i between the new joint position i (nx,ny)
				// and the joint i+1 (jx,jy)
				float jx = chain.joints[i+1].x;
				float jy = chain.joints[i+1].y;
				float nx = chain.joints[i].x;
				float ny = chain.joints[i].y;
				float r_i = distance(jx,jy,nx,ny);
				float lambda_i = ((float)lengths[i])/r_i;

				// Find the new joint positions
				chain.joints[i+1].x = (float)((1-lambda_i)*nx + lambda_i*jx);
				chain.joints[i+1].y = (float)((1-lambda_i)*ny + lambda_i*jy);
				printChain();
			}

			// Update distance between end effector and target
			ex = chain.joints[this->numJoints-1].x;
			ey = chain.joints[this->numJoints-1].y;
			dif = distance(ex,ey,x,y);
		}
	}
	msg << "FAB: solved" << Message::endl;

	chain.joints[0].angle = atan2(this->chain.joints[1].y, this->chain.joints[1].x);
	printChain();

	float prevAngle = chain.joints[0].angle;
	for (int i = 2; i <= numJoints-1; i++)
	{
		float ax = chain.joints[i-1].x;
		float ay = chain.joints[i-1].y;
		float bx = chain.joints[i].x;
		float by = chain.joints[i].y;

		float aAngle = atan2(by-ay,bx-ax);

		chain.joints[i-1].angle = aAngle-prevAngle;

		prevAngle = aAngle;
		printChain();
	}

	return true;
}

/* getX(joint)
 * inputs: joint number
 * outputs: x position of joint
 */
float Fabrik2D::getX(int joint)
{
  if (joint >= 0 && joint < numJoints) {

	  return chain.joints[joint].x;

  }
  return 0;
}

/* getY(joint)
 * inputs: joint number
 * outputs: y position of joint
 */
float Fabrik2D::getY(int joint)
{
  if (joint >= 0 && joint < numJoints) {

	  return chain.joints[joint].y;

  }
  return 0;
}

/* getAngle(joint)
 * inputs: joint number
 * outputs: angle (radians) of joint
 */
float Fabrik2D::getAngle(int joint)
{
  if (joint >= 0 && joint < numJoints) {

	  return chain.joints[joint].angle;

  }
  return 0;
}

/* setTolerance(tolerance)
 * inputs: tolerance value
 *
 * sets the tolerance of the distance between the end effector and the target
 */
void Fabrik2D::setTolerance(float tolerance)
{
	tolerance = tolerance;
}

/* distance(x1,y1,x2,y2)
 * inputs: coordinates
 * outputs: distance between points
 *
 * Uses euclidean distance
 */
float Fabrik2D::distance(float x1, float y1, float x2, float y2)
{
	float xDiff = x2-x1;
	float yDiff = y2-y1;
	return sqrt(xDiff*xDiff + yDiff*yDiff);
}

