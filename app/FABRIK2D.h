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


#ifndef FABRIK2D_h
#define FABRIK2D_h

#include <stdint.h>

class Fabrik2D
{
  public:
	/* Fabrik2D(numJoints, lengths)
	 * inputs: numJoints, lengths
	 *
	 * creates the chain to be used for the inverse kinematics solver
	 */
	Fabrik2D(int numJoints, const uint8_t * lengths);

	/* solve(x, y, lengths)
	 * inputs: x and y positions of target, lengths between each joint
	 * outputs: True if solvable, false if not solvable
	 *
	 * solves the inverse kinematics of the stored chain to reach the target
	 */
	bool solve(float x, float y, const uint8_t * lengths);

	/* getX(joint)
	 * inputs: joint number
	 * outputs: x position of joint
	 */
	float getX(int joint);

	/* getY(joint)
	 * inputs: joint number
	 * outputs: y position of joint
	 */
	float getY(int joint);

	/* getAngle(joint)
	 * inputs: joint number
	 * outputs: angle (radians) of joint
	 */
	float getAngle(int joint);

	/* setTolerance(tolerance)
	 * inputs: tolerance value
	 *
	 * sets the tolerance of the distance between the end effector and the target
	 */
	void setTolerance(float tolerance);

	/* createChain(lengths)
	 * inputs: lengths
	 *
	 * length size should always be one lesser than the number of joints
	 */
	void createChain(const uint8_t * lengths);

  private:

	// Joint struct
	typedef struct
	{
		float x; // x position of joint relative to origin
		float y; // y position of joint relative to origin
		float angle; // angle of joint (if the joint has adjacent joints or origin)
	} Joint;

	// Chain struct
	typedef struct
	{
	  Joint joints[3]; // list of joints
	  float angle; // base (plane) rotation
	} Chain;

	// Number of joints in the chain
	int numJoints;
	// Tolerance of distance between end effector and target
	float tolerance;
	// The chain containing joints
	Chain chain;

	/* distance(x1,y1,x2,y2)
	 * inputs: coordinates
	 * outputs: distance between points
	 *
	 * Uses euclidean distance
	 */
	float distance(float x1, float y1, float x2, float y2);
};

#endif
