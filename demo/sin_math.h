/*
 * sin-math.h
 *
 *  Created on: Oct 29, 2018
 *      Author: Ocanath
 */

#ifndef SIN_MATH_H_
#define SIN_MATH_H_
#include <stdio.h>
#include <stdint.h>


#define ONE_BY_THREE_FACTORIAL 	0.16666666666f
#define ONE_BY_FIVE_FACTORIAL 	0.00833333333f
#define HALF_PI 				1.57079632679f
#define PI						3.14159265359f
#define THREE_BY_TWO_PI     	4.71238898038f
#define TWO_PI              	6.28318530718f
#define ONE_BY_TWO_PI 			0.1591549f
#define ONE_BY_PI 0.318310f

#define ONE_BY_THREE_PI			1.0471975512f

float abs_f(float v);
float fmod_2pi(float in);
float unwrap(float theta);
float atan2_approx(float sinVal, float cosVal);
float cos_fast(float theta);
float sin_fast(float theta);

#endif /* SIN_MATH_H_ */
