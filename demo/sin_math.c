/*
 * sin-math.c
 *
 *  Created on: Oct 29, 2018
 *      Author: Ocanath
 */
#include "sin_math.h"

float abs_f(float v)
{
	if(v < 0.0f)
		v = -v;
	return v;
}


float prev_theta = 0;
float sin_fast(float theta)
{
	theta = fmod_2pi(theta+PI)-PI;
	uint8_t is_neg = 0;
	if(theta > HALF_PI && theta <= PI)	// if positive and in quadrant II, put in quadrant I (same)
	{
		theta = PI - theta;
	}
	else if (theta >= PI && theta < THREE_BY_TWO_PI)  // if positive and in quadrant III (possible for cosine)
	{
		is_neg = 1;
		theta = theta - PI;
	}

	else if (theta > THREE_BY_TWO_PI && theta < TWO_PI)  // if positive and in quadrant IV (edge case of cosine, rare but possible)
	{
		theta = theta - TWO_PI;
	}
	else if (theta < -HALF_PI && theta >= -PI ) // if negative and in quadrant III,
	{
		is_neg = 1;
		theta = PI + theta;
	}

	float theta_2 = theta*theta;
	float theta_3 = theta_2*theta;
	float theta_5 = theta_3*theta_2;
	float res = theta-theta_3*ONE_BY_THREE_FACTORIAL + theta_5 * ONE_BY_FIVE_FACTORIAL;
	if(is_neg == 1)
		return -res;
	else
		return res;
}


float cos_fast(float theta)
{
	return sin_fast(theta + HALF_PI);
}


/* https://math.stackexchange.com/questions/1098487/atan2-faster-approximation. */
float atan2_approx(float sinVal, float cosVal)
{
	float abs_s, abs_c, min_v, max_v;
	abs_s = sinVal;
	abs_c = cosVal;
	if(abs_s < 0)
		abs_s = -abs_s;
	if(abs_c < 0)
		abs_c = -abs_c;
	if(abs_s < abs_c)
	{
		min_v = abs_s;
		max_v = abs_c;
	}
	else
	{
		min_v = abs_c;
		max_v = abs_s;
	}
	float a = min_v/max_v;
	float sv = a*a;
	float r = ((-0.0464964749f * sv + 0.15931422f) * sv - 0.327622764f) * sv * a + a;
	if(abs_s > abs_c)
		r = 1.57079637f -r;
	if(cosVal < 0)
		r = 3.14159274f - r;
	if(sinVal < 0)
		r = -r;
	return r;
}

/*
	fast 2pi mod. needed for sin and cos FAST for angle limiting
 */
float fmod_2pi(float in)
{
	uint8_t aneg = 0;
	float in_eval = in;
	if(in < 0)
	{
		aneg = 1;
		in_eval = -in;
	}
	float fv = (float)((int)(in_eval*ONE_BY_TWO_PI));
	if(aneg == 1)
		fv = (-fv)-1;
	return in-TWO_PI*fv;
}

float unwrap(float theta)
{
	float dif = fmod_2pi(fmod_2pi(prev_theta) - theta + PI);
	if(dif < 0)
		dif = dif + TWO_PI;
	dif = dif - PI;
	float theta_ret = prev_theta - dif;
	prev_theta = theta_ret;
	return theta_ret * 0.5f;
}
