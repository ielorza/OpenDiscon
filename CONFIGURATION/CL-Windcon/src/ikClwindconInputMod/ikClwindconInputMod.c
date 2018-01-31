/*
Copyright (C) 2017 IK4-IKERLAN

This file is part of OpenDiscon.
 
OpenDiscon is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
 
OpenDiscon is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
 
You should have received a copy of the GNU General Public License
along with OpenDiscon. If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file ikClwindconInputMod.c
 *
 * @brief CL-Windcon wind turbine controller input modification
 */

#include "ikClwindconInputMod.h"

void ikClwindconInputMod(ikClwindconWTConInputs *in) {

	ikGeneratorSpeedSingalFail(in);

}

void ikGeneratorSpeedSingalFail(ikClwindconWTConInputs *in) {
	static int _n = 0;
	
	/*! [Speed sensor fault] */
	/*
	####################################################################
	                    Speed sensor fault

	A generator speed measurement of val is enforced after N sampling
	intervals (N <= 0 to disable).

	Set parameters here:
	*/
	const int N = 10000;
	const double val = 0.0;
	/*
	####################################################################
	*/
	/*! [Speed sensor fault] */
	
	if (0 < N && _n < N) {
		_n++;
		return;
	}
	
	in->generatorSpeed = val;
	
}
