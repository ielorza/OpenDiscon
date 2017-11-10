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
 * @file ikPowman.c
 * 
 * @brief Class ikPowman implementation
 */

/* @cond */

#include "ikPowman.h"

int ikPowman_init(ikPowman *self, const ikPowmanParams *params) {
	int err;
	
	/* register rated power */
	self->ratedPower = params->ratedPower;
	
	/* register efficiency */
	if (0 == params->efficiency) return -1;
	self->efficiency = params->efficiency;
	
	/* initialise look-up tables */
	ikLutbl_init(&(self->lutblKopt));
	err = ikLutbl_setPoints(&(self->lutblKopt), params->belowRatedTorqueGainTableN, params->belowRatedTorqueGainTableX, params->belowRatedTorqueGainTableY);
	if (err) return -2;
	
	ikLutbl_init(&(self->lutblPitch));
	err = ikLutbl_setPoints(&(self->lutblPitch), params->minimumPitchTableN, params->minimumPitchTableX, params->minimumPitchTableY);
	if (err) return -3;
	
	return 0;
}

void ikPowman_initParams(ikPowmanParams *params) {
	/* set power to 0 */
	params->ratedPower = 0.0;
	
	/* set efficiency to 1 */
	params->efficiency = 1.0;
	
	/* make the below rated torque gain 0 */
	params->belowRatedTorqueGainTableN = 1;
	params->belowRatedTorqueGainTableX[0] = 0.0;
	params->belowRatedTorqueGainTableY[0] = 0.0;
	
	/* make the minimum pitch 0 */
	params->minimumPitchTableN = 1;
	params->minimumPitchTableX[0] = 0.0;
	params->minimumPitchTableY[0] = 0.0;
}

double ikPowman_step(ikPowman *self, double deratingRatio, double maxSpeed, double measuredSpeed) {
	/* register inputs */
	self->deratingRatio = deratingRatio;
	self->maxSpeed = maxSpeed;
	self->measuredSpeed = measuredSpeed;
	
	/* calculate maximum torque */	
	self->maximumTorque = (1-deratingRatio)*self->ratedPower/maxSpeed/self->efficiency;
	
	/* calculate below rated torque */
	self->belowRatedTorque = ikLutbl_eval(&(self->lutblKopt), deratingRatio)*measuredSpeed*measuredSpeed;
	
	/* calculate minimum pitch */
	self->minimumPitch = ikLutbl_eval(&(self->lutblPitch), deratingRatio);
	
	/* return the maximum torque */
	return self->maximumTorque;
}

int ikPowman_getOutput(const ikPowman *self, double *output, const char *name) {
	/* pick up the signal names */
    if (!strcmp(name, "derating ratio")) {
        *output = self->deratingRatio;
        return 0;
    }
    if (!strcmp(name, "maximum speed")) {
        *output = self->maxSpeed;
        return 0;
    }
    if (!strcmp(name, "measured speed")) {
        *output = self->measuredSpeed;
        return 0;
    }
    if (!strcmp(name, "maximum torque")) {
        *output = self->maximumTorque;
        return 0;
    }
    if (!strcmp(name, "below rated torque")) {
        *output = self->belowRatedTorque;
        return 0;
    }
    if (!strcmp(name, "minimum pitch")) {
        *output = self->minimumPitch;
        return 0;
    }
	
	return -1;
}

/* @endcond */
