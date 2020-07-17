/*
Copyright (C) 2020 IKERLAN

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
 * @file ikStaticon.c
 * 
 * @brief Class ikStaticon implementation
 */

#include <string.h>
#include "ikStaticon.h"

int ikStaticon_init(ikStaticon *self, const ikStaticonParams *params) {
    int err = 0;
    /* copy parameters */
    self->priv.maximumTorque = params->maximumTorque;
    self->priv.minimumTorque = params->minimumTorque;
    self->priv.maximumPitch = params->maximumPitch;
    self->priv.minimumPitch = params->minimumPitch;
    self->priv.maximumSpeed = params->maximumSpeed;
    self->priv.minimumSpeed = params->minimumSpeed;
    self->priv.pitchStep = params->pitchStep;
    self->priv.torqueStep = params->torqueStep;
    self->priv.torqueN = params->torqueN;

    /* check parameters */
    if (params->maximumTorque < params->minimumTorque) err = -1;
    if (params->maximumPitch < params->minimumPitch) err = -2;
    if (params->maximumSpeed < params->minimumSpeed) err = -3;
    if (params->pitchStep < 0.0) err = -4;
    if (params->torqueStep < 0.0) err = -5;
    if (params->torqueN < 0) err = -6;

    /* init outputs */
    self->out.torqueDemand = self->priv.minimumTorque;
    self->out.pitchDemand = self->priv.maximumPitch;

    /* init private members */
    self->priv.torqueStepSign = 1;
    self->priv.constantTorqueCounter = 0;
    
    return err;
}

void ikStaticon_initParams(ikStaticonParams *params) {
    params->maximumTorque = 0.0;
    params->minimumTorque = 0.0;
    params->maximumPitch = 0.0;
    params->minimumPitch = 0.0;
    params->maximumSpeed = 0.0;
    params->minimumSpeed = 0.0;
    params->pitchStep = 0.0;
    params->torqueStep = 0.0;
    params->torqueN = 0;
}

int ikStaticon_step(ikStaticon *self) {
    double nextTorque;
    double nextPitch;

    nextTorque = self->out.torqueDemand;
    nextPitch = self->out.pitchDemand;
    self->priv.constantTorqueCounter++;
    if (self->priv.constantTorqueCounter > self->priv.torqueN) {
	self->priv.constantTorqueCounter = 0;
	nextTorque = self->out.torqueDemand + self->priv.torqueStepSign*self->priv.torqueStep;
	if (nextTorque < self->priv.maximumTorque &
	    nextTorque > self->priv.minimumTorque &
	    ((self->priv.torqueStepSign > 0 & self->in.generatorSpeed > self->priv.minimumSpeed) |
	     (self->priv.torqueStepSign < 0 & self->in.generatorSpeed < self->priv.maximumSpeed))) {
	    self->out.torqueDemand = nextTorque;
	} else {
	    self->priv.torqueStepSign *= -1;
	    nextPitch -= self->priv.torqueStep;
	}
    }
    if (nextPitch > self->priv.minimumPitch) self->out.pitchDemand = nextPitch;
}

int ikStaticon_getOutput(const ikStaticon *self, double *output, const char *name) {
    return -1;
}
