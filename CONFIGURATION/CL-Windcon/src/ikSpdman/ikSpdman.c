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
 * @file ikSpdman.c
 * 
 * @brief Class ikSpdman implementation
 */

/* @cond */

#include <stdlib.h>
#include <string.h>

#include "ikSpdman.h"

int ikSpdman_init(ikSpdman *self, const ikSpdmanParams *params) {
    int err;
	int err_ = 0;
	
	/* pass on member parameters */
	err = ikSensorDiagnoser_init(&(self->diagnoser), &(params->diagnoser));
	if (err && !err_) err_ = -1;
	
	/* register parameter values */
	self->gbratio = params->gearboxRatio;
	if (0.0 > params->T) return -2;
	self->T = params->T;
	if (params->maxAzimuth <= params->minAzimuth) return -3;
	self->azimuthRange = params->maxAzimuth - params->minAzimuth;

    return err_;
}

void ikSpdman_initParams(ikSpdmanParams *params) {
	
	/* pass on member parameters */
	ikSensorDiagnoser_initParams(&(params->diagnoser));
	
	/* set default values */
	params->gearboxRatio = 1.0;
	params->T = 1.0;
	params->minAzimuth = 0.0;
	params->maxAzimuth = 360.0;
}

int ikSpdman_step(ikSpdman *self, double generatorSpeed, double rotorSpeed, double azimuth, int ResetSignal) {
    int i;
	double diff;
	
	/* prepare signals */
	self->signals[0] = generatorSpeed;
	self->signals[1] = self->gbratio*rotorSpeed;
	diff = azimuth - self->lastAzimuth;
	diff = diff < self->azimuthRange/2 ? diff : diff - self->azimuthRange;
	diff = diff > -self->azimuthRange/2 ? diff : diff + self->azimuthRange;
	self->signals[2] = self->gbratio * diff / self->T / 180.0 * 3.14159265358979;
	self->lastAzimuth = azimuth;
	self->diagnoser.ResetSignal = ResetSignal;
	/* run diagnoser */
	ikSensorDiagnoser_step(&(self->diagnoser), self->ok, self->signals);
	
	/* compute status */
	self->status = 0;
	for (i = 0; i < 3; i++) {
		if (!self->ok[i]) {
			if (!self->status) self->status = -1 - i;
			else self->status = 4;
		}
	}
	
	/* choose the signal to output */
	switch (self->status) {
		case -1 :
			self->outputSpeed = self->signals[1];
			break;
		default :
			self->outputSpeed = self->signals[0];
	}

    return self->status;
}

int ikSpdman_getOutput(const ikSpdman *self, double *output, const char *name) {
    const char *sep;

    /* pick up the signal names */
    if (!strcmp(name, "generator speed equivalent")) {
        *output = self->outputSpeed;
        return 0;
    }
    if (!strcmp(name, "signal 1")) {
        *output = self->signals[0];
        return 0;
    }
    if (!strcmp(name, "signal 2")) {
        *output = self->signals[1];
        return 0;
    }
    if (!strcmp(name, "signal 3")) {
        *output = self->signals[2];
        return 0;
    }
    if (!strcmp(name, "ok 1")) {
        *output = self->ok[0];
        return 0;
    }
    if (!strcmp(name, "ok 2")) {
        *output = self->ok[1];
        return 0;
    }
    if (!strcmp(name, "ok 3")) {
        *output = self->ok[2];
        return 0;
    }

    /* pick up the block names */
    sep = strstr(name, ">");
    if (NULL == sep) return -1;

    return -2;
}

/* @endcond */



