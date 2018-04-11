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
 * @file ikTpman.c
 * 
 * @brief Class ikTpman implementation
 */

/* @cond */

#include <stdlib.h>
#include <string.h>

#include "ikTpman.h"

int ikTpman_init(ikTpman *self, const ikTpmanParams *params) {
    /* set state to 0 */
    self->state = 0;

    return 0;
}

void ikTpman_initParams(ikTpmanParams *params) {
}

int ikTpman_step(ikTpman *self, double torque, double maxTorque, double minTorqueExt, double pitch, double maxPitchExt, double minPitchExt) {
    /* save inputs */
    self->maxPitchExt = maxPitchExt;
    self->minPitchExt = minPitchExt;
    self->torque = torque;
    self->pitch = pitch;
    self->minTorqueExt = minTorqueExt;
    self->maxTorque = maxTorque;

    /* transition between states if necessary */
    switch (self->state) {
        case 0:
            if ((torque >= maxTorque) || (pitch > self->minPitchExt)) self->state = 1;
            break;
        case 1:
            if (pitch <= self->minPitchExt) self->state = 0;
            break;
    }

    /* calculate limits depending on state */
    switch (self->state) {
        case 0:
            self->maxPitch = pitch;
            self->maxPitch = self->maxPitch < maxPitchExt ? self->maxPitch : maxPitchExt;
            self->maxPitch = self->maxPitch > self->minPitchExt ? self->maxPitch : self->minPitchExt;
            self->minTorque = minTorqueExt;
            break;
        case 1:
            self->maxPitch = maxPitchExt;
            self->minTorque = torque;
            self->minTorque = self->minTorque < maxTorque ? self->minTorque : maxTorque;
            self->minTorque = self->minTorque > minTorqueExt ? self->minTorque : minTorqueExt;
            break;
    }

    return self->state;
}

int ikTpman_getOutput(const ikTpman *self, double *output, const char *name) {
    const char *sep;

    /* pick up the signal names */
    if (!strcmp(name, "maximum pitch")) {
        *output = self->maxPitch;
        return 0;
    }
    if (!strcmp(name, "minimum torque")) {
        *output = self->minTorque;
        return 0;
    }
    if (!strcmp(name, "external maximum pitch")) {
        *output = self->maxPitchExt;
        return 0;
    }
    if (!strcmp(name, "external minimum pitch")) {
        *output = self->minPitchExt;
        return 0;
    }
    if (!strcmp(name, "torque")) {
        *output = self->torque;
        return 0;
    }
    if (!strcmp(name, "pitch")) {
        *output = self->pitch;
        return 0;
    }
    if (!strcmp(name, "external minimum torque")) {
        *output = self->minTorqueExt;
        return 0;
    }
    if (!strcmp(name, "maximum torque")) {
        *output = self->maxTorque;
        return 0;
    }

    /* pick up the block names */
    sep = strstr(name, ">");
    if (NULL == sep) return -1;

    return -2;
}

/* @endcond */



