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
 * @file ikClwindconWTCon.c
 * 
 * @brief Class ikClwindconWTCon implementation
 */

#include <stdlib.h>
#include <string.h>
#include "ikClwindconWTCon.h"

int ikClwindconWTCon_init(ikClwindconWTCon *self, const ikClwindconWTConParams *params) {
    int err;
	ikClwindconWTConParams params_ = *params;

	/* pass reference to collective pitch demand for use in gain scheduling */
	params_.collectivePitchControl.linearController.gainShedXVal = &(self->priv.collectivePitchDemand);

	/* pass reference to preferred torque for use in torque control */
	params_.torqueControl.setpointGenerator.preferredControlAction = &(self->priv.belowRatedTorque);

    /* pass on the member parameters */
    err = ikConLoop_init(&(self->priv.dtdamper), &(params_.drivetrainDamper));
    if (err) return -1;
    err = ikConLoop_init(&(self->priv.torquecon), &(params_.torqueControl));
    if (err) return -2;
    err = ikConLoop_init(&(self->priv.colpitchcon), &(params_.collectivePitchControl));
    if (err) return -3;
    err = ikTpman_init(&(self->priv.tpManager), &(params_.torquePitchManager));
    if (err) return -5;
	err = ikPowman_init(&(self->priv.powerManager), &(params_.powerManager));
	if (err) return -6;
	err = ikIpc_init(&(self->priv.ipc), &(params_.individualPitchControl));
	if (err) return -7;
	err = ikConLoop_init(&(self->priv.yawByIpc), &(params_.yawByIpc));
	if (err) return -8;
	err = ikSpdman_init(&(self->priv.speedSensorManager), &(params_.speedSensorManager));
	if (err) return -9;
    
    /* initialise feedback signals */
    self->priv.torqueFromTorqueCon = 0.0;
	self->priv.collectivePitchDemand = 0.0;

    return 0;
}

void ikClwindconWTCon_initParams(ikClwindconWTConParams *params) {
    /* pass on the member parameters */
    ikConLoop_initParams(&(params->collectivePitchControl));
    ikConLoop_initParams(&(params->drivetrainDamper));
    ikConLoop_initParams(&(params->torqueControl));
    ikTpman_initParams(&(params->torquePitchManager));
	ikPowman_initParams(&(params->powerManager));
	ikIpc_initParams(&(params->individualPitchControl));
	ikConLoop_initParams(&(params->yawByIpc));
	ikSpdman_initParams(&(params->speedSensorManager));
}

int ikClwindconWTCon_step(ikClwindconWTCon *self) {
	int i;
	
	/* run speed sensor manager */
	ikSpdman_step(&(self->priv.speedSensorManager), self->in.generatorSpeed, self->in.rotorSpeed, self->in.azimuth);
	ikSpdman_getOutput(&(self->priv.speedSensorManager), &(self->priv.generatorSpeedEquivalent), "generator speed equivalent");
	
	/* run power manager */
	self->priv.maxTorqueFromPowman = ikPowman_step(&(self->priv.powerManager), self->in.deratingRatio, self->in.maximumSpeed, self->priv.generatorSpeedEquivalent);
	ikPowman_getOutput(&(self->priv.powerManager), &(self->priv.minPitchFromPowman), "minimum pitch");
	ikPowman_getOutput(&(self->priv.powerManager), &(self->priv.belowRatedTorque), "below rated torque");

	/* calculate minimum pitch */
	self->priv.minPitch = self->priv.minPitchFromPowman > self->in.externalMinimumPitch ? self->priv.minPitchFromPowman : self->in.externalMinimumPitch;
	
	/* calculate maximum torque */
	self->priv.maxTorque = self->priv.maxTorqueFromPowman < self->in.externalMaximumTorque ? self->priv.maxTorqueFromPowman : self->in.externalMaximumTorque;

    /* run torque-pitch manager */
    self->priv.tpManState = ikTpman_step(&(self->priv.tpManager), self->priv.torqueFromTorqueCon, self->priv.maxTorque, self->in.externalMinimumTorque, self->priv.collectivePitchDemand, self->in.externalMaximumPitch, self->priv.minPitch);
    ikTpman_getOutput(&(self->priv.tpManager), &(self->priv.maxPitch), "maximum pitch");
    ikTpman_getOutput(&(self->priv.tpManager), &(self->priv.minTorque), "minimum torque");
	
    /* run drivetrain damper */
    self->priv.torqueFromDtdamper = ikConLoop_step(&(self->priv.dtdamper), 0.0, self->priv.generatorSpeedEquivalent, -(self->in.externalMaximumTorque), self->in.externalMaximumTorque);

    /* run torque control */
    self->priv.torqueFromTorqueCon = ikConLoop_step(&(self->priv.torquecon), self->in.maximumSpeed, self->priv.generatorSpeedEquivalent, self->priv.minTorque, self->priv.maxTorque);

    /* calculate torque demand */
    self->out.torqueDemand = self->priv.torqueFromDtdamper + self->priv.torqueFromTorqueCon;

    /* run collective pitch control */
    self->priv.collectivePitchDemand = ikConLoop_step(&(self->priv.colpitchcon), self->in.maximumSpeed, self->priv.generatorSpeedEquivalent, self->priv.minPitch, self->priv.maxPitch);

	/* run yaw by ipc */
	self->priv.individualPitchForYaw = ikConLoop_step(&(self->priv.yawByIpc), self->in.yawErrorReference, self->in.yawError, -self->in.maximumIndividualPitch, self->in.maximumIndividualPitch);

	/* run individual pitch control */
	self->priv.ipc.in.azimuth = self->in.azimuth;
	self->priv.ipc.in.collectivePitch = self->priv.collectivePitchDemand;
	self->priv.ipc.in.maximumPitch = self->priv.maxPitch;
	self->priv.ipc.in.minimumPitch = self->priv.minPitch;
	for (i = 0; i < 3; i++)	{
		self->priv.ipc.in.bladeRootMoments[i] = self->in.bladeRootMoments[i];
	}
	self->priv.ipc.in.demandedMy = 0.0;
	self->priv.ipc.in.demandedMz = 0.0;
	self->priv.ipc.in.maximumIndividualPitch = self->in.maximumIndividualPitch;
	self->priv.ipc.in.externalPitchY = self->priv.individualPitchForYaw;
	self->priv.ipc.in.externalPitchZ = 0.0;
	ikIpc_step(&(self->priv.ipc));
    
    /* run IPC */
    self->out.pitchDemandBlade1 = self->priv.ipc.out.pitch[0];
    self->out.pitchDemandBlade2 = self->priv.ipc.out.pitch[1];
    self->out.pitchDemandBlade3 = self->priv.ipc.out.pitch[2];

    return self->priv.tpManState;
}

int ikClwindconWTCon_getOutput(const ikClwindconWTCon *self, double *output, const char *name) {
    int err;
    const char *sep;
    
	/* pick up the signal names */
    if (!strcmp(name, "torque demand from torque control")) {
        *output = self->priv.torqueFromTorqueCon;
        return 0;
    }
    if (!strcmp(name, "torque demand from drivetrain damper")) {
        *output = self->priv.torqueFromDtdamper;
        return 0;
    }
    if (!strcmp(name, "minimum pitch")) {
        *output = self->priv.minPitch;
        return 0;
    }
    if (!strcmp(name, "maximum pitch")) {
        *output = self->priv.maxPitch;
        return 0;
    }
    if (!strcmp(name, "maximum torque")) {
        *output = self->priv.maxTorque;
        return 0;
    }
    if (!strcmp(name, "minimum torque")) {
        *output = self->priv.minTorque;
        return 0;
    }
    if (!strcmp(name, "collective pitch demand")) {
        *output = self->priv.collectivePitchDemand;
        return 0;
    }
    if (!strcmp(name, "maximum torque from power manager")) {
        *output = self->priv.maxTorqueFromPowman;
        return 0;
    }
    if (!strcmp(name, "minimum pitch from power manager")) {
        *output = self->priv.minPitchFromPowman;
        return 0;
    }
    if (!strcmp(name, "individual pitch for yaw")) {
        *output = self->priv.individualPitchForYaw;
        return 0;
    }
    if (!strcmp(name, "generator speed equivalent")) {
        *output = self->priv.generatorSpeedEquivalent;
        return 0;
    }

    /* pick up the block names */
    sep = strstr(name, ">");
    if (NULL == sep) return -1;
	if (!strncmp(name, "power manager", strlen(name) - strlen(sep))) {
        err = ikPowman_getOutput(&(self->priv.powerManager), output, sep + 1);
        if (err) return -1;
        else return 0;
    }
	if (!strncmp(name, "torque-pitch manager", strlen(name) - strlen(sep))) {
        err = ikTpman_getOutput(&(self->priv.tpManager), output, sep + 1);
        if (err) return -1;
        else return 0;
    }
	if (!strncmp(name, "drivetrain damper", strlen(name) - strlen(sep))) {
        err = ikConLoop_getOutput(&(self->priv.dtdamper), output, sep + 1);
        if (err) return -1;
        else return 0;
    }
	if (!strncmp(name, "torque control", strlen(name) - strlen(sep))) {
        err = ikConLoop_getOutput(&(self->priv.torquecon), output, sep + 1);
        if (err) return -1;
        else return 0;
    }
	if (!strncmp(name, "collective pitch control", strlen(name) - strlen(sep))) {
        err = ikConLoop_getOutput(&(self->priv.colpitchcon), output, sep + 1);
        if (err) return -1;
        else return 0;
    }
	if (!strncmp(name, "individual pitch control", strlen(name) - strlen(sep))) {
        err = ikIpc_getOutput(&(self->priv.ipc), output, sep + 1);
        if (err) return -1;
        else return 0;
    }
	if (!strncmp(name, "yaw by ipc", strlen(name) - strlen(sep))) {
        err = ikConLoop_getOutput(&(self->priv.yawByIpc), output, sep + 1);
        if (err) return -1;
        else return 0;
    }


    return -2;
}

