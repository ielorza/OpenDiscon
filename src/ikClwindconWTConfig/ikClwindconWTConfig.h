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
 * @file ikClwindconWTConfig.h
 * 
 * @brief CL-Windcon wind turbine controller configuration interface
 */

#ifndef IKCLWINDCONWTCONFIG_H
#define IKCLWINDCONWTCONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ikClwindconWTCon.h"  

	void setParams(ikClwindconWTConParams *param);
	
	void ikTuneDrivetrainDamper(ikConLoopParams *params, double T);
	
	void ikTuneSpeedRange(ikConLoopParams *params);
	
	void ikTunePowerSettings(ikPowmanParams *params);
	
	void ikTuneDeratingTorqueStrategy(ikPowmanParams *params);
	
	void ikTuneDeratingPitchStrategy(ikPowmanParams *params);
	
	void ikTunePitchLowpassFilter(ikConLoopParams *params, double T);
	
	void ikTunePitchNotches(ikConLoopParams *params, double T);
	
	void ikTunePitchPI(ikConLoopParams *params, double T);
	
	void ikTuneTorqueLowpassFilter(ikConLoopParams *params, double T);
	
	void ikTuneTorqueNotches(ikConLoopParams *params, double T);
	
	void ikTuneTorquePI(ikConLoopParams *params, double T);

	void ikTunePitchPIGainSchedule(ikConLoopParams *params);

#ifdef __cplusplus
}
#endif

#endif /* IKCLWINDCONWTCONFIG_H */

