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
 * @file ikSpdman.h
 * 
 * @brief Class ikSpdman interface
 */

#ifndef IKSPDMAN_H
#define IKSPDMAN_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include "ikSensorDiagnoser.h"
    
    /**
     * @struct ikSpdman
     * @brief Speed sensor manager
     * 
     * This takes the generator speed, rotor speed and azimuth angle measurements, cross-checks them and outputs a usable generator speed signal when possible.
     * 
     * @par Inputs
     * @li generator speed: generator speed in rad/s, specify via @link ikSpdman_step @endlink
     * @li rotor speed: rotor speed in rad/s, specify via @link ikSpdman_step @endlink
	 * @li azimuth: rotor azimuth angle in degrees, specify via @link ikSpdman_step @endlink
     * 
     * @par Outputs
     * @li generator speed equivalent: generator speed for use by controller, in rad/s, get via @link ikSpdman_getOutput @endlink
     * @li status: status code, returned by @link ikSpdman_step @endlink, alternatively get via @link ikSpdman_getOutput @endlink
     * 
     * @par Unit block
     * 
     * @image html ikSpdman_unit_block.svg
     * 
     * @par Block diagram
     * 
     * @image html ikSpdman_block_diagram.svg
     * 
     * @par State machine
     * 
     * @image html ikSpdman_state_machine.svg
     * 
     * @par Methods
     * @li @link ikSpdman_initParams @endlink initialise initialisation parameter structure
     * @li @link ikSpdman_init @endlink initialise an instance
     * @li @link ikSpdman_step @endlink execute periodic calculations
     * @li @link ikSpdman_getOutput @endlink get output value
     */
    typedef struct ikSpdman {
        /**
         * Private members
         */
        /* @cond */
        ikSensorDiagnoser diagnoser;
		double gbratio;
		double T;
		double azimuthRange;
		double lastAzimuth;
		double signals[3];
		int ok[3];
		double outputSpeed;
		int status;
        /* @endcond */
    } ikSpdman;
    
    /**
     * @struct ikSpdmanParams
     * @brief Speed sensor manager initialisation parameters
     */
    typedef struct ikSpdmanParams {
		ikSensorDiagnoserParams diagnoser; /**<diagnoser initialisation parameters*/
		double gearboxRatio; /**<gearbox ratio, dimensionless*/
		double T; /**<sampling interval in s*/
		double minAzimuth; /**<minimum azimuth angle value, in degrees*/
		double maxAzimuth; /**<maximum azimuth angle value, in degrees*/
    } ikSpdmanParams;
    
    /**
     * Initialise an instance
     * @param self instance
     * @param params initialisation parameters
     * @return error code:
     * @li 0: no error
	 * @li -1: invalid diagnoser initialisation parameters
	 * @li -2: invalid sampling interval, must be positive
	 * @li -3: invalid azimuth limits, maximum must be larger than minimum
     */
    int ikSpdman_init(ikSpdman *self, const ikSpdmanParams *params);
    
    /**
     * Initialise initialisation parameter structure
     * @param params initialisation parameter structure
     */
    void ikSpdman_initParams(ikSpdmanParams *params);
    
    /**
	 * Execute periodic calculations
	 * @param self instance
	 * @param generatorSpeed generator speed, in rad/s
	 * @param rotorSpeed rotor speed, in rad/s
	 * @param azimuth rotor azimuth angle, in degrees
	 * @return status:
	 * @li 0: all sensors agree
	 * @li -1: bad generator speed
	 * @li -2: bad rotor speed
	 * @li -3: bad azimuth
	 * @li -4: two or more bad signals
	 */
    int ikSpdman_step(ikSpdman *self, double generatorSpeed, double rotorSpeed, double azimuth);
    
    /**
     * Get output value by name. All signals named on the block diagram of
     * @link ikSpdman @endlink are accessible.
     * @param self instance
     * @param output output value
     * @param name output name
     * @return error code:
     * @li 0: no error
     * @li -1: invalid signal name
     * @li -2: invalid block name
     */
    int ikSpdman_getOutput(const ikSpdman *self, double *output, const char *name);


#ifdef __cplusplus
}
#endif

#endif /* IKSPDMAN_H */

