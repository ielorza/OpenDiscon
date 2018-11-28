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
 * @file ikTpman.h
 * 
 * @brief Class ikTpman interface
 */

#ifndef IKTPMAN_H
#define IKTPMAN_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include "ikLutbl.h"
    
    /**
     * @struct ikTpman
     * @brief Torque-pitch manager
     * 
     * This is the above-below rated state machine.
     * 
     * @par Inputs
     * @li torque: generator torque in kN, specify via @link ikTpman_step @endlink
     * @li maximum torque: upper torque limit for speed regulation, in kN, specify via @link ikTpman_step @endlink
     * @li external minimum torque: externally imposed lower torque limit for speed regulation, in kN, specify via @link ikTpman_step @endlink
     * @li pitch: pitch angle due to speed regulation loop, in degrees, specify via @link ikTpman_step @endlink
     * @li external maximum pitch: externally imposed upper pitch angle limit, in degrees, specify via @link ikTpman_step @endlink
     * @li external minimum pitch: externally imposed lower pitch angle limit, in degrees, specify via @link ikTpman_step @endlink
     * 
     * @par Outputs
     * @li maximum pitch: upper pitch angle limit, in degrees, get via @link ikTpman_getOutput @endlink
     * @li minimum torque: lower torque limit for speed regulation, in kNm, get via @link ikTpman_getOutput @endlink
     * 
     * @par Unit block
     * 
     * @image html ikTpman_unit_block.svg
     * 
     * @par Block diagram
     * 
     * @image html ikTpman_block_diagram.svg
     * 
     * @par State machine
     * 
     * @image html ikTpman_state_machine.svg
     * 
     * @par Methods
     * @li @link ikTpman_initParams @endlink initialise initialisation parameter structure
     * @li @link ikTpman_init @endlink initialise an instance
     * @li @link ikTpman_step @endlink execute periodic calculations
     * @li @link ikTpman_getOutput @endlink get output value
     */
    typedef struct ikTpman {
        /**
         * Private members
         */
        /* @cond */
        int state;
        double minTorque;
        double maxPitch;
        ikLutbl * minPitchTbl;
        double maxPitchExt;
        double minPitchExt;
        double torque;
        double pitch;
        double minTorqueExt;
        double maxTorque;
        /* @endcond */
    } ikTpman;
    
    /**
     * @struct ikTpmanParams
     * @brief Torque-pitch manager initialisation parameters
     */
    typedef struct ikTpmanParams {
		/* @cond */
		int foo;
		/* @endcond */
    } ikTpmanParams;
    
    /**
     * Initialise an instance
     * @param self instance
     * @param params initialisation parameters
     * @return error code:
     * @li 0: no error
     */
    int ikTpman_init(ikTpman *self, const ikTpmanParams *params);
    
    /**
     * Initialise initialisation parameter structure
     * @param params initialisation parameter structure
     */
    void ikTpman_initParams(ikTpmanParams *params);
    
    /**
     * Execute periodic calculations
     * @param self torque-pitch manager instance
     * @param torque generator torque in kN
     * @param maxTorque upper torque limit for speed regulation, in kN
     * @param minTorqueExt externally imposed lower torque limit for speed regulation, in kN
     * @param pitch pitch angle, in degrees
     * @param maxPitchExt externally imposed upper pitch angle limit, in degrees
     * @param minPitchExt externally imposed lower pitch angle limit, in degrees
     * @return state
     * @li 0: below rated
     * @li 1: above rated
     */
    int ikTpman_step(ikTpman *self, double torque, double maxTorque, double minTorqueExt, double pitch, double maxPitchExt, double minPitchExt);
    
    /**
     * Get output value by name. All signals named on the block diagram of
     * @link ikTpman @endlink are accessible.
     * @param self torque-pitch manager instance
     * @param output output value
     * @param name output name
     * @return error code:
     * @li 0: no error
     * @li -1: invalid signal name
     * @li -2: invalid block name
     */
    int ikTpman_getOutput(const ikTpman *self, double *output, const char *name);


#ifdef __cplusplus
}
#endif

#endif /* IKTPMAN_H */

