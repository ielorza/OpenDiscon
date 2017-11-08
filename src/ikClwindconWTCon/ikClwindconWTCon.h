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
 * @file ikClwindconWTCon.h
 * 
 * @brief Class ikClwindconWTCon interface
 */

#ifndef IKCLWINDCONWTCON_H
#define IKCLWINDCONWTCON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ikConLoop.h"
#include "ikTpman.h"

    /**
     * @struct ikClwindconWTConInputs
     * @brief controller inputs
     */
    typedef struct ikClwindconWTConInputs {
        double externalMaximumTorque; /**<external maximum torque in kNm*/
        double externalMinimumTorque; /**<external minimum torque in kNm*/
        double externalMaximumPitch; /**<external maximum pitch in degrees*/
        double externalMinimumPitch; /**<external minimum pitch in degrees*/
        double maximumSpeed; /**<maximum generator speed setpoing in rad/s*/
        double generatorSpeed; /**<generator speed in rad/s*/
    } ikClwindconWTConInputs;

    /**
     * @struct ikClwindconWTConOutputs
     * @brief controller outputs
     */
    typedef struct ikClwindconWTConOutputs {
        double torqueDemand; /**<torque demand in kNm*/
        double pitchDemandBlade1; /**<pitch demand for blade 1 in degrees*/
        double pitchDemandBlade2; /**<pitch demand for blade 2 in degrees*/
        double pitchDemandBlade3; /**<pitch demand for blade 3 in degrees*/
    } ikClwindconWTConOutputs;

    /* @cond */

    typedef struct ikClwindconWTConPrivate {
        ikTpman   tpManager;
        ikConLoop dtdamper;
        ikConLoop torquecon;
        ikConLoop colpitchcon;
        double maxPitch;
        double minPitch;
        double maxSpeed;
        int tpManState;
        double minTorque;
        double torqueFromDtdamper;
        double torqueFromTorqueCon;
        double collectivePitchDemand;
		double (*preferredTorqueFcn)(double);
		double preferredTorque;
    } ikClwindconWTConPrivate;
    /* @endcond */

    /**
     * @struct ikClwindconWTCon
     * @brief Main controller class
     * 
     * This is and ad hoc wind turbine controller for CL-Windcon.
     * 
     * @par Inputs
	 * @li external maximum torque: externally set upper torque limit, specify via @link ikClwindconWTConInputs.externalMaximumTorque @endlink at @link in @endlink
	 * @li external minimum torque: externally set lower torque limit, specify via @link ikClwindconWTConInputs.externalMinimumTorque @endlink at @link in @endlink
	 * @li external maximum pitch: externally set upper pitch limit, specify via @link ikClwindconWTConInputs.externalMaximumPitch @endlink at @link in @endlink
	 * @li external minimum pitch: externally set lower pitch limit, specify via @link ikClwindconWTConInputs.externalMinimumPitch @endlink at @link in @endlink
     * @li maximum speed: maximum generator speed setpoint, specify via @link ikClwindconWTConInputs.maximumSpeed @endlink at @link in @endlink
     * @li generator speed: current generator speed, specify via @link ikClwindconWTConInputs.generatorSpeed @endlink at @link in @endlink
     * 
     * @par Outputs
     * @li torque demand: in kNm, get via @link ikClwindconWTConOutputs.torqueDemand @endlink at @link out @endlink
     * @li pitch demand for blade 1: in degrees, get via @link ikClwindconWTConOutputs.pitchDemandBlade1 @endlink at @link out @endlink
     * @li pitch demand for blade 2: in degrees, get via @link ikClwindconWTConOutputs.pitchDemandBlade2 @endlink at @link out @endlink
     * @li pitch demand for blade 3: in degrees, get via @link ikClwindconWTConOutputs.pitchDemandBlade3 @endlink at @link out @endlink
     * 
     * @par Unit block
     * 
     * @image html ikClwindconWTCon_unit_block.svg
     * 
     * @par Block diagram
     * 
     * @image html ikClwindconWTCon_block_diagram.svg
     * 
     * @par Public members
     * @li @link in @endlink inputs
     * @li @link out @endlink outputs
     * 
     * @par Methods
     * @li @link ikClwindconWTCon_initParams @endlink initialise initialisation parameter structure
     * @li @link ikClwindconWTCon_init @endlink initialise an instance
     * @li @link ikClwindconWTCon_step @endlink execute periodic calculations
     * @li @link ikClwindconWTCon_getOutput @endlink get output value
     * 
     */
    typedef struct ikClwindconWTCon {
        ikClwindconWTConInputs in; /**<inputs*/
        ikClwindconWTConOutputs out; /**<outputs*/
        /* @cond */
        ikClwindconWTConPrivate priv;
        /* @endcond */
    } ikClwindconWTCon;

    /**
     * @struct ikClwindconWTConParams
     * @brief controller initialisation parameters
     */
    typedef struct ikClwindconWTConParams {
        ikConLoopParams drivetrainDamper; /**<drivetrain damper initialisation parameters*/
        ikConLoopParams torqueControl; /**<torque control initialisation parameters*/
        ikConLoopParams collectivePitchControl; /**<collective pitch control initialisation parameters*/
        ikTpmanParams torquePitchManager; /**<torque-pitch manager inintialisation parameters*/
		double (*preferredTorqueFcn)(double); /**<pointer to function which, given the generator speed in rad/s, returns the preferred torque in kNm*/
    } ikClwindconWTConParams;

    /**
     * Initialise a controller instance
     * @param self instance
     * @param params initialisation parameters
     * @return error code:
     * @li 0: no error
     * @li -1: drivetrain damper initialisation failed
     * @li -2: torque control initialisation failed
     * @li -3: collective pitch control initialisation failed
     * @li -5: torque-pitch manager initialisation failed
     */
    int ikClwindconWTCon_init(ikClwindconWTCon *self, const ikClwindconWTConParams *params);

    /**
     * Initialise initialisation parameter structure
     * @param params initialisation parameter structure
     */
    void ikClwindconWTCon_initParams(ikClwindconWTConParams *params);

    /**
     * Execute periodic calculations
     * @param self controller instance
     * @return state
	 * @li 0: below rated
	 * @li 1: above rated
     */
    int ikClwindconWTCon_step(ikClwindconWTCon *self);

    /**
     * Get output value by name. All signals named on the block diagram of
     * @link ikClwindconWTCon @endlink are accessible, except for inputs and outputs,
     * which are available at @link ikClwindconWTCon.in @endlink and
     * @link ikClwindconWTCon.out @endlink, respectively. To refer to sub-block
     * signals, use the sub-block name followed by a ">" character and the
     * signal name. For example:
     * @li to access the torque demand from the drivetrain damper, use "torque demand from drivetrain damper"
     * @li to access the torque control control action, use "torque control>control action"
     * 
     * @param self controller instance
     * @param output output value
     * @param name output name, NULL terminated string
     * @return error code:
     * @li 0: no error
     * @li -1: invalid signal name
     * @li -2: invalid block name
     */
    int ikClwindconWTCon_getOutput(const ikClwindconWTCon *self, double *output, const char *name);



#ifdef __cplusplus
}
#endif

#endif /* IKCLWINDCONWTCON_H */

