/*
  Copyright (C) 2017, 2021 IKERLAN

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
 * @file ikSimpleWTCon.h
 * 
 * @brief Class ikSimpleWTCon interface
 */

#ifndef IKSIMPLEWTCON_H
#define IKSIMPLEWTCON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ikConLoop.h"
#include "ikTpman.h"
#include "ikPowman.h"

    /**
     * @struct ikSimpleWTConInputs
     * @brief controller inputs
     */
    typedef struct ikSimpleWTConInputs {
        double externalMaximumTorque; /**<external maximum torque in kNm*/
        double externalMinimumTorque; /**<external minimum torque in kNm*/
        double externalMaximumPitch; /**<external maximum pitch in degrees*/
        double externalMinimumPitch; /**<external minimum pitch in degrees*/
        double externalMaximumPitchRate; /**<external maximum pitch rate in degrees per second*/
        double externalMinimumPitchRate; /**<external minimum pitch rate in degrees per second*/
        double maximumSpeed; /**<maximum generator speed setpoing in rad/s*/
        double generatorSpeed; /**<generator speed in rad/s*/
    } ikSimpleWTConInputs;

    /**
     * @struct ikSimpleWTConOutputs
     * @brief controller outputs
     */
    typedef struct ikSimpleWTConOutputs {
        double torqueDemand; /**<torque demand in kNm*/
        double pitchDemand; /**<pitch demand in degrees*/
    } ikSimpleWTConOutputs;

    /* @cond */

    typedef struct ikSimpleWTConPrivate {
        ikPowman powerManager;
        ikTpman   tpManager;
        ikConLoop dtdamper;
        ikConLoop torquecon;
        ikConLoop colpitchcon;
        double maxPitch;
        double minPitch;
        double maxSpeed;
        int tpManState;
        double maxTorque;
        double minTorque;
        double torqueFromDtdamper;
        double torqueFromTorqueCon;
        double belowRatedTorque;
        double minPitchFromPowman;
        double maxTorqueFromPowman;
    } ikSimpleWTConPrivate;
    /* @endcond */

    /**
     * @struct ikSimpleWTCon
     * @brief Main controller class
     * 
     * This is a simple wind turbine controller
     * 
     * @par Inputs
     * @li external maximum torque: externally set upper torque limit, in kNm, specify via @link ikSimpleWTConInputs.externalMaximumTorque @endlink at @link in @endlink
     * @li external minimum torque: externally set lower torque limit, in kNm, specify via @link ikSimpleWTConInputs.externalMinimumTorque @endlink at @link in @endlink
     * @li external maximum pitch: externally set upper pitch limit, in degrees, specify via @link ikSimpleWTConInputs.externalMaximumPitch @endlink at @link in @endlink
     * @li external minimum pitch: externally set lower pitch limit, in degrees, specify via @link ikSimpleWTConInputs.externalMinimumPitch @endlink at @link in @endlink
     * @li external maximum pitch rate: externally set upper pitch rate limit, in degrees per second, specify via @link ikSimpleWTConInputs.externalMaximumPitchRate @endlink at @link in @endlink
     * @li external minimum pitch rate: externally set lower pitch rate limit, in degrees per second, specify via @link ikSimpleWTConInputs.externalMinimumPitchRate @endlink at @link in @endlink
     * @li maximum speed: maximum generator speed setpoint, in rad/s, specify via @link ikSimpleWTConInputs.maximumSpeed @endlink at @link in @endlink
     * @li generator speed: current generator speed, in rad/s, specify via @link ikSimpleWTConInputs.generatorSpeed @endlink at @link in @endlink
     * 
     * @par Outputs
     * @li torque demand: in kNm, get via @link ikSimpleWTConOutputs.torqueDemand @endlink at @link out @endlink
     * @li pitch demand in degrees, get via @link ikSimpleWTConOutputs.pitchDemand @endlink at @link out @endlink
     * 
     * @par Unit block
     * 
     * @image html ikSimpleWTCon_unit_block.svg
     * 
     * @par Block diagram
     * 
     * @image html ikSimpleWTCon_block_diagram.svg
     * 
     * @par Public members
     * @li @link in @endlink inputs
     * @li @link out @endlink outputs
     * 
     * @par Methods
     * @li @link ikSimpleWTCon_initParams @endlink initialise initialisation parameter structure
     * @li @link ikSimpleWTCon_init @endlink initialise an instance
     * @li @link ikSimpleWTCon_step @endlink execute periodic calculations
     * @li @link ikSimpleWTCon_getOutput @endlink get output value
     * 
     */
    typedef struct ikSimpleWTCon {
        ikSimpleWTConInputs in; /**<inputs*/
        ikSimpleWTConOutputs out; /**<outputs*/
        /* @cond */
        ikSimpleWTConPrivate priv;
        /* @endcond */
    } ikSimpleWTCon;

    /**
     * @struct ikSimpleWTConParams
     * @brief controller initialisation parameters
     */
    typedef struct ikSimpleWTConParams {
        ikConLoopParams drivetrainDamper; /**<drivetrain damper initialisation parameters*/
        ikConLoopParams torqueControl; /**<torque control initialisation parameters*/
        ikConLoopParams collectivePitchControl; /**<collective pitch control initialisation parameters*/
        ikTpmanParams torquePitchManager; /**<torque-pitch manager inintialisation parameters*/
        ikPowmanParams powerManager; /**<power manager initialisation parameters*/
    } ikSimpleWTConParams;

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
     * @li -6: power manager initialisation failed
     */
    int ikSimpleWTCon_init(ikSimpleWTCon *self, const ikSimpleWTConParams *params);

    /**
     * Initialise initialisation parameter structure
     * @param params initialisation parameter structure
     */
    void ikSimpleWTCon_initParams(ikSimpleWTConParams *params);

    /**
     * Execute periodic calculations
     * @param self controller instance
     * @return state
     * @li 0: below rated
     * @li 1: above rated
     */
    int ikSimpleWTCon_step(ikSimpleWTCon *self);

    /**
     * Get output value by name. All signals named on the block diagram of
     * @link ikSimpleWTCon @endlink are accessible, except for inputs and outputs,
     * which are available at @link ikSimpleWTCon.in @endlink and
     * @link ikSimpleWTCon.out @endlink, respectively. To refer to sub-block
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
    int ikSimpleWTCon_getOutput(const ikSimpleWTCon *self, double *output, const char *name);



#ifdef __cplusplus
}
#endif

#endif /* IKSIMPLEWTCON_H */

