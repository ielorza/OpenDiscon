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
 * @file ikStaticon.h
 * 
 * @brief Class ikStaticon interface
 */

#ifndef IKSTATICON_H
#define IKSTATICON_H

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * @struct ikStaticonInputs
     * @brief controller inputs
     */
    typedef struct ikStaticonInputs {
        double generatorSpeed; /**<generator speed in rad/s*/
    } ikStaticonInputs;

    /**
     * @struct ikStaticonOutputs
     * @brief controller outputs
     */
    typedef struct ikStaticonOutputs {
        double torqueDemand; /**<torque demand in kNm*/
        double pitchDemand; /**<pitch demand in degrees*/
    } ikStaticonOutputs;

    /* @cond */

    typedef struct ikStaticonPrivate {
        double maximumTorque;
        double minimumTorque;
        double maximumPitch;
        double minimumPitch;
        double maximumSpeed;
        double minimumSpeed;
        double pitchStep;
        double torqueStep;
        int torqueN;
        int torqueStepSign;
	int constantTorqueCounter;
    } ikStaticonPrivate;
    /* @endcond */

    /**
     * @struct ikStaticon
     * @brief Main controller class
     * 
     * This is and ad hoc wind turbine controller for quasi-static analysis.
     * 
     * @par Inputs
     * @li generator speed: current generator speed, in rad/s, specify via @link ikStaticonInputs.generatorSpeed @endlink at @link in @endlink
     * 
     * @par Outputs
     * @li torque demand: in kNm, get via @link ikStaticonOutputs.torqueDemand @endlink at @link out @endlink
     * 
     * @par Unit block
     * 
     * @image html ikStaticon_unit_block.svg
     * 
     * @par Block diagram
     * 
     * @image html ikStaticon_block_diagram.svg
     * 
     * @par Public members
     * @li @link in @endlink inputs
     * @li @link out @endlink outputs
     * 
     * @par Methods
     * @li @link ikStaticon_initParams @endlink initialise initialisation parameter structure
     * @li @link ikStaticon_init @endlink initialise an instance
     * @li @link ikStaticon_step @endlink execute periodic calculations
     * @li @link ikStaticon_getOutput @endlink get output value
     * 
     */
    typedef struct ikStaticon {
        ikStaticonInputs in; /**<inputs*/
        ikStaticonOutputs out; /**<outputs*/
        /* @cond */
        ikStaticonPrivate priv;
        /* @endcond */
    } ikStaticon;

    /**
     * @struct ikStaticonParams
     * @brief controller initialisation parameters
     */
    typedef struct ikStaticonParams {
        double maximumTorque; /**<maximum torque in kNm*/
        double minimumTorque; /**<minimum torque in kNm*/
        double maximumPitch; /**<maximum pitch in degrees*/
        double minimumPitch; /**<minimum pitch in degrees*/
        double maximumSpeed; /**<maximum generator speed in rad/s*/
        double minimumSpeed; /**<minimum generator speed in rad/s*/
        double pitchStep; /**<size of pitch steps in degrees*/
        double torqueStep; /**<size of torque steps in kN*/
        int torqueN; /**<number of steps at constant torque*/
    } ikStaticonParams;

    /**
     * Initialise a controller instance
     * @param self instance
     * @param params initialisation parameters
     * @return error code:
     * @li 0: no error
     * @li -1: bad torque interval
     * @li -2: bad pitch interval
     * @li -3: bad speed interval
     * @li -4: negative pitch step
     * @li -5: negative torque step
     * @li -6: negative constant torque step number
     */
    int ikStaticon_init(ikStaticon *self, const ikStaticonParams *params);

    /**
     * Initialise initialisation parameter structure
     * @param params initialisation parameter structure
     */
    void ikStaticon_initParams(ikStaticonParams *params);

    /**
     * Execute periodic calculations
     * @param self controller instance
     * @return state
         * @li 0: below rated
         * @li 1: above rated
     */
    int ikStaticon_step(ikStaticon *self);

    /**
     * Get output value by name. All signals named on the block diagram of
     * @link ikStaticon @endlink are accessible, except for inputs and outputs,
     * which are available at @link ikStaticon.in @endlink and
     * @link ikStaticon.out @endlink, respectively.
     * 
     * @param self controller instance
     * @param output output value
     * @param name output name, NULL terminated string
     * @return error code:
     * @li 0: no error
     * @li -1: invalid signal name
     */
    int ikStaticon_getOutput(const ikStaticon *self, double *output, const char *name);

#ifdef __cplusplus
}
#endif

#endif /* IKSTATICON_H */

