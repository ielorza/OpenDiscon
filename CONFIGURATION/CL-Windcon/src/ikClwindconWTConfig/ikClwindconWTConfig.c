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
 * @file ikClwindconWTConfig.c
 *
 * @brief CL-Windcon wind turbine controller configuration implementation
 */

#include "ikClwindconWTConfig.h"

void setParams(ikClwindconWTConParams *param) {
	/*! [Sampling interval] */
    /*
	####################################################################
                     Sampling interval

    Set sampling interval here:
	*/
	const double T = 0.01; /* [s] */
    /*
    ####################################################################
	*/
	/*! [Sampling interval] */

	ikTuneDrivetrainDamper(&(param->drivetrainDamper), T);
	ikTuneSpeedRange(&(param->torqueControl));
	ikTunePowerSettings(&(param->powerManager));
	ikTuneDeratingTorqueStrategy(&(param->powerManager));
	ikTuneDeratingPitchStrategy(&(param->powerManager));
	ikTunePitchPIGainSchedule(&(param->collectivePitchControl));
	ikTunePitchLowpassFilter(&(param->collectivePitchControl), T);
	ikTunePitchNotches(&(param->collectivePitchControl), T);
	ikTunePitchPI(&(param->collectivePitchControl), T);
	ikTuneTorqueLowpassFilter(&(param->torqueControl), T);
	ikTuneTorqueNotches(&(param->torqueControl), T);
	ikTuneTorquePI(&(param->torqueControl), T);
	ikConfigureRotorForIpc(&(param->individualPitchControl));
	ikTuneIpcMyPI(&(param->individualPitchControl.controlMy), T);
	ikTuneIpcMzPI(&(param->individualPitchControl.controlMz), T);
	ikTuneYawByIpc(&(param->yawByIpc), T);
	ikTuneYawByIpcLowpassFilter(&(param->yawByIpc), T);
	ikConfigureSpeedManager(&(param->speedSensorManager), T);

}

void ikTuneDrivetrainDamper(ikConLoopParams *params, double T) {

	/*! [Drivetrain damper] */
    /*
	####################################################################
                     Drivetrain damper

    Transfer function:

    D(s) = G*s*w^2/(s^2 + 2*d*w*s + w^2)

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    const double G = 0.0382; /* [kNm s^2/rad] 4 Nm s/rpm */
    const double d = 0.1; /* [-] */
    const double w = 21.1; /* [rad/s] */
    /*
    ####################################################################
	*/
	/*! [Drivetrain damper] */


    /*
	tune the drivetrain damper to this tf:
                       z^2 - 1
    D(z) = G*T/2*w^2 -------------------------------------------------------------------------------
                     (1 + T*d*w + T^2*w^2/4)*z^2 -2*(1 - T^2*w^2/4)*z + (1 - T*d*w + T^2*w^2/4)
    rad/s --> kNm
	*/
    params->linearController.errorTfs.tfParams[0].enable = 1;
    params->linearController.errorTfs.tfParams[0].b[0] = 1.0;
    params->linearController.errorTfs.tfParams[0].b[1] = 0.0;
    params->linearController.errorTfs.tfParams[0].b[2] = -1.0;
    params->linearController.errorTfs.tfParams[0].a[0] = 1.0 + T*d*w + T*T*w*w/4.0;
    params->linearController.errorTfs.tfParams[0].a[1] = -2.0*(1.0 - T*T*w*w/4.0);
    params->linearController.errorTfs.tfParams[0].a[2] = (1.0 - T*d*w + T*T*w*w/4.0);
    params->linearController.errorTfs.tfParams[1].enable = 1;
    params->linearController.errorTfs.tfParams[1].b[0] = -G*T/2.0*w*w;

}

void ikTuneSpeedRange(ikConLoopParams *params) {

	/*
	####################################################################
					 Variable generator speed range
					 
	Set parameters here:
	*/
	const double Wmin = 31.4159265358979; /* [rad/s] 300 rpm */
	const double Wmax = 50.2654824574367; /* [rad/s] 480 rpm */
	/*
	####################################################################
	*/

    params->setpointGenerator.nzones = 1;
    params->setpointGenerator.setpoints[0][0] = Wmin;
    params->setpointGenerator.setpoints[1][0] = Wmax;

}
	
void ikTunePowerSettings(ikPowmanParams *params) {
	
	/*
	####################################################################
					 Power settings
					 
	Set parameters here:
	*/
	const double Pn = 10.0e3; /* kW */
	const double eff = 0.94; /* - */
	/*
	####################################################################
	*/

	params->ratedPower = Pn;
	params->efficiency = eff;
}

void ikTuneDeratingTorqueStrategy(ikPowmanParams *params) {
/*
This is an original implementation of derating strategy 3a as described by ECN in deliverable D2.1 of H2020 project CL-Windcon.
*/

	int i;
	
	/*! [Optimum torque] */
    /*
	####################################################################
					 Below rated speed-torque curve

	Curve:

	Q = Kopt(dr) * w^2

	The default values for dr and Kopt have been kindly provided by ECN, who have calculated them to suit the DTU 10MW reference wind turbine from FP7 project INNWIND.
	Set parameters here:
	*/
	const int n = 11; /* number of points in the lookup table */
	const double dr[] = {0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50}; /* - */
	const double Kopt[] = {90.607511506848581, 86.115902720799966, 81.575353112422349, 77.050958297021111, 72.492888078483688, 68.064126426095299, 63.512773230238686, 58.970705560510474, 54.464434076487962, 49.891764181889293, 45.401884663773203}; /* Nm*s^2/rad^2 */
	/*
	####################################################################
	*/
	/*! [Optimum torque] */

	params->belowRatedTorqueGainTableN = n;
	for (i = 0; i < n; i++) {
		params->belowRatedTorqueGainTableX[i] = dr[i];
		params->belowRatedTorqueGainTableY[i] = Kopt[i]/1.0e3;
	}		
}

void ikTuneDeratingPitchStrategy(ikPowmanParams *params) {
/*
This is an original implementation of derating strategy 3a as described by ECN in deliverable D2.1 of H2020 project CL-Windcon.
*/

	int i;
	
	/*! [Minimum pitch] */
    /*
	####################################################################
					 Minimum pitch

	The default values have been kindly provided by ECN, who have calculated them to suit the DTU 10MW reference wind turbine from FP7 project INNWIND.
	Set parameters here:
	*/
	const int n = 11; /* number of points in the lookup table */
	const double dr[] = {0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50}; /* - */
	const double pitch[] = {0.00, 0.039449747839419, 0.058560350086376, 0.073725555631053, 0.086762305188347, 0.098108135965117, 0.108839079483571, 0.118773997213269, 0.128018250433713, 0.136903315900539, 0.145235569651071}; /* rad */
	/*
	####################################################################
	*/
	/*! [Minimum pitch] */

	params->minimumPitchTableN = n;
	for (i = 0; i < n; i++) {
		params->minimumPitchTableX[i] = dr[i];
		params->minimumPitchTableY[i] = pitch[i]/3.1416*180.0;
	}		
}

void ikTunePitchPIGainSchedule(ikConLoopParams *params) {
	int i;
	
	/*! [Gain schedule] */
    /*
	####################################################################
                     Pitch Gain Schedule

	Set parameters here:
	*/
	const int n = 11; /* number of points in the lookup table */
	const double pitch[] = {0.0, 3.8424, 5.6505, 8.1091, 11.6797, 14.5687, 17.1140, 19.4472, 21.6249, 23.6774, 25.0}; /* degrees */
	const double gain[] = {2.1000, 2.1000, 2.0727, 1.7182, 1.5182, 1.3545, 1.2636, 1.1909, 1.1182, 1.0545, 1.0545}; /* - */
	/*
    ####################################################################
	*/
	/*! [Gain schedule] */

	params->linearController.gainSchedN = n;

	for (i = 0; i < n; i++) {
		params->linearController.gainSchedX[i] = pitch[i];
		params->linearController.gainSchedY[i] = gain[i];
	}	
}

void ikTunePitchLowpassFilter(ikConLoopParams *params, double T) {

	/*! [Pitch lowpass filter] */
    /*
	####################################################################
                     Speed feedback low pass filter

    Transfer function (to be done twice - we want a 4th order filter):
    H(s) = w^2 / (s^2 + 2*d*w*s + w^2)

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    const double w = 5.6; /* [rad/s] */
    const double d = 0.5; /* [-] */
    /*
    ####################################################################
	*/
	/*! [Pitch lowpass filter] */

    /*
	tune the pitch control feedback filter to this tf (twice, mind you):
                   (0.5*T*w)^2                                                                     z^2 + 2z + 1
    H(z) =  -----------------------------   ------------------------------------------------------------------------------------------------------------------------
            1 + T*d*w +  (0.5*T*w)^2    z^2 - 2*(1 - (0.5*T*w)^2) / (1 + T*d*w +  (0.5*T*w)^2)z +  (1 - T*d*w +  (0.5*T*w)^2) / (1 + T*d*w +  (0.5*T*w)^2)
    */
	params->linearController.measurementTfs.tfParams[1].enable = 1;
    params->linearController.measurementTfs.tfParams[1].b[0] = 1.0;
    params->linearController.measurementTfs.tfParams[1].b[1] = 2.0;
    params->linearController.measurementTfs.tfParams[1].b[2] = 1.0;
    params->linearController.measurementTfs.tfParams[1].a[0] = 1.0;
    params->linearController.measurementTfs.tfParams[1].a[1] = -2 * (1 - (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));
    params->linearController.measurementTfs.tfParams[1].a[2] = (1 - T*d*w + (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));

    params->linearController.measurementTfs.tfParams[2].enable = 1;
    params->linearController.measurementTfs.tfParams[2].b[0] = 1.0;
    params->linearController.measurementTfs.tfParams[2].b[1] = 2.0;
    params->linearController.measurementTfs.tfParams[2].b[2] = 1.0;
    params->linearController.measurementTfs.tfParams[2].a[0] = 1.0;
    params->linearController.measurementTfs.tfParams[2].a[1] = -2 * (1 - (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));
    params->linearController.measurementTfs.tfParams[2].a[2] = (1 - T*d*w + (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));

    params->linearController.measurementTfs.tfParams[3].enable = 1;
    params->linearController.measurementTfs.tfParams[3].b[0] = ((0.5*T*w)*(0.5*T*w) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w))) * ((0.5*T*w)*(0.5*T*w) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w)));

}

void ikTunePitchNotches(ikConLoopParams *params, double T) {

	/*! [1st fore-aft tower mode filter] */
    /*
	####################################################################
                     1st fore-aft tower mode filter

    Transfer function:
    H(s) = (s^2 + 2*dnum*w*s + w^2) / (s^2 + 2*dden*w*s + w^2)

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    const double w = 1.59; /* [rad/s] */
    const double dnum = 0.01; /* [-] */
    const double dden = 0.2; /* [-] */
    /*
    ####################################################################
	*/
	/*! [1st fore-aft tower mode filter] */

    params->linearController.measurementNotches.dT = T;
    params->linearController.measurementNotches.notchParams[0].enable = 1;
    params->linearController.measurementNotches.notchParams[0].freq = w;
    params->linearController.measurementNotches.notchParams[0].dampNum = dnum;
    params->linearController.measurementNotches.notchParams[0].dampDen = dden;

}

void ikTunePitchPI(ikConLoopParams *params, double T) {

	/*! [Pitch PI] */
    /*
	####################################################################
                     Pitch PI

    Transfer function:

    C(s) = (Kp*s + Ki)/s

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    const double Kp = -0.3939; /* [degs/rad] 7.2e-4 rad/rpm */
    const double Ki = -0.1313; /* [deg/rad] 2.4e-4 rad/rpms */
    /*
    ####################################################################
	*/
	/*! [Pitch PI] */


	/*
	tune the speed control to this tf:
           (Kp + Ki*T/2)z - (Kp - Ki*T/2)
    C(z) = ------------------------------
                      z - 1
	rad/s --> deg
	*/
	params->linearController.errorTfs.tfParams[0].enable = 1;
    params->linearController.errorTfs.tfParams[0].b[0] = (Kp + Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[1] = -(Kp - Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[2] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[0] = 1.0;
    params->linearController.errorTfs.tfParams[0].a[1] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[2] = 0.0;

	params->linearController.postGainTfs.tfParams[0].enable = 1;
    params->linearController.postGainTfs.tfParams[0].b[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].b[1] = 0.0;
    params->linearController.postGainTfs.tfParams[0].b[2] = 0.0;
    params->linearController.postGainTfs.tfParams[0].a[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].a[1] = -1.0;
    params->linearController.postGainTfs.tfParams[0].a[2] = 0.0;

}

void ikTuneTorqueLowpassFilter(ikConLoopParams *params, double T) {

	/*! [Torque lowpass filter] */
    /*
	####################################################################
                    Speed feedback low pass filter

    Transfer function (to be done twice - we want a 4th order filter):
    H(s) = w^2 / (s^2 + 2*d*w*s + w^2)

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    const double w = 3.39; /* [rad/s] */
    const double d = 0.5; /* [-] */
    /*
    ####################################################################
	*/
	/*! [Torque lowpass filter] */

    /*
	tune the torque control feedback filter to this tf (twice, mind you):
                   (0.5*T*w)^2                                                                     z^2 + 2z + 1
    H(z) =  -----------------------------   ------------------------------------------------------------------------------------------------------------------------
            1 + T*d*w +  (0.5*T*w)^2    z^2 - 2*(1 - (0.5*T*w)^2) / (1 + T*d*w +  (0.5*T*w)^2)z +  (1 - T*d*w +  (0.5*T*w)^2) / (1 + T*d*w +  (0.5*T*w)^2)
	*/
    params->linearController.measurementTfs.tfParams[0].enable = 1;
    params->linearController.measurementTfs.tfParams[0].b[0] = 1.0;
    params->linearController.measurementTfs.tfParams[0].b[1] = 2.0;
    params->linearController.measurementTfs.tfParams[0].b[2] = 1.0;
    params->linearController.measurementTfs.tfParams[0].a[0] = 1.0;
    params->linearController.measurementTfs.tfParams[0].a[1] = -2 * (1 - (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));
    params->linearController.measurementTfs.tfParams[0].a[2] = (1 - T*d*w + (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));

    params->linearController.measurementTfs.tfParams[1].enable = 1;
    params->linearController.measurementTfs.tfParams[1].b[0] = 1.0;
    params->linearController.measurementTfs.tfParams[1].b[1] = 2.0;
    params->linearController.measurementTfs.tfParams[1].b[2] = 1.0;
    params->linearController.measurementTfs.tfParams[1].a[0] = 1.0;
    params->linearController.measurementTfs.tfParams[1].a[1] = -2 * (1 - (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));
    params->linearController.measurementTfs.tfParams[1].a[2] = (1 - T*d*w + (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));

    params->linearController.measurementTfs.tfParams[2].enable = 1;
    params->linearController.measurementTfs.tfParams[2].b[0] = ((0.5*T*w)*(0.5*T*w) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w))) * ((0.5*T*w)*(0.5*T*w) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w)));

}

void ikTuneTorqueNotches(ikConLoopParams *params, double T) {

	/*! [1st side-side tower mode filter] */
    /*
	####################################################################
                    1st side-side tower mode filter

    Transfer function:
    H(s) = (s^2 + 2*dnum*w*s + w^2) / (s^2 + 2*dden*w*s + w^2)

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    const double w = 1.59; /* [rad/s] */
    const double dnum = 0.01; /* [-] */
    const double dden = 0.2; /* [-] */
    /*
    ####################################################################
	*/
	/*! [1st side-side tower mode filter] */

    params->linearController.measurementNotches.dT = T;
    params->linearController.measurementNotches.notchParams[0].enable = 1;
    params->linearController.measurementNotches.notchParams[0].freq = w;
    params->linearController.measurementNotches.notchParams[0].dampNum = dnum;
    params->linearController.measurementNotches.notchParams[0].dampDen = dden;

}

void ikTuneTorquePI(ikConLoopParams *params, double T) {

	/*! [Torque PI] */
    /*
	####################################################################
                    Torque PI

    Transfer function:

    C(s) = (Kp*s + Ki)/s

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    const double Kp = -34.3775; /* [kNms/rad] 3600 Nm/rpm */
    const double Ki = -11.4592; /* [kNm/rad] 1200 Nm/rpms */
    /*
    ####################################################################
	*/
	/*! [Torque PI] */

	/*
	tune the torque control to this tf:
           (Kp + Ki*T/2)z - (Kp - Ki*T/2)
    C(z) = ------------------------------
                      z - 1
	rad/s --> kNm
	*/
	params->linearController.errorTfs.tfParams[0].enable = 1;
    params->linearController.errorTfs.tfParams[0].b[0] = (Kp + Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[1] = -(Kp - Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[2] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[0] = 1.0;
    params->linearController.errorTfs.tfParams[0].a[1] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[2] = 0.0;

	params->linearController.postGainTfs.tfParams[0].enable = 1;
    params->linearController.postGainTfs.tfParams[0].b[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].b[1] = 0.0;
    params->linearController.postGainTfs.tfParams[0].b[2] = 0.0;
    params->linearController.postGainTfs.tfParams[0].a[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].a[1] = -1.0;
    params->linearController.postGainTfs.tfParams[0].a[2] = 0.0;

}

void ikConfigureRotorForIpc(ikIpcParams *params) {

	params->azimuthOffset = 0.0;
	params->bladeOrder = 1;

}

void ikTuneIpcMyPI(ikConLoopParams *params, double T) {

	/*! [IPC My PI] */
    /*
	####################################################################
                    IPC My PI

    Transfer function:

    C(s) = (Kp*s + Ki)/s

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    const double Kp = 0.0; /* [deg/kNm] */
    const double Ki = -0.1e-3; /* [deg/kNms] */
    /*
    ####################################################################
	*/
	/*! [IPC My PI] */

	/*
	tune the ipc My control to this tf:
           (Kp + Ki*T/2)z - (Kp - Ki*T/2)
    C(z) = ------------------------------
                      z - 1
	rad/s --> kNm
	*/
	params->linearController.errorTfs.tfParams[0].enable = 1;
    params->linearController.errorTfs.tfParams[0].b[0] = (Kp + Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[1] = -(Kp - Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[2] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[0] = 1.0;
    params->linearController.errorTfs.tfParams[0].a[1] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[2] = 0.0;

	params->linearController.postGainTfs.tfParams[0].enable = 1;
    params->linearController.postGainTfs.tfParams[0].b[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].b[1] = 0.0;
    params->linearController.postGainTfs.tfParams[0].b[2] = 0.0;
    params->linearController.postGainTfs.tfParams[0].a[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].a[1] = -1.0;
    params->linearController.postGainTfs.tfParams[0].a[2] = 0.0;

}

void ikTuneIpcMzPI(ikConLoopParams *params, double T) {

	/*! [IPC Mz PI] */
    /*
	####################################################################
                    IPC Mz PI

    Transfer function:

    C(s) = (Kp*s + Ki)/s

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    const double Kp = 0.0; /* [deg/kNm] */
    const double Ki = -0.1e-3; /* [deg/kNms] */
    /*
    ####################################################################
	*/
	/*! [IPC Mz PI] */

	/*
	tune the ipc Mz control to this tf:
           (Kp + Ki*T/2)z - (Kp - Ki*T/2)
    C(z) = ------------------------------
                      z - 1
	rad/s --> kNm
	*/
	params->linearController.errorTfs.tfParams[0].enable = 1;
    params->linearController.errorTfs.tfParams[0].b[0] = (Kp + Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[1] = -(Kp - Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[2] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[0] = 1.0;
    params->linearController.errorTfs.tfParams[0].a[1] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[2] = 0.0;

	params->linearController.postGainTfs.tfParams[0].enable = 1;
    params->linearController.postGainTfs.tfParams[0].b[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].b[1] = 0.0;
    params->linearController.postGainTfs.tfParams[0].b[2] = 0.0;
    params->linearController.postGainTfs.tfParams[0].a[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].a[1] = -1.0;
    params->linearController.postGainTfs.tfParams[0].a[2] = 0.0;

}

void ikTuneYawByIpc(ikConLoopParams *params, double T) {
/*
This is an original implementation of the yaw by IPC strategy in 87e4a2fe8e8ac8fc51305a3f840e23a0deaf6caa of https://github.com/TUDelft-DataDrivenControl/DRC_Fortran
*/

	/*! [Yaw by IPC PI] */
    /*
	####################################################################
                    Yaw by IPC PI

    Transfer function:

    C(s) = (Kp*s + Ki)/s

    The sampling time is given by function parameter T.

    Set parameters here:
	*/
    const double Kp = 0.0; /* [-] */
    const double Ki = 0.0; /* [1/s] */
    /*
    ####################################################################
	*/
	/*! [Yaw by IPC PI] */

	/*
	tune the yaw by ipc control to this tf:
           (Kp + Ki*T/2)z - (Kp - Ki*T/2)
    C(z) = ------------------------------
                      z - 1
	deg --> deg
	*/
	params->linearController.errorTfs.tfParams[0].enable = 1;
    params->linearController.errorTfs.tfParams[0].b[0] = (Kp + Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[1] = -(Kp - Ki*T/2);
    params->linearController.errorTfs.tfParams[0].b[2] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[0] = 1.0;
    params->linearController.errorTfs.tfParams[0].a[1] = 0.0;
    params->linearController.errorTfs.tfParams[0].a[2] = 0.0;

	params->linearController.postGainTfs.tfParams[0].enable = 1;
    params->linearController.postGainTfs.tfParams[0].b[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].b[1] = 0.0;
    params->linearController.postGainTfs.tfParams[0].b[2] = 0.0;
    params->linearController.postGainTfs.tfParams[0].a[0] = 1.0;
    params->linearController.postGainTfs.tfParams[0].a[1] = -1.0;
    params->linearController.postGainTfs.tfParams[0].a[2] = 0.0;

}

void ikTuneYawByIpcLowpassFilter(ikConLoopParams *params, double T) {
/*
This is an original implementation of the yaw by IPC strategy in 87e4a2fe8e8ac8fc51305a3f840e23a0deaf6caa of https://github.com/TUDelft-DataDrivenControl/DRC_Fortran
*/

	/*! [Yaw by IPC lowpass filter] */
    /*
	####################################################################
                    Yaw error feedback low pass filter

    Transfer function:
    H(s) = w^2 / (s^2 + 2*d*w*s + w^2)

    The sampling time is given by function parameter T.

	The default values have been kindly provided by TUDelft, who have calculated them to suit the DTU 10MW reference wind turbine from FP7 project INNWIND.
    Set parameters here:
	*/
    const double w = 0.6283185; /* [rad/s] */
    const double d = 1.0; /* [-] */
    /*
    ####################################################################
	*/
	/*! [Yaw by IPC lowpass filter] */

    /*
	tune the yaw by ipc control feedback filter to this tf:
                   (0.5*T*w)^2                                                                     z^2 + 2z + 1
    H(z) =  -----------------------------   ------------------------------------------------------------------------------------------------------------------------
            1 + T*d*w +  (0.5*T*w)^2    z^2 - 2*(1 - (0.5*T*w)^2) / (1 + T*d*w +  (0.5*T*w)^2)z +  (1 - T*d*w +  (0.5*T*w)^2) / (1 + T*d*w +  (0.5*T*w)^2)
	*/
    params->linearController.measurementTfs.tfParams[0].enable = 1;
    params->linearController.measurementTfs.tfParams[0].b[0] = 1.0;
    params->linearController.measurementTfs.tfParams[0].b[1] = 2.0;
    params->linearController.measurementTfs.tfParams[0].b[2] = 1.0;
    params->linearController.measurementTfs.tfParams[0].a[0] = 1.0;
    params->linearController.measurementTfs.tfParams[0].a[1] = -2 * (1 - (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));
    params->linearController.measurementTfs.tfParams[0].a[2] = (1 - T*d*w + (0.5*T*w)*(0.5*T*w)) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));

    params->linearController.measurementTfs.tfParams[1].enable = 1;
    params->linearController.measurementTfs.tfParams[1].b[0] = (0.5*T*w)*(0.5*T*w) / (1 + T*d*w + (0.5*T*w)*(0.5*T*w));

}

void ikConfigureSpeedManager(ikSpdmanParams *params, double T) {
	
	/*! [Speed sensor manager] */
	/*
	####################################################################
	                    Speed sensor management

	Differences between the generator speed, rotor speed and azimuth derivative
	(the latter two multiplied by the gearbox ratio) are considered a fault if
	they are larger than tol for longer than N sampling intervals T.

	Set parameters here:
	*/
	const int N = 10; /* [-] */
	const double tol = 1.0; /* [rad/s] */
	const double gbRatio = 50.0; /* [-] */
    /*
    ####################################################################
	*/
	/*! [Speed sensor manager] */

	params->diagnoser.nStepsToFault = N;
	params->diagnoser.tolerance = tol;
	
	params->gearboxRatio = gbRatio;
	params->T = T;
	params->minAzimuth = 0.0;
	params->maxAzimuth = 360.0;
	
}
