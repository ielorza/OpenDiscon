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
	const double eff = 1.0; /* - */
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

	These default values for dr and Kopt have been calculated for DTU's 10MW FAST model.
	Set parameters here:
	*/
	const int n = 121; /* number of points in the lookup table */
	const double dr[] = {0.000000, 0.000010, 0.000020, 0.000030, 0.000040, 0.000050, 0.000060, 0.000070, 0.000080, 0.000090, 0.000100, 0.000110, 0.000120, 0.000130, 0.000140, 0.000150, 0.000160, 0.000170, 0.000180, 0.000190, 0.000200, 0.000210, 0.000220, 0.000230, 0.000240, 0.000250, 0.000260, 0.000270, 0.000280, 0.000290, 0.000300, 0.000310, 0.000320, 0.000330, 0.000340, 0.000350, 0.000360, 0.000370, 0.000380, 0.000390, 0.000400, 0.000410, 0.000420, 0.000430, 0.000440, 0.000450, 0.000460, 0.000470, 0.000480, 0.000490, 0.000500, 0.000510, 0.000520, 0.000530, 0.000540, 0.000550, 0.000560, 0.000570, 0.000580, 0.000590, 0.000600, 0.000610, 0.000620, 0.000630, 0.000640, 0.000650, 0.000660, 0.000670, 0.000680, 0.000690, 0.000700, 0.000710, 0.000720, 0.000730, 0.000740, 0.000750, 0.000760, 0.000770, 0.000780, 0.000790, 0.000800, 0.000810, 0.000820, 0.000830, 0.000840, 0.000850, 0.000860, 0.000870, 0.000880, 0.000890, 0.000900, 0.000910, 0.000920, 0.000930, 0.000940, 0.000950, 0.000960, 0.000970, 0.000980, 0.000990, 0.001000, 0.001010, 0.001020, 0.001030, 0.001040, 0.001050, 0.001060, 0.001070, 0.001080, 0.001090, 0.001100, 0.001110, 0.001120, 0.001130, 0.001140, 0.001150, 0.001160, 0.001170, 0.001180, 0.001190, 0.001200, }; /* - */
	const double Kopt[] = {103.278666, 102.409365, 101.540064, 100.670763, 99.801462, 98.932161, 98.062860, 97.193558, 96.324257, 95.454956, 94.585655, 93.716354, 92.847053, 91.977752, 91.108451, 90.239150, 89.369849, 88.500548, 87.631246, 86.761945, 85.892644, 85.023343, 84.154042, 83.284741, 82.415440, 81.546139, 80.676838, 79.807537, 78.938236, 78.068934, 77.199633, 76.330332, 75.461031, 74.591730, 73.722429, 72.853128, 71.983827, 71.114526, 70.245225, 69.375924, 68.506622, 67.637321, 66.768020, 65.898719, 65.029418, 64.160117, 63.290816, 62.421515, 61.552214, 60.682913, 59.813612, 58.944310, 58.075009, 57.205708, 56.336407, 55.467106, 54.597805, 53.728504, 52.859203, 51.989902, 51.120601, 50.251300, 49.381998, 48.512697, 47.643396, 46.774095, 45.904794, 45.035493, 44.166192, 43.296891, 42.427590, 41.558289, 40.688988, 39.819686, 38.950385, 38.081084, 37.211783, 36.342482, 35.473181, 34.603880, 33.734579, 32.865278, 31.995977, 31.126676, 30.257374, 29.388073, 28.518772, 27.649471, 26.780170, 25.910869, 25.041568, 24.172267, 23.302966, 22.433665, 21.564364, 20.695062, 19.825761, 18.956460, 18.087159, 17.217858, 16.348557, 15.479256, 14.609955, 13.740654, 12.871353, 12.002052, 11.132750, 10.263449, 9.394148, 8.524847, 7.655546, 6.786245, 5.916944, 5.047643, 4.178342, 3.309041, 2.439740, 1.570438, 0.701137, -0.168164, -1.037465, }; /* Nm*s^2/rad^2 */
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

	These default values have been calculated for DTU's 10MW FAST model.
	Set parameters here:
	*/
	const int n = 121; /* number of points in the lookup table */
	const double dr[] = {0.000000, 0.000010, 0.000020, 0.000030, 0.000040, 0.000050, 0.000060, 0.000070, 0.000080, 0.000090, 0.000100, 0.000110, 0.000120, 0.000130, 0.000140, 0.000150, 0.000160, 0.000170, 0.000180, 0.000190, 0.000200, 0.000210, 0.000220, 0.000230, 0.000240, 0.000250, 0.000260, 0.000270, 0.000280, 0.000290, 0.000300, 0.000310, 0.000320, 0.000330, 0.000340, 0.000350, 0.000360, 0.000370, 0.000380, 0.000390, 0.000400, 0.000410, 0.000420, 0.000430, 0.000440, 0.000450, 0.000460, 0.000470, 0.000480, 0.000490, 0.000500, 0.000510, 0.000520, 0.000530, 0.000540, 0.000550, 0.000560, 0.000570, 0.000580, 0.000590, 0.000600, 0.000610, 0.000620, 0.000630, 0.000640, 0.000650, 0.000660, 0.000670, 0.000680, 0.000690, 0.000700, 0.000710, 0.000720, 0.000730, 0.000740, 0.000750, 0.000760, 0.000770, 0.000780, 0.000790, 0.000800, 0.000810, 0.000820, 0.000830, 0.000840, 0.000850, 0.000860, 0.000870, 0.000880, 0.000890, 0.000900, 0.000910, 0.000920, 0.000930, 0.000940, 0.000950, 0.000960, 0.000970, 0.000980, 0.000990, 0.001000, 0.001010, 0.001020, 0.001030, 0.001040, 0.001050, 0.001060, 0.001070, 0.001080, 0.001090, 0.001100, 0.001110, 0.001120, 0.001130, 0.001140, 0.001150, 0.001160, 0.001170, 0.001180, 0.001190, 0.001200, }; /* - */
	const double pitch[] = {-0.005934, 0.014136, 0.022397, 0.028856, 0.034388, 0.039305, 0.043829, 0.047990, 0.051895, 0.055589, 0.059091, 0.062437, 0.065642, 0.068736, 0.071741, 0.074638, 0.077465, 0.080213, 0.082883, 0.085492, 0.088011, 0.090453, 0.092895, 0.095338, 0.097651, 0.099918, 0.102184, 0.104451, 0.106593, 0.108718, 0.110842, 0.112967, 0.115009, 0.117026, 0.119043, 0.121061, 0.123034, 0.124952, 0.126870, 0.128788, 0.130706, 0.132547, 0.134379, 0.136212, 0.138044, 0.139865, 0.141611, 0.143357, 0.145103, 0.146849, 0.148585, 0.150257, 0.151928, 0.153599, 0.155271, 0.156942, 0.158559, 0.160172, 0.161785, 0.163398, 0.165011, 0.166598, 0.168161, 0.169724, 0.171286, 0.172849, 0.174412, 0.175920, 0.177423, 0.178927, 0.180430, 0.181934, 0.183428, 0.184855, 0.186282, 0.187709, 0.189136, 0.190563, 0.191990, 0.193369, 0.194748, 0.196127, 0.197506, 0.198885, 0.200264, 0.201618, 0.202959, 0.204300, 0.205642, 0.206983, 0.208324, 0.209659, 0.210962, 0.212266, 0.213569, 0.214872, 0.216175, 0.217478, 0.218766, 0.220036, 0.221306, 0.222576, 0.223846, 0.225116, 0.226386, 0.227633, 0.228865, 0.230097, 0.231329, 0.232561, 0.233793, 0.235026, 0.236242, 0.237443, 0.238645, 0.239847, 0.241048, 0.242250, 0.243451, 0.244646, 0.245820, }; /* rad */
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
	const int n = 10; /* number of points in the lookup table */
	const double pitch[] = {0.0, 3.8424, 5.6505, 8.1091, 11.6797, 14.5687, 17.1140, 19.4472, 21.6249, 23.6774}; /* degrees */
	const double gain[] = {1.7783, 1.7783, 6.0812, 4.4304, 3.1524, 2.3037, 1.9031, 1.6168, 1.3596, 1.1573}; /* - */
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
