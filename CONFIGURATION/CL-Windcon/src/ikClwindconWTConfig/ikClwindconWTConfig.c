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

	These default values for dr and Kopt have been calculated for DTU's 10MW FAST model.
	Set parameters here:
	*/
		const int n = 101; /* number of points in the lookup table */
	const double dr[] = {0.000000, 0.010000, 0.020000, 0.030000, 0.040000, 0.050000, 0.060000, 0.070000, 0.080000, 0.090000, 0.100000, 0.110000, 0.120000, 0.130000, 0.140000, 0.150000, 0.160000, 0.170000, 0.180000, 0.190000, 0.200000, 0.210000, 0.220000, 0.230000, 0.240000, 0.250000, 0.260000, 0.270000, 0.280000, 0.290000, 0.300000, 0.310000, 0.320000, 0.330000, 0.340000, 0.350000, 0.360000, 0.370000, 0.380000, 0.390000, 0.400000, 0.410000, 0.420000, 0.430000, 0.440000, 0.450000, 0.460000, 0.470000, 0.480000, 0.490000, 0.500000, 0.510000, 0.520000, 0.530000, 0.540000, 0.550000, 0.560000, 0.570000, 0.580000, 0.590000, 0.600000, 0.610000, 0.620000, 0.630000, 0.640000, 0.650000, 0.660000, 0.670000, 0.680000, 0.690000, 0.700000, 0.710000, 0.720000, 0.730000, 0.740000, 0.750000, 0.760000, 0.770000, 0.780000, 0.790000, 0.800000, 0.810000, 0.820000, 0.830000, 0.840000, 0.850000, 0.860000, 0.870000, 0.880000, 0.890000, 0.900000, 0.910000, 0.920000, 0.930000, 0.940000, 0.950000, 0.960000, 0.970000, 0.980000, 0.990000, 1.000000, }; /* - */
	const double Kopt[] = {103.278666, 102.245879, 101.213093, 100.180306, 99.147519, 98.114733, 97.081946, 96.049159, 95.016373, 93.983586, 92.950799, 91.918013, 90.885226, 89.852440, 88.819653, 87.786866, 86.754080, 85.721293, 84.688506, 83.655720, 82.622933, 81.590146, 80.557360, 79.524573, 78.491786, 77.459000, 76.426213, 75.393426, 74.360640, 73.327853, 72.295066, 71.262280, 70.229493, 69.196706, 68.163920, 67.131133, 66.098346, 65.065560, 64.032773, 62.999986, 61.967200, 60.934413, 59.901626, 58.868840, 57.836053, 56.803266, 55.770480, 54.737693, 53.704906, 52.672120, 51.639333, 50.606546, 49.573760, 48.540973, 47.508186, 46.475400, 45.442613, 44.409826, 43.377040, 42.344253, 41.311466, 40.278680, 39.245893, 38.213106, 37.180320, 36.147533, 35.114746, 34.081960, 33.049173, 32.016386, 30.983600, 29.950813, 28.918027, 27.885240, 26.852453, 25.819667, 24.786880, 23.754093, 22.721307, 21.688520, 20.655733, 19.622947, 18.590160, 17.557373, 16.524587, 15.491800, 14.459013, 13.426227, 12.393440, 11.360653, 10.327867, 9.295080, 8.262293, 7.229507, 6.196720, 5.163933, 4.131147, 3.098360, 2.065573, 1.032787, -0.000000, }; /* Nm*s^2/rad^2 */

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
	const int n = 101; /* number of points in the lookup table */
	const double dr[] = {0.000000, 0.010000, 0.020000, 0.030000, 0.040000, 0.050000, 0.060000, 0.070000, 0.080000, 0.090000, 0.100000, 0.110000, 0.120000, 0.130000, 0.140000, 0.150000, 0.160000, 0.170000, 0.180000, 0.190000, 0.200000, 0.210000, 0.220000, 0.230000, 0.240000, 0.250000, 0.260000, 0.270000, 0.280000, 0.290000, 0.300000, 0.310000, 0.320000, 0.330000, 0.340000, 0.350000, 0.360000, 0.370000, 0.380000, 0.390000, 0.400000, 0.410000, 0.420000, 0.430000, 0.440000, 0.450000, 0.460000, 0.470000, 0.480000, 0.490000, 0.500000, 0.510000, 0.520000, 0.530000, 0.540000, 0.550000, 0.560000, 0.570000, 0.580000, 0.590000, 0.600000, 0.610000, 0.620000, 0.630000, 0.640000, 0.650000, 0.660000, 0.670000, 0.680000, 0.690000, 0.700000, 0.710000, 0.720000, 0.730000, 0.740000, 0.750000, 0.760000, 0.770000, 0.780000, 0.790000, 0.800000, 0.810000, 0.820000, 0.830000, 0.840000, 0.850000, 0.860000, 0.870000, 0.880000, 0.890000, 0.900000, 0.910000, 0.920000, 0.930000, 0.940000, 0.950000, 0.960000, 0.970000, 0.980000, 0.990000, 1.000000, }; /* - */
	const double pitch[] = {-0.005934, 0.015899, 0.024958, 0.032066, 0.038137, 0.043571, 0.048515, 0.053082, 0.057376, 0.061420, 0.065266, 0.068945, 0.072488, 0.075905, 0.079212, 0.082410, 0.085516, 0.088492, 0.091394, 0.094295, 0.097110, 0.099803, 0.102496, 0.105159, 0.107684, 0.110208, 0.112733, 0.115166, 0.117562, 0.119959, 0.122347, 0.124626, 0.126905, 0.129183, 0.131437, 0.133614, 0.135791, 0.137968, 0.140120, 0.142195, 0.144270, 0.146344, 0.148416, 0.150402, 0.152387, 0.154373, 0.156359, 0.158300, 0.160216, 0.162132, 0.164048, 0.165960, 0.167816, 0.169673, 0.171529, 0.173386, 0.175216, 0.177002, 0.178788, 0.180574, 0.182360, 0.184102, 0.185797, 0.187492, 0.189187, 0.190883, 0.192558, 0.194197, 0.195835, 0.197474, 0.199112, 0.200749, 0.202343, 0.203936, 0.205530, 0.207124, 0.208717, 0.210286, 0.211834, 0.213383, 0.214931, 0.216479, 0.218028, 0.219540, 0.221049, 0.222558, 0.224066, 0.225575, 0.227078, 0.228542, 0.230006, 0.231470, 0.232934, 0.234397, 0.235855, 0.237283, 0.238710, 0.240138, 0.241565, 0.242993, 0.244419, }; /* rad */
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
	const double gain[] = {2.1000, 2.1000, 2.0727, 1.7182, 1.5182, 1.3545, 1.2636, 1.1909, 1.1182, 1.0545}; /* - */
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
