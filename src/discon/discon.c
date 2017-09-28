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

/*
This includes and implementation of a derating strategy described by ECN in deliverable D2.1 of H2020 project CL-Windcon.
This includes an IPC and yaw strategy based on commit 19b3d1f146a03e22399cced946bd485d1ac7f960 of https://github.com/TUDelft-DataDrivenControl/DISCON_Fortran.git.
*/

#define NINT(a) ((a) >= 0.0 ? (int) ((a)+0.5) : (int) ((a)-0.5))

#include "ikClwindconWTConfig.h"
#include "OpenDiscon_EXPORT.h"
#include "ikEcnDerating.h"
#include "ikTudFilters.h"
#include "ikTudFunctionToolbox.h"
#include "ikTudIpc.h"
#include "math.h"
#include <stdio.h>

void OpenDiscon_EXPORT DISCON(float *DATA, int FLAG, const char *INFILE, const char *OUTNAME, char *MESSAGE) {
	/* Torque and collective pitch */
	static ikClwindconWTCon con;
	/* Derating */
	const double deratingRatio = 0.0; /* later to be got via the supercontroller interface */
	/* IPC */
	int K;
	double rootMOOP[3];
	double IPC_PitComF[3];
	double PitComT_IPC[3];
	double BlPitch[3];
	double PitCom[3];
	/* Yaw */
	static double Y_YawEndT = -1.0;
	double Y_ErrLPFFast;
	double Y_ErrLPFSlow;
	static double Y_AccErr = 0.0;
	
	/* Init */
	if (NINT(DATA[0]) == 0) {
		ikClwindconWTConParams param;
		ikClwindconWTCon_initParams(&param);
		setParams(&param);
		ikClwindconWTCon_init(&con, &param);
	}

	/* Derating */
	con.in.externalMaximumTorque = ikEcnDerateTorque(deratingRatio, (double) DATA[19], 10e6, 0.94, 480.0/30*3.1416)/1e3; /* kNm */
	con.in.externalMinimumTorque = 0.0; /* 0.0 */
	con.in.externalMaximumPitch = (double) DATA[6]; /* deg */
	con.in.externalMinimumPitch = ikEcnDeratePitch(deratingRatio)/3.1416*180.0; /* deg */
	con.in.generatorSpeed = (double) DATA[19]; /* rad/s */
	con.in.maximumSpeed = 480.0/30*3.1416; /* rpm to rad/s */
	
	/* Torque and collective pitch */
	ikClwindconWTCon_step(&con);
	
	/* IPC */
	rootMOOP[0] = (double) DATA[29];
	rootMOOP[1] = (double) DATA[30];
	rootMOOP[2] = (double) DATA[31];
	
	ikTudIpc(rootMOOP, (double) DATA[59], 0.436332313, (double) DATA[23], 0.01, 8e-10, 0.3141592, 0.6283185, 1.269330365, 0.70, 1.0, 0.5, NINT(DATA[0]), NINT(DATA[119]), 3, IPC_PitComF);

	BlPitch[0] = (double) DATA[3];
	BlPitch[1] = (double) DATA[32];
	BlPitch[2] = (double) DATA[33];

	for (K = 0; K < 3; K++) {
		PitComT_IPC[K] = con.out.pitchDemandBlade1/180.0*3.1416 + IPC_PitComF[K];
		PitComT_IPC[K] = ikTudSaturate(PitComT_IPC[K], (double) DATA[5], (double) DATA[6]);
		PitCom[K] = ikTudRatelimit(PitComT_IPC[K], BlPitch[K], (double) DATA[7], (double) DATA[8], 0.01);
		PitCom[K] = ikTudSaturate(PitComT_IPC[K], (double) DATA[5], (double) DATA[6]);
	}
	
	DATA[46] = (float) (con.out.torqueDemand*1.0e3); /* kNm to Nm */
	DATA[41] = (float) PitCom[0];
	DATA[42] = (float) PitCom[1];
	DATA[43] = (float) PitCom[2];
	DATA[44] = (float) (con.out.pitchDemandBlade1/180.0*3.1416); /* deg to rad (collective pitch angle) */
	
	/* Yaw */
	if (NINT(DATA[119]) == 1) {
		DATA[28] = (float) 0.0;
		if ((double) DATA[1] >= Y_YawEndT) {
			DATA[47] = (float) 0.0;

			Y_ErrLPFFast = ikTudLPFilter((double) DATA[23], 0.01, 1.0, NINT(DATA[0]), 0, 2);
			Y_ErrLPFSlow = ikTudLPFilter((double) DATA[23], 0.01, 0.016666667, NINT(DATA[0]), 0, 3);

			Y_AccErr = Y_AccErr + 0.01*Y_ErrLPFFast*fabs(Y_ErrLPFFast);

			if (fabs(Y_AccErr) >= 1.745329252) {
				Y_YawEndT = fabs(Y_ErrLPFSlow/(double) DATA[120]) + (double) DATA[1];
			}
		} else {
			DATA[47] = (float) ( (double) DATA[23] >= 0.0 ? fabs((double) DATA[120]) : -fabs((double) DATA[120]) );
			Y_ErrLPFFast = ikTudLPFilter((double) DATA[23], 0.01, 1.0, NINT(DATA[0]), 1, 2);
			Y_ErrLPFSlow = ikTudLPFilter((double) DATA[23], 0.01, 0.016666667, NINT(DATA[0]), 1, 3);
			Y_AccErr = 0.0;
		}
	}

}	
