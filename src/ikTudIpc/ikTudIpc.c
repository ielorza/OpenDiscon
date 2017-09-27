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
This is a C implementation of IPC.f90 as published in commit 19b3d1f146a03e22399cced946bd485d1ac7f960 of https://github.com/TUDelft-DataDrivenControl/DISCON_Fortran.git.
*/

#include "ikTudIpc.h"
#include "ikTudFunctionToolbox.h"
#include "ikTudFilters.h"
#include <math.h>

static const double PI = 3.14159265359;

void ikTudColemanTransform(const double rootMOOP[3], double aziAngle, double *axisTilt, double *axisYaw) {
	const double phi2 = 2.0/3.0*PI;
	const double phi3 = 4.0/3.0*PI;
	
	*axisTilt = 2.0/3.0 * (cos(aziAngle)*rootMOOP[1] + cos(aziAngle+phi2)*rootMOOP[2] + cos(aziAngle+phi3)*rootMOOP[3]);
	*axisYaw	= 2.0/3.0 * (sin(aziAngle)*rootMOOP[1] + sin(aziAngle+phi2)*rootMOOP[2] + sin(aziAngle+phi3)*rootMOOP[3]);
	
}

void ikTudColemanTransformInverse(double axisTilt, double axisYaw, double aziAngle, double phi, double PitComIPC[3]) {
	const double phi2 = 2.0/3.0*PI;
	const double phi3 = 4.0/3.0*PI;
	
	PitComIPC[1] = cos(aziAngle+phi)*axisTilt + sin(aziAngle+phi)*axisYaw;
	PitComIPC[2] = cos(aziAngle+phi+phi2)*axisTilt + sin(aziAngle+phi+phi2)*axisYaw;
	PitComIPC[3] = cos(aziAngle+phi+phi3)*axisTilt + sin(aziAngle+phi+phi3)*axisYaw;
	
}

void ikTudCalculatePitCom(const double rootMOOP[3], double aziAngle, double Y_MErr, double DT, double KInter, double omegaHP, double omegaLP, double zetaHP, double zetaLP, double phi, int iStatus, int Y_ControlMode, double PitComIPC[3]) {
	double axisTilt, axisYaw, axisYawF;
	double IntAxisTilt, IntAxisYaw;
	double IntAxisYawIPC;
	double Y_MErrF, Y_MErrF_IPC;
	
	if (iStatus == 0) {
		IntAxisTilt = 0.0;
		IntAxisYaw = 0.0;
	}
	
	ikTudColemanTransform(rootMOOP, aziAngle, &axisTilt, &axisYaw);
	
	if (Y_ControlMode == 2) {
		axisYawF = ikTudHPFilter(axisYaw, DT, omegaHP, iStatus, 1);
		Y_MErrF = ikTudSecLPFilter(Y_MErr, DT, omegaLP, zetaLP, iStatus, 1);
	} else {
		axisYawF = axisYaw;
		Y_MErrF = 0.0;
	}
	
	IntAxisTilt	= IntAxisTilt + DT * KInter * axisTilt;
	IntAxisYaw	= IntAxisYaw + DT * KInter * axisYawF;
	
	Y_MErrF_IPC = ikTudPIController(Y_MErrF, -0.16, -0.002, -100.0, 100.0, DT, 0.0, 3);
	IntAxisYawIPC = IntAxisYaw + Y_MErrF_IPC;
	
	ikTudColemanTransformInverse(IntAxisTilt, IntAxisYawIPC, aziAngle, phi, PitComIPC);
	
}

void ikTudIpc(const double rootMOOP[3], double aziAngle, double phi, double Y_MErr, double DT, double KInter, double omegaHP, double omegaLP, double omegaNotch, double zetaHP, double zetaLP, double zetaNotch, int iStatus, int Y_ControlMode, int NumBl, double PitComIPCF[3]) {
	double rootMOOPF[3];
	double PitComIPC[3];
	int K;
	
	for (K = 0; K < NumBl; K++) {
		rootMOOPF[K] = rootMOOP[K];
	}
	
	ikTudCalculatePitCom(rootMOOPF, aziAngle, Y_MErr, DT, KInter, omegaHP, omegaLP, zetaHP, zetaLP, phi, iStatus, Y_ControlMode, PitComIPC);
	
	for (K = 0; K < NumBl; K++) {
		PitComIPCF[K] = PitComIPC[K];
	}
	
}	
