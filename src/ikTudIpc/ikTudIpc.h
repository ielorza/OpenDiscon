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

#ifndef IKTUDIPC_H
#define IKTUDIPC_H

#ifdef __cplusplus
extern "C" {
#endif

	void ikTudColemanTransform(const double rootMOOP[3], double aziAngle, double *axisTilt, double *axisYaw);

	void ikTudColemanTransformInverse(double axisTilt, double axisYaw, double aziAngle, double phi, double PitComIPC[3]);

	void ikTudCalculatePitCom(const double rootMOOP[3], double aziAngle, double Y_MErr, double DT, double KInter, double omegaHP, double omegaLP, double zetaHP, double zetaLP, double phi, int iStatus, int Y_ControlMode, double PitComIPC[3]);

	void ikTudIpc(const double rootMOOP[3], double aziAngle, double phi, double Y_MErr, double DT, double KInter, double omegaHP, double omegaLP, double omegaNotch, double zetaHP, double zetaLP, double zetaNotch, int iStatus, int Y_ControlMode, int NumBl, double PitComIPCF[3]);	

#ifdef __cplusplus
}
#endif

#endif /* IKTUDIPC_H */
	