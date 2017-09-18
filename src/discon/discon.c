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

#define NINT(a) ((a) >= 0.0 ? (int) ((a)+0.5) : ((a)-0.5))

#include "ikClwindconWTConfig.h"
#include "OpenDiscon_EXPORT.h"
#include <stdio.h>

void OpenDiscon_EXPORT DISCON(float *DATA, int FLAG, const char *INFILE, const char *OUTNAME, char *MESSAGE) {
	int err;
	static ikClwindconWTCon con;
	double output = -12.0;
	static FILE *f = NULL;
		
	if (NINT(DATA[0]) == 0) {
		ikClwindconWTConParams param;
		ikClwindconWTCon_initParams(&param);
		setParams(&param);
		ikClwindconWTCon_init(&con, &param);
		f = fopen("log.bin", "wb");
	}
	
	con.in.externalMaximumTorque = 198.0; /* kNm */
	con.in.externalMinimumTorque = 0.0; /* 0.0 */
	con.in.externalMaximumPitch = 90.0; /* deg */
	con.in.externalMinimumPitch = 0.0; /* deg */
	con.in.generatorSpeed = (double) DATA[19]; /* rad/s */
	con.in.maximumSpeed = 480.0/30*3.1416; /* rpm to rad/s */
	
	ikClwindconWTCon_step(&con);
	
	DATA[46] = (float) (con.out.torqueDemand*1.0e3); /* kNm to Nm */
	DATA[41] = (float) (con.out.pitchDemandBlade1/180.0*3.1416); /* deg to rad */
	DATA[42] = (float) (con.out.pitchDemandBlade2/180.0*3.1416); /* deg to rad */
	DATA[43] = (float) (con.out.pitchDemandBlade3/180.0*3.1416); /* deg to rad */
	DATA[44] = (float) (con.out.pitchDemandBlade1/180.0*3.1416); /* deg to rad (collective pitch angle) */

	err = ikClwindconWTCon_getOutput(&con, &output, "collective pitch control>linear controller>error transfer functions>0");
	fwrite(&(output), 1, sizeof(output), f);
}	
