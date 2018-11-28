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

#include "ikClwindconInputMod.h"
#include "ikClwindconWTConfig.h"
#include "OpenDiscon_EXPORT.h"
#include <stdio.h>

void OpenDiscon_EXPORT DISCON(float *DATA, int FLAG, const char *INFILE, const char *OUTNAME, char *MESSAGE) {
	int err;
	static ikClwindconWTCon con;
	double output = -12.0;
	static FILE *f = NULL;
	const double deratingRatio = 0.2; /* later to be got via the supercontroller interface */
		
	if (NINT(DATA[0]) == 0) {
    		/*
		####################################################################
                    	 	Sampling interval
				
   		 Set sampling interval here:
		*/
		const double T = DATA[2];/* [s] */
   		 /*
   		 ####################################################################
		*/
		ikClwindconWTConParams param;
		ikClwindconWTCon_initParams(&param);
		setParams(&param,T);
		ikClwindconWTCon_init(&con, &param);
		f = fopen("log.bin", "wb");
	}

	con.in.deratingRatio = deratingRatio;
	con.in.externalMaximumTorque = 230.0; /* kNm */
	con.in.externalMinimumTorque = 0.0; /* kNm */
	con.in.externalMaximumPitch = 90.0; /* deg */
	con.in.externalMinimumPitch = 0.0; /* deg */
	con.in.generatorSpeed = (double) DATA[19]; /* rad/s */
	con.in.rotorSpeed = (double) DATA[20]; /* rad/s */
	con.in.maximumSpeed = 480.0/30*3.1416; /* rpm to rad/s */
	con.in.azimuth = 180.0/3.1416 * (double) DATA[59]; /* rad to deg */
	con.in.maximumIndividualPitch = 10.0; /* deg */
	con.in.yawErrorReference = 0.0; /* deg */
	con.in.yawError = 180.0/3.1416 * (double) DATA[23]; /* rad to deg */
	con.in.bladeRootMoments[0].c[0] = 1.0e-3 * (double) DATA[68]; /* Nm to kNm */
	con.in.bladeRootMoments[0].c[1] = 1.0e-3 * (double) DATA[29]; /* Nm to kNm */
	con.in.bladeRootMoments[0].c[2] = 0.0; /* kNm */
	con.in.bladeRootMoments[1].c[0] = 1.0e-3 * (double) DATA[69]; /* Nm to kNm */
	con.in.bladeRootMoments[1].c[1] = 1.0e-3 * (double) DATA[30]; /* Nm to kNm */
	con.in.bladeRootMoments[1].c[2] = 0.0; /* kNm */
	con.in.bladeRootMoments[2].c[0] = 1.0e-3 * (double) DATA[70]; /* Nm to kNm */
	con.in.bladeRootMoments[2].c[1] = 1.0e-3 * (double) DATA[31]; /* Nm to kNm */
	con.in.bladeRootMoments[2].c[2] = 0.0; /* kNm */
	
	ikClwindconInputMod(&(con.in));
	ikClwindconWTCon_step(&con);
	
	DATA[46] = (float) (con.out.torqueDemand*1.0e3); /* kNm to Nm */
	DATA[41] = (float) (con.out.pitchDemandBlade1/180.0*3.1416); /* deg to rad */
	DATA[42] = (float) (con.out.pitchDemandBlade2/180.0*3.1416); /* deg to rad */
	DATA[43] = (float) (con.out.pitchDemandBlade3/180.0*3.1416); /* deg to rad */
	err = ikClwindconWTCon_getOutput(&con, &output, "collective pitch demand");
	DATA[44] = (float) (output/180.0*3.1416); /* deg to rad (collective pitch angle) */

	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>pitch y from control");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>pitch z from control");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>My");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>Mz");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>pitch increment 1");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>pitch increment 2");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "individual pitch control>pitch increment 3");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "generator speed equivalent");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "speed sensor manager>signal 1");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "speed sensor manager>signal 2");
	fwrite(&(output), 1, sizeof(output), f);
	err = ikClwindconWTCon_getOutput(&con, &output, "speed sensor manager>signal 3");
	fwrite(&(output), 1, sizeof(output), f);
}	
