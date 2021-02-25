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

#define NINT(a) ((a) >= 0.0 ? (int) ((a)+0.5) : ((a)-0.5))

#include "ikSimpleInputMod.h"
#include "ikSimpleWTConfig.h"
#include "OpenDiscon_EXPORT.h"
#include <stdio.h>

void OpenDiscon_EXPORT DISCON(float *DATA, int FLAG, const char *INFILE, const char *OUTNAME, char *MESSAGE) {
    int err;
    static ikSimpleWTCon con;
                
    if (NINT(DATA[0]) == 0) {
        ikSimpleWTConParams param;
        ikSimpleWTCon_initParams(&param);
        setParams(&param);
        ikSimpleWTCon_init(&con, &param);
    }

    con.in.externalMaximumTorque = 230.0; /* kNm */
    con.in.externalMinimumTorque = 0.0; /* kNm */
    con.in.externalMaximumPitch = 90.0; /* deg */
    con.in.externalMinimumPitch = 0.0; /* deg */
    con.in.generatorSpeed = (double) DATA[19]; /* rad/s */
    con.in.maximumSpeed = 480.0/30*3.1416; /* rpm to rad/s */
        
    ikSimpleWTCon_step(&con);
        
    DATA[46] = (float) (con.out.torqueDemand*1.0e3); /* kNm to Nm */
    DATA[41] = (float) (con.out.pitchDemand/180.0*3.1416); /* deg to rad */
    DATA[42] = (float) (con.out.pitchDemand/180.0*3.1416); /* deg to rad */
    DATA[43] = (float) (con.out.pitchDemand/180.0*3.1416); /* deg to rad */
    DATA[44] = (float) (con.out.pitchDemand/180.0*3.1416); /* deg to rad (collective pitch angle) */

}       
