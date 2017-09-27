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
This is a partial C implementation of Filters.f90 as published in commit 19b3d1f146a03e22399cced946bd485d1ac7f960 of https://github.com/TUDelft-DataDrivenControl/DISCON_Fortran.git.
*/

double ikTudSecLPFilter(double InputSignal, double DT, double CornerFreq, double Damp, int iStatus, int inst) {
	static double InputSignalLast1[99];
	static double InputSignalLast2[99];
	static double OutputSignalLast1[99];
	static double OutputSignalLast2[99];
	double output;

	if (!iStatus)  {
		OutputSignalLast1[inst]  = InputSignal;
		OutputSignalLast2[inst]  = InputSignal;
		InputSignalLast1[inst]   = InputSignal;
		InputSignalLast2[inst]   = InputSignal;
	}

	output = 1/(4+4*DT*Damp*CornerFreq+DT*DT*CornerFreq*CornerFreq) * ( (8-2*DT*DT*CornerFreq*CornerFreq)*OutputSignalLast1[inst]
						+ (-4+4*DT*Damp*CornerFreq-DT*DT*CornerFreq*CornerFreq)*OutputSignalLast2[inst] + (DT*DT*CornerFreq*CornerFreq)*InputSignal
							+ (2*DT*DT*CornerFreq*CornerFreq)*InputSignalLast1[inst] + (DT*DT*CornerFreq*CornerFreq)*InputSignalLast2[inst] );

	InputSignalLast2[inst]   = InputSignalLast1 [inst];
	InputSignalLast1[inst]   = InputSignal;
	OutputSignalLast2[inst]  = OutputSignalLast1 [inst];
	OutputSignalLast1[inst]  = output;

	return output;
}

double ikTudHPFilter(double InputSignal, double DT, double CornerFreq, int iStatus, int inst) {
	double K;
	static double InputSignalLast[99];
	static double OutputSignalLast[99];
	double output;

	if (!iStatus) {
		OutputSignalLast[inst] = InputSignal;
		InputSignalLast[inst] = InputSignal;
	}

	K = 2.0 / DT;

	output = K/(CornerFreq + K)*InputSignal - K/(CornerFreq + K)*InputSignalLast[inst] - (CornerFreq - K)/(CornerFreq + K)*OutputSignalLast[inst];
	
	InputSignalLast[inst]   = InputSignal;
	OutputSignalLast[inst]  = output;

	return output;
}

double ikTudLPFilter(double InputSignal, double DT, double CornerFreq, int iStatus, int reset, int inst) {
    static double InputSignalLast[99];
    static double OutputSignalLast[99];
	double output;

	if (!iStatus || reset) {
		OutputSignalLast[inst] = InputSignal;
		InputSignalLast[inst] = InputSignal;
	}

	output = (DT*CornerFreq*InputSignal + DT*CornerFreq*InputSignalLast[inst] - (DT*CornerFreq-2.0)*OutputSignalLast[inst])/(DT*CornerFreq+2.0);

	InputSignalLast[inst]  = InputSignal;
	OutputSignalLast[inst] = output;

}
