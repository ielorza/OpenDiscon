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
This is a partial C implementation of FunctionToolbox.f90 as published in commit 19b3d1f146a03e22399cced946bd485d1ac7f960 of https://github.com/TUDelft-DataDrivenControl/DISCON_Fortran.git.
*/

double ikTudSaturate(double inputValue, double minValue, double maxValue) {
	double output;
	
	output = inputValue > minValue ? inputValue : minValue;
	output = output < maxValue ? output : maxValue;
	
	return output;
}

double ikTudRatelimit(double refSignal, double measSignal,  double minRate, double maxRate, double DT) {
	double rate;

	rate = (refSignal - measSignal)/DT;
	rate = ikTudSaturate(rate, minRate, maxRate);
	
	return measSignal + rate*DT;
}

double ikTudPIController(double error, double kp, double ki, double minValue, double maxValue, double DT, double I0, int inst) {
	int i;
	double PTerm;
	static double ITerm[99];
	static double ITermLast[99];
	static int init[99];
	double output;
	
	if (!init[inst]) {
		for (i = 0; i < 99; i++) ITerm[i] = 9999.9;
		for (i = 0; i < 99; i++) ITermLast[i] = 9999.9;
			
		ITerm[inst] = I0;
		ITermLast[inst] = I0;
			
		init[inst] = 1;
	}
		
	PTerm = kp*error;
	ITerm[inst] = ITerm[inst] + DT*ki*error;
	ITerm[inst] = ikTudSaturate(ITerm[inst], minValue, maxValue);
	output = PTerm + ITerm[inst];
	output = ikTudSaturate(output, minValue, maxValue);

	ITermLast[inst] = ITerm[inst];

	return output;
}
