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
This is an original implementation of derating strategy 3a as described by ECN in deliverable D2.1 of H2020 project CL-Windcon.
The values for the literals have been kindly provided by ECN, who have calculated them to suit the DTU 10MW reference wind turbine from FP7 project INNWIND.
*/

#include "ikEcnDerating.h"

double ikEcnDerateTorque(double deratingRatio, double generatorSpeed, double nominalPower, double generatorEfficiency, double nominalGeneratorSpeed) {
	static ikLutbl lutblKopt;
	static int init = 0;
	const double x[] = {0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50};
	const double y[] = {90.607511506848581, 86.115902720799966, 81.575353112422349, 77.050958297021111, 72.492888078483688, 68.064126426095299, 63.512773230238686, 58.970705560510474, 54.464434076487962, 49.891764181889293, 45.401884663773203};
	double Kopt;
	double Qopt;
	double Qpow;
	double Qmax;
	double Q;
	
	if (!init) {
		init = 1;
		ikLutbl_init(&lutblKopt);
		ikLutbl_setPoints(&lutblKopt, 11, x, y);
	}
	
	Kopt = ikLutbl_eval(&lutblKopt, deratingRatio);
	Qopt = Kopt*generatorSpeed*generatorSpeed;
	Qpow = nominalPower*(1-deratingRatio)/generatorSpeed/generatorEfficiency;
	Qmax = nominalPower/nominalGeneratorSpeed/generatorEfficiency;
	Q = Qopt < Qpow ? Qopt : Qpow;
	Q = Q < Qmax ? Q : Qmax;
	return Q;
}

double ikEcnDeratePitch(double deratingRatio) {
	static ikLutbl lutblPitch;
	static int init = 0;
	const double x[] = {0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50};
	const double y[] = {0.00, 0.039449747839419, 0.058560350086376, 0.073725555631053, 0.086762305188347, 0.098108135965117, 0.108839079483571, 0.118773997213269, 0.128018250433713, 0.136903315900539, 0.145235569651071};
	
	if (!init) {
		init = 1;
		ikLutbl_init(&lutblPitch);
		ikLutbl_setPoints(&lutblPitch, 11, x, y);
	}
	
	return ikLutbl_eval(&lutblPitch, deratingRatio);
}	
