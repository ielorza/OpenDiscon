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

#ifndef IKTUDFILTERS_H
#define IKTUDFILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

	double ikTudSecLPFilter(double InputSignal, double DT, double CornerFreq, double Damp, int iStatus, int inst);

	double ikTudHPFilter(double InputSignal, double DT, double CornerFreq, int iStatus, int inst);
	
	double ikTudLPFilter(double InputSignal, double DT, double CornerFreq, int iStatus, int reset, int inst);

#ifdef __cplusplus
}
#endif

#endif /* IKTUDFILTERS_H */
