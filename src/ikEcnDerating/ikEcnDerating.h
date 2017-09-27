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

#ifndef IKECNDERATING_H
#define IKECNDERATING_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include "ikLutbl.h"
    
    double ikEcnDerateTorque(double deratingRatio, double generatorSpeed, double nominalPower, double generatorEfficiency, double nominalGeneratorSpeed);

	double ikEcnDeratePitch(double deratingRatio);

#ifdef __cplusplus
}
#endif

#endif /* IKECNDERATING_H */

