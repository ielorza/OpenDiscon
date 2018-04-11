# OpenDiscon
Open source implementation of popular wind turbine controller interfaces.

Currently available distributions are for:
- The legacy GHBladed external controller interface (typically called DISCON, hence the name OpenDiscon).
- Level-2 MATLAB S-Function for Simulink.

Currently available configurations are for:
- CL-Windcon (DTU 10MW model).

This uses [IK4-IKERLAN](http://www.ikerlan.es/en/)'s library OpenWitcon, included as a submodule.
Documentation is provided in Doxygen format. To generate it, run Doxygen on ./doc/Doxyfile.

For compilation, run cmake here.
This will generate the VS solution, makefiles or MATLAB function for straightforward compilation, depending on your toolchain.
