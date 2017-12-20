# OpenDiscon
Open source implementation of the legacy GHBladed external controller interface.

This code is for the production of a shared library implementing the DISCON interface for the DTU 10MW model.

The source code is at ./src. The main implementation file is discon.c.
It uses [IK4-IKERLAN](http://www.ikerlan.es/en/)'s library OpenWitcon, included as a submodule.
Documentation is provided in Doxygen format. To generate it, run Doxygen on ./doc/Doxyfile.

For compilation, run cmake here.
This will generate the VS solution or makefiles for straightforward compilation, depending on your toolchain.
