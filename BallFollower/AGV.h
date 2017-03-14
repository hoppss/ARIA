/*
 * Joost van Stuijvenberg
 * Avans Hogeschool Breda
 * March 2017
 *
 * CC BY-SA 4.0, see: https://creativecommons.org/licenses/by-sa/4.0/
 * sources & updates: https://github.com/joostvanstuijvenberg/ARIA
 */

#pragma once

#include "Aria.h"

 /*
  * ---------------------------------------------------------------------------------------------- *
  * Class AGV                                                                                      *
  * ---------------------------------------------------------------------------------------------- *
  */
class AGV
{
public:
	AGV(int argc, char** argv, ArRobot* rob);
	~AGV();
	void rijden(double snelheid);
	void draaien(double snelheid);
private:
	ArRobot* robot;
};