/*
 * Joost van Stuijvenberg
 * Avans Hogeschool Breda
 * March 2017
 *
 * CC BY-SA 4.0, see: https://creativecommons.org/licenses/by-sa/4.0/
 * sources & updates: https://github.com/joostvanstuijvenberg/ARIA
 */

#include "AGV.h"

/*
 * ---------------------------------------------------------------------------------------------- *
 * AGV::AGV                                                                                       *
 * ---------------------------------------------------------------------------------------------- *
 */
AGV::AGV(int argc, char** argv, ArRobot* rob) : robot(rob)
{
	robot->lock();
	robot->runAsync(true);
	robot->disableSonar();
	robot->enableMotors();
	robot->unlock();
}

/*
 * ---------------------------------------------------------------------------------------------- *
 * AGV::~AGV                                                                                      *
 * ---------------------------------------------------------------------------------------------- *
 */
AGV::~AGV()
{
	robot->lock();
	robot->disableMotors();
	robot->stopRunning();
	robot->waitForRunExit();
	robot->unlock();
}

/*
 * ---------------------------------------------------------------------------------------------- *
 * AGV::rijden()                                                                                  *
 * ---------------------------------------------------------------------------------------------- *
 */
void AGV::rijden(double snelheid)
{
	robot->lock();
	robot->setVel(snelheid);
	robot->unlock();
}

/*
 * ---------------------------------------------------------------------------------------------- *
 * AGV::draaien()                                                                                 *
 * ---------------------------------------------------------------------------------------------- *
 */
void AGV::draaien(double snelheid)
{
	robot->lock();
	robot->setRotVel(snelheid);
	robot->unlock();
}