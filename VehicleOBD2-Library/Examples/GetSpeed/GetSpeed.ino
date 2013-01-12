/*
    Get Speed with the Vehicle OBD2 Shield - SiGAlabs (www.sigalabs.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 /* 
    Description
    -----------------------------------
    This is a simple demo to start working with the Vehicle OBD2 Shield.
    Establish a connection with the vehicle's ECU and then read the current speed.
 */
 
#include "OBD2.h"

OBD2 obd2;

void setup()
{
  //Initialize connection with the vehicle
  obd2.Init(&Serial);  
}

void loop()
{
        //Refresh Vehicle status, should be called in every loop
	obd2.Refresh();

        //The vehicle's ECU sends data only when the ignition is ON
	if(obd2.isIgnitionOn){
	
            int myspeed = obd2.Speed();  //Read current vehicle's speed
 
	}

	delay(1000);
}