/*
    DTC Read with the Vehicle OBD2 Shield - SiGAlabs (www.sigalabs.com)

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
    This is a simple demo to start working with your vehicle's Diagnostic Trouble Codes.
    Establish a connection with the vehicle's ECU and then read the available DTC codes, 
	if there are DTC codes available then search for a specific trouble code. 
	If your DTC is found a digital output will go high to drive a led, an alarm etc.
 */

#include "OBD2.h"

#define PREDEFINED_DTC "B1882"
#define DTC_OUTPUT_PIN 13

OBD2 obd2;

void setup()
{
	//CONFIGURE DTC ALARM PIN AS OUTPUT
	pinMode(DTC_OUTPUT_PIN, OUTPUT);
	//SET NORMAL STATE OF THE ALARM PIN TO LOW
	digitalWrite(DTC_OUTPUT_PIN, LOW);  
	//INIT CONNECTION TO THE VEHICLE USING SIGALABS VEHICLE OBD2 SHIELD.
	obd2.Init(&Serial);  
}

void loop()
{
	//REFRESH OBD2 CONNECTION STATUS ON EACH LOOP
	obd2.Refresh();

	//READ AVAILABLE DTC TROUBLE CODES
	if(obd2.dtc_read())
        {      
		//CHECK IF THERE IS A DTC AVAILABLE
		if (obd2.has_dtc)
		{
			//LOOP ALL AVAILABLE MEMORY POSITIONS FOR DTC CODES (MAX NUMBER IS DEFINED IN THE LIBRARY FILES)
			for (char i=0;i<MAX_DTC_READ;i++)
			{
				//COMPARE EACH FOUND DTC CODE TO THE PREDEFINED ONE
				if(strcmp(obd2.DTC[i].code,PREDEFINED_DTC)==0)
				{
					//DTC FOUND
					digitalWrite(DTC_OUTPUT_PIN, HIGH); 
				}
			}
		}
		else
		{
			//NO DTC FOUND
			digitalWrite(DTC_OUTPUT_PIN, LOW); 
		}
	}
	delay(1000);
}