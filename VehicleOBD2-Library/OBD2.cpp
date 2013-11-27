/*
    stn1110.cpp - library for the Vehicle OBD2 Shield - SiGAlabs
	www.sigalabs.com

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
    History
    ---------------------------------
	This library is created to help people start using the Vehicle OBD2 Shield 
	and the Scantool's STN1110 IC.
	
    Library Version
    ------------------------------------
	BETA    (010) 0.10   First public beta
	BETA	(011) 0.11	 Added DTC handle, Speed PID result fixed.						 
		 
	
	
	
	TO-DO
    ---------------------------------
    - A lot of things!

*/

 #if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
#endif

#include "OBD2.h"



  prog_uchar pid_reslen[] PROGMEM=
{
  // pid 0x00 to 0x1F
  4,4,2,2,1,1,1,1,1,1,1,1,2,1,1,1,
  2,1,1,1,2,2,2,2,2,2,2,2,1,1,1,4,

  // pid 0x20 to 0x3F
  4,2,2,2,4,4,4,4,4,4,4,4,1,1,1,1,
  1,2,2,1,4,4,4,4,4,4,4,4,2,2,2,2,

  // pid 0x40 to 0x4E
  4,8,2,2,2,1,1,1,1,1,1,1,1,2,2
};


/**********************************************************
Method returns library version

return val: 100 means library version 1.00
            101 means library version 1.01
**********************************************************/
int OBD2::LibVer(void)
{
  return (OBD2_LIB_VERSION);
}

/**********************************************************
  Constructor definition
***********************************************************/

OBD2::OBD2(void)
{
 _Serial = NULL;
  
}

/**********************************************************
Method initialize serial object for communication and other parameters

return: 
		ERROR ret. val:
        ---------------
		-1 - Initialize failed
		
        OK ret val:
        -----------
		 0 - Init OK
		 
**********************************************************/
char OBD2::Init(HardwareSerial *serIn)
{
  _Serial = serIn;
  _Serial->begin(9600);
   char str[STRLEN];
  // reset
  stn1110_command(str, PSTR("ATWS\r"));
  // turn echo off
  stn1110_command(str, PSTR("ATE0\r"));
  
  // send 01 00 until we are connected
  do
  {
    stn1110_command(str, PSTR("0100\r"));
    delay(1000);
  }
  while(stn1110_check_response("0100", str)!=0);
  
  // ask protocol
  stn1110_command(str, PSTR("ATDPN\r"));

  check_supported_pids();
  
  return 0;
}

/**********************************************************
Method read response from STN1110

return: 
        ERROR ret. val:
        ---------------

        OK ret val:
        -----------
        PROMPT - We got only a prompt
		
		DATA - Data received
		
**********************************************************/
char OBD2::stn1110_read(char *str, byte size)
{
  int b;
  byte i=0;
  
  // wait for something on com port
  while((b=_Serial->read())!=PROMPT && i<size)
  {
    if(b>=' ')
      str[i++]=b;
  }

  if(i!=size)  // we got a prompt
  {
    str[i]=NUL;  // replace CR by NUL
    return PROMPT;
  }
  else
    return DATA;
}

/**********************************************************
Method write data to STN1110
		
**********************************************************/
void OBD2::stn1110_write(char *str)
{
  while(*str!=NUL)
    _Serial->write(*str++);
}

/**********************************************************
Method check response from the STN1110 compared to the given response

return: 
        ERROR ret. val:
        ---------------
        -1 - Wrong response

        OK ret val:
        -----------
        0 - OK response
**********************************************************/
char OBD2::stn1110_check_response(const char *cmd, char *str)
{
  // cmd is something like "010D"
  // str should be "41 0D blabla"
  if(cmd[0]+4 != str[0]
    || cmd[1]!=str[1]
    || cmd[2]!=str[3]
    || cmd[3]!=str[4])
    return -1;

  return 0;  // no error
}

/**********************************************************
Method check response from the STN1110 compared to the given response

return: 
        size of the response array
**********************************************************/
char OBD2::stn1110_compact_response(byte *buf, char *str)
{
  byte i=0;

  str+=6;
  while(*str!=NUL)
    buf[i++]=strtoul(str, &str, 16);  // 16 = hex

  return i;
}

/**********************************************************
Method send command to the STN1110 and reply back the responce

return: 
        response
**********************************************************/
char OBD2::stn1110_command(char *str, char *cmd)
{
  strcpy_P(str, cmd);
  stn1110_write(str);
  return stn1110_read(str, STRLEN);
}

/**********************************************************
Method get vehicle's supported PIDs

return: 
       void
**********************************************************/
void OBD2::check_supported_pids(void)
{
  char str[STRLEN];

  // on some ECU first PID read attemts some time fails, changed to 3 attempts
  for (byte i=0; i<3; i++)
  {
    pid01to20_support = (get_pid(PID_SUPPORT00, &tempLong)) ? tempLong : 0;
    if (pid01to20_support) 
      break; 
  }

  if(is_pid_supported(PID_SUPPORT20))
    if (get_pid(PID_SUPPORT20, &tempLong))
      pid21to40_support = tempLong; 

  if(is_pid_supported(PID_SUPPORT40))
    if (get_pid(PID_SUPPORT40, &tempLong))
      pid41to60_support = tempLong;
}

/**********************************************************
Method check response from the STN1110 compared to the given response

return: 

        OK ret val:
        -----------
        0 - Not Supported
		1 - Supported
**********************************************************/
char OBD2::is_pid_supported(byte pid)
{
  if(pid==0)
    return 1;
  else
  if(pid<=0x20)
  {
    if(1L<<(byte)(0x20-pid) & pid01to20_support)
      return 1;
  }
  else
  if(pid<=0x40)
  {
    if(1L<<(byte)(0x40-pid) & pid21to40_support)
      return 1;
  }
  else
  if(pid<=0x60)
  {
    if(1L<<(byte)(0x60-pid) & pid41to60_support)
      return 1;
  }

  return 0;
}


/**********************************************************
Method request value for a PID

return: 
		ERROR ret val:
		--------------
		0 - Value not received

        OK ret val:
        -----------
		1 - Value is received
**********************************************************/
char OBD2::get_pid(byte pid, long *ret)
{
  char cmd_str[6];   // to send to STN1110
  char str[STRLEN];   // to receive from STN1110
  byte i;
  byte buf[10];   // to receive the result
  byte reslen;
  char decs[16];
 
  // check if PID is supported
  if(!is_pid_supported(pid))
  {
    // Not Supported
    return 0;
  }
  
  // receive length depends on pid
  reslen=pgm_read_byte_near(pid_reslen+pid);

  sprintf_P(cmd_str, PSTR("01%02X\r"), pid);
  stn1110_write(cmd_str);
  stn1110_read(str, STRLEN);
 
  if(stn1110_check_response(cmd_str, str)!=0)
  {
    return 0;
  }
   stn1110_compact_response(buf, str);

   *ret=buf[0]*256U+buf[1];

  // Calculate different for each PID
  switch(pid)
  {
  case ENGINE_RPM:
    *ret=*ret/4U;
  break;
  case MAF_AIR_FLOW:
  break;
  case VEHICLE_SPEED:
    *ret=buf[0] / 100U;
  break;
  case FUEL_STATUS:
  case LOAD_VALUE:
  case THROTTLE_POS:
  case REL_THR_POS:
  case EGR:
  case EGR_ERROR:
  case FUEL_LEVEL:
  case ABS_THR_POS_B:
  case ABS_THR_POS_C:
  case ACCEL_PEDAL_D:
  case ACCEL_PEDAL_E:
  case ACCEL_PEDAL_F:
  case CMD_THR_ACTU:
    *ret=(buf[0]*100U)/255U;
  break;
  case ABS_LOAD_VAL:
    *ret=(*ret*100)/255;
  break;
  case B1S1_O2_V:
  case B1S2_O2_V:
  case B1S3_O2_V:
  case B1S4_O2_V:
  case B2S1_O2_V:
  case B2S2_O2_V:
  case B2S3_O2_V:
  case B2S4_O2_V:
  case O2S1_WR_V:
  case O2S2_WR_V:
  case O2S3_WR_V:
  case O2S4_WR_V:
  case O2S5_WR_V:
  case O2S6_WR_V:
  case O2S7_WR_V:
  case O2S8_WR_V:
  case O2S1_WR_C:
  case O2S2_WR_C:
  case O2S3_WR_C:
  case O2S4_WR_C:
  case O2S5_WR_C:
  case O2S6_WR_C:
  case O2S7_WR_C:
  case O2S8_WR_C:
  case CMD_EQUIV_R:
  case DIST_MIL_ON:
  case DIST_MIL_CLR:
  case TIME_MIL_ON:
  case TIME_MIL_CLR:
  case COOLANT_TEMP:
  case INT_AIR_TEMP:
  case AMBIENT_TEMP:
  case CAT_TEMP_B1S1:
  case CAT_TEMP_B2S1:
  case CAT_TEMP_B1S2:
  case CAT_TEMP_B2S2:
  case STFT_BANK1:
  case LTFT_BANK1:
  case STFT_BANK2:
  case LTFT_BANK2:
  case FUEL_PRESSURE:
  case MAN_PRESSURE:
  case BARO_PRESSURE:
    *ret=buf[0];
    if(pid==FUEL_PRESSURE)
      *ret*=3U;
  break;
  case EVAP_PRESSURE:
    *ret=((int)buf[0]*256+buf[1])/4;
  break;
  case TIMING_ADV:
    *ret=(buf[0]/2)-64;
  break;
  case CTRL_MOD_V:
  break;
  case RUNTIME_START:
  break;
  case OBD_STD:
    *ret=buf[0];
  break;
 
  default:
    *ret=0;
    for(i=0; i<reslen; i++)
    {
      *ret*=256L;
      *ret+=buf[i];
    }
  break;
  }

  return 1;
}

/**********************************************************
Method check if vehicle's ECU is connected. 

return: 
		ERROR ret val:
		--------------
		0 - ECU is not connected

        OK ret val:
        -----------
		1 - ECU is connected
**********************************************************/
char OBD2::verifyECUAlive()
{
  char cmd_str[6];   
  char str[STRLEN];   
  sprintf_P(cmd_str, PSTR("01%02X\r"), ENGINE_RPM);
  stn1110_write(cmd_str);
  stn1110_read(str, STRLEN);
  
  if(stn1110_check_response(cmd_str, str) == 0)
  {  
	return 1;
  }
  else
  {
    return 0;
  }
}

/**********************************************************
Method refresh values. Run in each loop

return: 
		ERROR ret val:
		--------------
		0 - Error update

        OK ret val:
        -----------
		1 - Update done
**********************************************************/
char OBD2::Refresh()
{
	isIgnitionOn = verifyECUAlive();
	
	//If ignition is on, check for engine
	if (isIgnitionOn)
		isEngineOn = (get_pid(ENGINE_RPM, &engineRPM) && engineRPM > 0) ? 1 : 0;
	else // else engine must be off
		isEngineOn = 0;
}


/**********************************************************
Method read vehicle's speed

return: 
		Vehicle current's Speed
**********************************************************/
int OBD2::Speed()
{

  if (get_pid(VEHICLE_SPEED,&tempLong)==0)
  {
    return -1; // not valid, exit
  }
  
  return (int)tempLong;
}

/**********************************************************
Method read vehicle's RPM

return: 
		Vehicle's current RPM
**********************************************************/
int OBD2::RPM()
{

  if (get_pid(ENGINE_RPM,&tempLong)==0)
  {
    return -1; // not valid, exit
  }
  
  return (int)tempLong;
}


bool OBD2::dtc_clear(void)
{
	char cmd_answer[DTC_BUFFER]="";
	
	stn1110_write("04\r");
	stn1110_read(cmd_answer,DTC_BUFFER);
    strip_answer(cmd_answer);

    if (strcmp(cmd_answer, "44")!=0)
    {
       return false;
    } else
    {
		has_dtc=false;
       return true;
    }
    
    return true;
}





bool OBD2::dtc_read(void)
{
    char cmd_answer[DTC_BUFFER]="";
	has_dtc=false;
     
	stn1110_write("03\r");
       
	stn1110_read(cmd_answer,DTC_BUFFER);
	
	
                   
	for (char i=0;i<MAX_DTC_READ;i++)
	{
		strcpy(DTC[i].code,"");
	}
	
	strip_answer(cmd_answer);

		

	if (strstr(cmd_answer, "NODATA"))
    {
	//No errors
		return true;
    }
	
	
    if (strncmp(cmd_answer, "43", 2)!=0)
    {
	   
	//ERROR: Incorrect answer
	return false;
    }
    
  
						  
    char *ss=cmd_answer+2;
	char dtclen=0;
	
    while (strlen(ss) >= 4)
    {
	const char *prefix[16]=
	    {
		"P0", "P1", "P2", "P3",
		"C0", "C1", "C2", "C3",
		"B0", "B1", "B2", "B3",
		"U0", "U1", "U2", "U3",
	    };
	uint8_t p=0;
	if ( ((*ss)>='0') && ((*ss)<='9') ) p=(*ss)-'0'; else
	if ( ((*ss)>='A') && ((*ss)<='F') ) p=(*ss)-'A'+10; else
	if ( ((*ss)>='a') && ((*ss)<='f') ) p=(*ss)-'a'+10;
	char code[6];
	strcpy(code, prefix[p]);
	code[2]=ss[1];
	code[3]=ss[2];
	code[4]=ss[3];
	code[5]=0;
	 
	if (strcmp(code, "P0000")!=0)
	{
		strcpy(DTC[dtclen].code,code);
	    has_dtc=true;
		dtclen++;
	}
	ss+=4;
    }
    
    return true;
}

char *OBD2::strip_answer(char *s)
{
    char *ss;
    for (ss=s; *s; s++)
    {
	if ( ((*s)!=' ') && ((*s)!='\t') && ((*s)!='\n') && ((*s)!='\r') )
	    (*ss++)=(*s);
    }
    (*ss)=0;
	
	return s;
}



