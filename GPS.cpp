/********************************************************************
* GPS.cpp - Libary for using the GPS module - LS20126 from Sparkfun *
*********************************************************************/

#if defined(ARDUINO) && ARDUINO >= 100
// Choose Arduino.h for IDE 1.0
#include "Arduino.h"
#else
// Choose WProgram.h if IDE is older than 1.0
#include "WProgram.h"
#endif

#include "RR_GPS.h"
#include <math.h>
char byteGPS      = -1;
char buffer[300]  = "";
int  bufferIndex  = 0;
RR_GPS::RR_GPS(void)
#if USING_GPS_SOFTWARESERIAL
 :gps_SoftSerial(GPS_RX_PIN,GPS_TX_PIN)
#endif

{
    //_baud = baud;
    
    pinMode(GPS_ENABLE_PIN,OUTPUT);
    _new_term=false;
	On=false;
    
}

void RR_GPS::Enable()
{
	setTarget(TARGET_LAT, TARGET_LON);
    digitalWrite(GPS_ENABLE_PIN,HIGH);
    Serial.println(GGA_OFF);
	delay(200);
	//Serial.println(INIT);
    //delay(200);
    Serial.println(GLL_OFF);
    delay(200);
    Serial.println(GSA_OFF);
    delay(200);
    Serial.println(GSV_OFF);
    delay(200);
    Serial.println(VTG_OFF);
    delay(200);
    Serial.println(PLSR_ON_5HZ);
	delay(200);
	//Serial.println(BAUD57600);
	//delay(200);
	//Serial.begin(57600);
    char temp = '\0';
    while(temp != '$')
    {
        if(Serial.available()){
            temp = Serial.read();
        }
    }
	On=true;
}

void RR_GPS::Disable()
{
    digitalWrite(GPS_ENABLE_PIN,LOW);
}

uint8_t RR_GPS::readMessage(boolean readRMC)
{
    char RMCBuffer[300] = "", PLSRBuffer[300] = "";
    boolean RMC_completed = false, PLSR_completed = false;
    while(1)
    {
        if(Serial.available())
        {        
		
            byteGPS = Serial.read();                  // Read a byte of the serial port
			//if(DBUG) Serial.print(byteGPS);
            if(byteGPS == -1)
            {                        // Wait 100ms if Port empty
                if(DBUG) Serial.println(F("Empty Port"));
				return 0; 
            }
            else
            {
                if(byteGPS=='$')
                {
                    _new_term=true;
                    _term_type=_TERM_OTHER;
				//	Serial.println("New Term");
                }
                else if(_new_term)                            // $ sign not carried to buffer
                {
                    switch(_term_type)
                    {
                    case _TERM_OTHER:                         //New Term. Store enough characters to determine if of interest.
                        buffer[bufferIndex] = byteGPS;        // Store valid Serial data in Buffer
                        bufferIndex++;
                        
                        if(bufferIndex==10)
                        {    
                            if(strncmp(buffer,_PLSR_COMPASS1_ID,10)==0&&!PLSR_completed)
                            {
                                strcpy(PLSRBuffer,buffer);
                                buffer[0] = '\0';
                                _term_type=_TERM_PLSR;
                            }
                            else if(strncmp(buffer,_RMC_ID,5)==0&&!RMC_completed)
                            {
                                if(readRMC){
                                    strcpy(RMCBuffer,buffer);
                                    buffer[0] = '\0';
                                    _term_type=_TERM_RMC;
                                }
                                else
                                {
                                    _new_term=false;
                                    _term_type=_TERM_OTHER;
                                    bufferIndex = 0;
                                    buffer[0] = '\0';
                                }
                            }
                            else // Message not of interest 
                            {
                            //    Serial.println("Message Not Interesting");
                                _new_term=false;
                                _term_type=_TERM_OTHER;
                                bufferIndex = 0;
                                buffer[0] = '\0';
                            }
                        }
                        break;
                    case _TERM_RMC:
                        RMCBuffer[bufferIndex]=byteGPS;
                        bufferIndex++;
                        //Serial.write(byteGPS);
                        if(byteGPS == 10)
                        {    
							//Serial.write(RMCBuffer);
                            RMCBuffer[bufferIndex]='\0';
                            bufferIndex=0;
                            _new_term=false;
                            if(DBUG) Serial.println(F("RMC Reading completed"));
                            RMC_completed = true;
                        }
                        break;
                    case _TERM_PLSR:
                        PLSRBuffer[bufferIndex]=byteGPS;
                        bufferIndex++;                                    
                        if(byteGPS==10)
                        {
                         //   Serial.write(PLSRBuffer);
                            PLSRBuffer[bufferIndex]='\0';
                            bufferIndex=0;
                            _new_term=false;
                         //   Serial.println("PLSR Reading completed");
                            PLSR_completed = true;
                        }
                        break;
                    }
                }
            }            
        }
        if((PLSR_completed&&!readRMC)||RMC_completed&&PLSR_completed)
        {
            if(readRMC){
                extractData(RMCBuffer,RMC);
				extractData(PLSRBuffer,PLSR);
                if(status[0]!='A'){ 
					if(gps_fix)
					{
						gps_fix=false;
					}

                    RMC_completed = false;
                    Serial.println(F("Invalid RMC data. Reading again!"));
					break;
                }
				else
				{
                    if(!gps_fix)
					{
						gps_fix=true;
						
					}
					Serial.println(F("UpdateTarget"));
					updateTargetDistance();
					updateTargetHeading();
                    break;
                }
            }
			else
			{
                extractData(PLSRBuffer,PLSR);
				if(gps_fix)
				{
					updateTargetHeading();
				}
                break;
            }
        }
    }
    return 1;
}

void RR_GPS::storeRMCData(char value[], int segment)
{
    switch(segment)
    {
    case TIME:
        strcpy(time,value);
        break;
    case STATUS:
        strcpy(status,value);
        break;
    case LATITUDE:
        latitude = atof(value);
        break;
    case NS:
        if(value[0] == 'S')
            latitude *= -1;
        break;
    case LONGITUDE:
        longitude = atof(value);
        break;
    case EW:
        if(value[0] == 'W')
            longitude *= -1;
        break;
    }
}

void RR_GPS::storePLSRData(char value[], int segment)
{
    switch(segment)
    {
    case DIR:
        dir = atof(value);
        break;

    case AX:
        ax = atof(value);
        break;
    case AY:
        ay = atof(value);
        break;
    case AZ:
        az = atof(value);
        break;
    }
}

void RR_GPS::extractData(char line[], int type)
{
    int index = 0, segment = 0, indexTemp = 0;
    char segmentData[15] = "";
    while(line[index]){
        if(line[index] == ','){
            segmentData[indexTemp] = '\0';
            if(type == RMC){
                storeRMCData(segmentData,segment);
            }else if(type == PLSR){
                storePLSRData(segmentData,segment);
            }
            segment++;
            indexTemp = 0;
        }else{
            segmentData[indexTemp] = line[index];
            indexTemp++;
            if(line[index] == '*'){
                // Checksum is being ignored here.
                return;
            }
        }
        index++;
    }
}
/*
void RR_GPS::displayData(int type)
{
    if(type == RMC)
    {
			
        Serial.println(F(" Printing GPRMC Report"));
        Serial.print(F(" Time: "));
        Serial.println(time);
        Serial.print(F(" Status: "));
        Serial.println(status[0]);
        Serial.print(F(" Latitude: "));
        Serial.println(latitude,4);
        Serial.print(F(" Longitude: "));
        Serial.println(longitude,4);
        Serial.print(F(" Speed: "));
        Serial.println(spd);
        Serial.print(F(" Course: "));
        Serial.println(course);
        Serial.print(F(" Date: "));
        Serial.println(date);
		
    }
    else if(type == PLSR)
    {
		
        Serial.println(F(" Printing PLSR Report"));
        Serial.print(F(" Direction: "));
        Serial.println(dir);
        Serial.print(F(" Calib Status: "));
        Serial.println(cstatus);
        Serial.print(F(" Field Intensity: "));
        Serial.println(fint);
        Serial.print(F(" Ax: "));
        Serial.println(ax);
        Serial.print(F(" Ay: "));
        Serial.println(ay);
        Serial.print(F(" Az: "));
        Serial.println(az);
        Serial.print(F(" Temp: "));
        Serial.println(temperature);
        Serial.print(F(" Mode: "));
        Serial.println(mm);
        Serial.print(F(" CCD Status: "));
        Serial.println(ccds);
		
    }   
}
*/
void RR_GPS::setTarget(float latitude, float longitude)
{
  target_latitude = latitude;
  target_longitude = longitude;
}

void RR_GPS::updateTargetDistance()
{
  // Using Haversine formula for calculating Over-the-surface distance
  // Source: http://www.movable-type.co.uk/scripts/latlong.html
  int R = 6371;   // Mean Earth radius
  float lat1 = toRad(latitude);
  float lat2 = toRad(target_latitude);
  float dlat = lat2-lat1, dlong = toRad(target_longitude)-toRad(longitude);
  float a = sin(dlat/2)*sin(dlat/2)+sin(dlong/2)*sin(dlong/2)*cos(lat1)*cos(lat2);
  float c = 2*atan2(sqrt(a),sqrt(1-a));
  distance_rem = R*c;
  // Using the formula for bearing to calculate the required current heading.
  float y = sin(dlong)*cos(lat2);
  float x = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlong);
  _bearing = fmod((atan2(y,x)*57.295779513+360),360); // Multiplied by constant to convert to decimal degrees.
  
  
}

void RR_GPS::updateTargetHeading()
{
	delta_heading_reqd = _bearing - dir;
}

float RR_GPS::toRad(float value)
{
  float min = fmod(value, 100.0)/60.0;
  float deg = float(int(value/100));
  return (deg+min)*PI/180.0;
}
