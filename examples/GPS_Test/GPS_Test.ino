#include <GPS.h>

GPS gps;

void setup() {
  Serial.begin(9600);
  Serial.println(gps.toRad(3347.4843));
  gps.Enable();
}

void loop() {
  Serial.println("Reading both messages");
  gps.readMessage(true);            // This will read the GPS and collect the GPRMC and PLSR report and update the variables
  gps.displayData(RMC);   // This can be used to display all the data of the data from GPRMC report
  gps.displayData(PLSR);    // This can be used to display all the data of the data from PLSR report
  Serial.print("Distance Remaining: ");
  Serial.println(gps.distance_rem);
  Serial.print("Difference in Heading: ");
  Serial.println(gps.delta_heading_reqd);
  Serial.print("Direction: ");
  Serial.println(gps.dir);
  delay(500);
  /*
  Serial.println("Reading only PLSR");
  gps.readMessage(false);            // This will read the GPS and collect the GPRMC and PLSR report and update the variables
  gps.displayData(PLSR);    // This can be used to display all the data of the data from PLSR report
  Serial.println("Waiting for 2 seconds");
  delay(2000);
  */
}
