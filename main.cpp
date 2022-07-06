#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
/*
   This sample code demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 5, TXPin = 6;
static const uint32_t GPSBaud = 9600;
const double a = 6378137.0; // length of semi-major axis of the ellipsoid (radius at equator) in meters
const double f = 1.0/298.257223563; // flattening of the ellipsoid
const double b = 6356752.314245; // = (1 - f) * a, length of semi-minor axis of the ellipsoid (radius at the poles) in meters
//#DEFINE PI 3.14159265358979

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
double distanceBetweenTest(double lat1, double long1, double lat2, double long2)
{
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}
double calculateDistance(double lat0, double lng0, double lat1, double lng1) {
  /*
  double theta0 = lat0 * 0.017453292519943295;
  double theta1 = lat1 * 0.017453292519943295;
  double phi0 = lng0 * 0.017453292519943295;
  double phi1 = lng1 * 0.017453292519943295;
  */
  double theta0 = radians(lat0);
  double theta1 = radians(lat1);
  double phi0 = radians(lng0);
  double phi1 = radians(lng1);
  Serial.println(theta0, 10);
  Serial.println(phi0, 10);
  Serial.println(theta1, 10);
  Serial.println(phi1, 10);
  double u1 = atan((1 - f) * tan(theta0));
  double u2 = atan((1 - f) * tan(theta1));
  double dLng = radians(lng1 - lng0);
  double lambda0 = dLng;
  double lambda1 = 0;

  int n = 0;

  double sino = 0;
  double coso = 0;
  double o = 0;
  double sina = 0;
  double cos2a = 0;
  double cos2om = 0;
  double c = 0;

  Serial.println(dLng, 10);
  while (abs(lambda0 - lambda1) > 1e-12) {
    n = n + 1;
    Serial.println(u1, 10);
    Serial.println(u2, 10);
    Serial.println(lambda0, 10);
    sino = sqrt((sq((cos(u2) * sin(lambda0)))) + (sq((cos(u1) * sin(u2) - sin(u1) * cos(u2) * cos(lambda0)))));
    Serial.println(sino, 10);
    coso = sin(u1) * sin(u2) + cos(u1) * cos(u2) * cos(lambda0);
    Serial.println(coso, 10);
    o = atan2(sino, coso);
    Serial.println(o, 10);
    sina = (cos(u1) * cos(u2) * sin(lambda0)) / (sino);
    Serial.println(sina, 10);
    cos2a = 1.0 - sq(sina);
    Serial.println(cos2a, 10);
    cos2om = coso - (2.0 * sin(u1) * sin(u2)) / (cos2a);
    Serial.println(cos2om, 10);
    c = (f * cos2a / 16.0) * (4.0 + f * (4.0 - 3.0 * cos2a));
    Serial.println(c, 10);
    lambda1 = lambda0;
    lambda0 = dLng + (1.0 - c) * f * sina * (o + c * sino * (cos2om + c * coso * (-1.0 + 2.0 * sq(cos2om))));
    Serial.println(lambda0, 10);
  }
  Serial.println("Out of loop");
  
  double usquared = (cos2a * cos2a) * ((a * a - b * b) / (b * b));
  double biga = 1.0 + (usquared / 16384.0) * (4096.0 + usquared * (-786.0 + usquared * (320.0 - 175.0 * usquared)));
  double bigb = (usquared / 1024.0) * (256.0 + usquared * (-128.0 + usquared * (74.0 - 47.0 * usquared)));
  double dsigma = bigb * sino * (cos2om + 0.25 * bigb * (coso * (-1.0 + 2.0 * (cos2om * cos2om)) - (bigb / 6.0) * (cos2om) * (-3.0 + 4.0 * (sino * sino)) * (-3.0 + 4.0 * (cos2om * cos2om))));
  double s = b * biga * (o - dsigma);
  Serial.println(s, 10);
  double sdf = sin(3.141 / 2.123);
  Serial.println(sdf);
  return s;
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

void setup()
{
  Serial.begin(115200);
  //ss.begin(GPSBaud);
/*
  Serial.println(F("FullExample.ino"));
  Serial.println(F("An extensive example of many interesting TinyGPSPlus features"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));
  */
 double d0 = calculateDistance(30.2469749450, -95.5293045043, 30.2469749450, -95.5293121337);
 double d1 = distanceBetweenTest(30.2469749450, -95.5293045043, 30.2469749450, -95.5293121337);
 Serial.print("Vincenty:");
 Serial.println(d0, 10);
 Serial.print("Default:");
 Serial.println(d1, 10);
}
double previousLat = gps.location.lat();
double previousLng = gps.location.lng();
double totalDistance = 0;

void loop()
{
  /*
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(100);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
    */


/*
   
   Serial.println("Start");
   double currentLat = gps.location.lat();
   double currentLng = gps.location.lng();
   double distance = TinyGPSPlus::distanceBetween(currentLat, currentLng, previousLat, previousLng);
   Serial.println(previousLat, 10);
   Serial.println(previousLng, 10);
   //Serial.println("-----");
   Serial.println(currentLat, 10);
   Serial.println(currentLng, 10);
   Serial.println("Distance:");
   Serial.println(distance, 10);
   Serial.println("End");
   if(gps.location.isValid() && previousLat != 0 && distance > 2.5) {
    totalDistance += distance;
   }
   Serial.println("Total distance:");
   Serial.println(totalDistance);
   previousLat = currentLat;
   previousLng = currentLng;
   smartDelay(1000);
   */


  
  
}