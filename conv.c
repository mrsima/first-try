////////////////////////////////////////
// Utilities
//

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <inttypes.h>
//#include <compat.h>

//#define DEBUG
#define NMEA_BUF_LENGTH 0x48
#define GPS_FILTER_VECTOR_LENGTH 5
#define GPS_MUL 10000000.0f
#define NOINLINE __attribute__ ((noinline))

static float GPS_scaleLonDown; // this is used to offset the shrinking longitude as we go towards the poles

union {
    char string[NMEA_BUF_LENGTH]; // for NMEA parsing
    struct {
        int32_t lat;
        int32_t lon;
        int16_t alt;
        uint16_t hdop;
        int16_t course;
        uint16_t speed;
        uint8_t fix;
        uint8_t sats;
        uint32_t time;
        uint32_t date;
    } nmea;
//    byte bytes[0x40]; // for font uploading
} msg;


#define DIGIT_TO_VAL(_x)        (_x - '0')

uint32_t GPS_coord_to_degrees(char* s) {
#ifdef DEBUG
  printf("ParseD=%s\n",s);
#endif
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;
  uint8_t i;

  // scan for decimal point or end of field
  for (p = s; isdigit(*p); p++);
  q = s;

  // convert degrees
  while ((p - q) > 2) {
    if (deg)
      deg *= 10;
    deg += DIGIT_TO_VAL(*q++);
  }
  // convert minutes
  while (p > q) {
    if (min)
      min *= 10;
    min += DIGIT_TO_VAL(*q++);
  }
  // convert fractional minutes
  // expect up to four (five) digits, result is in
  // ten-thousandths of a minute
  if (*p == '.') {
    q = p + 1;
    for (i = 0; i < 5; i++) {
      frac_min *= 10;
      if (isdigit(*q))
        frac_min += *q++ - '0';
    }
  }
  printf("DD = %u , min = %u > .dd = %u , frac = %u > ..ddd = %u\n",deg,min,min*100/60,frac_min,frac_min*100/60);
  return deg * 10000000UL + (min * 10000000UL + frac_min * 100UL) / 60;
}


// helper functions 
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
#ifdef DEBUG
  printf("ParseG=%s\n",src);
#endif
  uint8_t i;
  uint16_t tmp = 0;

  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0)   break;
      else  src[i+mult] = 0;
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9') tmp += src[i]-'0';
  }
  return tmp;
}


uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
} 


// Parse a (potentially negative) number with up to 2 decimal digits -xxxx.yy
int32_t parseDecimal(const char *term)
{
#ifdef DEBUG
  printf("ParseP=%s\n", term);
#endif
  bool negative = *term == '-';
  if (negative) ++term;

  int32_t ret = 100 * (int32_t)atol(term);

  while (isdigit(*term)) ++term;
  if (*term == '.' && isdigit(term[1]))
  {
    ret += 10 * (term[1] - '0');
    if (isdigit(term[2]))
      ret += term[2] - '0';
  }
  return negative ? -ret : ret;
}


int32_t parseTime(const char *term)
{
#ifdef DEBUG
  printf("ParseT=%s\n", term);
#endif

  int32_t ret = (int32_t)atol(term);

  uint8_t h = ret / 10000;
  ret -= h*10000;
  uint8_t m=ret/100;
  ret-= m*100;

  return h * 3600 + m*60 + ret;  // time in seconds
}


#define FRAME_GGA  1
#define FRAME_RMC  2

bool parse_NMEA_char(char c, char *string) {
  uint8_t frameOK = 0;
  static uint8_t frame = 0, param = 0, offset = 0, parity = 0;
  static uint8_t checksum_param;
#ifdef DEBUG
  printf("FrameT=%d Param=%d Offset=%d Char=%c Str=%s\n",frame, param, offset, c, string);
#endif

    switch(c){
    case '$':
#ifdef DEBUG
  printf("Switch c Case=$\n",c);
#endif
        param = 0;     // start
        offset = 0;
        parity = 0;
        frame = 0;    // no frame
        break;

    case ',':
#ifdef DEBUG
  printf("Case=,\n",c);
#endif
    case '*':
#ifdef DEBUG
  printf("Case=*\n",c);
#endif
        string[offset] = 0;

        switch(param) { // words in sentence
        case 0:         // frame identification
#ifdef DEBUG
  printf("Switch param Case=0 FrameType\n");
#endif
            frame = 0;
//            if (string[0] == 'G' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
//            if (string[0] == 'G' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
            if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
            if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
#ifdef DEBUG
  printf("String 1=%c, 2=%c, 3=%c, 4=%c, 5=%c - Frame Type = %d\n",string[0],string[1],string[2],string[3],string[4],frame);
#endif
            break;

/*
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
$GPGGA,002153.000,3342.6618,N,11751.3858,W,1,10,1.2,27.0,M,-34.2,M,,0000*5E

$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
$GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10
*/
        case 1:
#ifdef DEBUG
  printf("Switch param Case=1 Time\n");
#endif
            if (frame !=0 ) { // time at 1st pos
                msg.nmea.time= parseTime(string);
#ifdef DEBUG
  printf("PTime=%d\n",msg.nmea.time);
#endif
            }
            break;

        case 2:
#ifdef DEBUG
  printf("Switch param Case=2 Lat\n");
#endif
            if (frame == FRAME_GGA) {msg.nmea.lat = GPS_coord_to_degrees(string);}
#ifdef DEBUG
  printf("LAT=%d\n",msg.nmea.lat);
#endif
            break;

        case 3:
#ifdef DEBUG
  printf("Switch param Case=3 S/E Lat\n");
#endif
            if (frame == FRAME_GGA && string[0] == 'S') msg.nmea.lat = -1*msg.nmea.lat;
            if (frame == FRAME_RMC) {msg.nmea.lat = GPS_coord_to_degrees(string);}
            break;

        case 4:
#ifdef DEBUG
  printf("Switch param Case=4 Lon S\n");
#endif
            if (frame == FRAME_GGA) {msg.nmea.lon = GPS_coord_to_degrees(string);}
            if (frame == FRAME_RMC  && string[0] == 'S') msg.nmea.lat = -msg.nmea.lat;
#ifdef DEBUG
            printf("LAT2=%d\n",msg.nmea.lon);
#endif
            break;

        case 5:
#ifdef DEBUG
  printf("Switch param Case=5 W/N Lon\n");
#endif
            if (frame == FRAME_GGA && string[0] == 'W') msg.nmea.lon = -msg.nmea.lon;
            if (frame == FRAME_RMC) {msg.nmea.lon = GPS_coord_to_degrees(string);}
            break;

        case 6:
#ifdef DEBUG
  printf("Switch param Case=6 Fix/W\n");
#endif
            if (frame == FRAME_GGA) {  msg.nmea.fix = grab_fields(string,0);}
            if (frame == FRAME_RMC && string[0] == 'W') msg.nmea.lon = -msg.nmea.lon;
            break;

        case 7:
#ifdef DEBUG
  printf("Switch param Case=7 Sats/Speed\n");
#endif
            if (frame == FRAME_GGA) {msg.nmea.sats = grab_fields(string,0);}
            if (frame == FRAME_RMC) {msg.nmea.speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation
            break;

        case 8:
#ifdef DEBUG
  printf("Switch param Case=8 Hdop/Course\n");
#endif
            if (frame == FRAME_GGA) {msg.nmea.hdop  = grab_fields(string,1);}
            if (frame == FRAME_RMC) {msg.nmea.course = grab_fields(string,0);}  //ground course deg
            break;

        case 9:
#ifdef DEBUG
  printf("Switch param Case=9 Alt/Date\n");
#endif
            if (frame == FRAME_GGA) {msg.nmea.alt = grab_fields(string,1);}  // altitude in meters added by Mis
            if (frame == FRAME_RMC) {msg.nmea.date = grab_fields(string,0);}
            break;
        }  // Switch (param) end

        param++; 
        offset = 0;
        if (c == '*') { 
            checksum_param = 1;
#ifdef DEBUG
  printf("CSum_param=%d\nParity=%d 0x%X\n",checksum_param, parity, parity);
#endif
        } else { 
            parity ^= c; 
        } 
        break;

        case '\r':
        case '\n':
        case '#':   // for testing from term

#ifdef DEBUG
  printf("----- CR/LF ---- Parity=%d 0x%X\n",parity, parity);
#endif
            if (checksum_param) {  //parity checksum
#ifdef DEBUG
  printf("CS_param=1\nParity=%d 0x%X\n",parity, parity);
#endif
                uint8_t checksum = hex_c(string[0]);
                checksum <<= 4;
                checksum += hex_c(string[1]);
#ifdef DEBUG
  printf("CheckSum=%d %X\n",checksum);
#endif
                if (checksum == parity) {
                    frameOK = 1;
#ifdef DEBUG
  printf("---------------- Eheeeeej checksum=parity = %d 0x%X ; FrameOK = %d ------------------\n",checksum,checksum,frameOK);
#endif
                }
            }  //parity checksum
            checksum_param = 0;
        break;

        default:
            if (offset < 15) {
//            printf("Offset<BUF=%d\n",offset);
            string[offset++] = c;
            }

            if (!checksum_param) parity ^= c; {
#ifdef DEBUG
  printf("CS_paramDef=%d Parity=%d 0x%X\n",checksum_param, parity, parity);
  printf("FrameOK!Def=%d\n",frameOK);
#endif
            }

    }  // Switch (c) end
    if(!frame) {
//#ifdef DEBUG
//        lost_bytes+=1;
//#endif
//        GPS_Present = 1;
    }
#ifdef DEBUG
  printf("RetFrameOK=%d\n",frameOK);
#endif
    return frameOK;
}



float GPS_calc_longitude_scaling(int32_t lat) {
  float rads       = (abs((float)lat) / 10000000.0) * 0.0174532925;
  GPS_scaleLonDown = cos(rads);
}


//void NOINLINE gps_norm(float &dst, long f) {
void NOINLINE gps_norm(float dst, long f) {
#ifdef DEBUG
  printf("GPS_N &dst=%d dst=%d *dst F=%d\n",&dst,dst,*dst,f);
#endif
//    dst = f / GPS_MUL;
    dst = f / GPS_MUL;
#ifdef DEBUG
  printf("DST=%f\n",dst);
#endif
}


int main() {
    uint8_t c;
    char string[NMEA_BUF_LENGTH];
    int32_t nmea;
    uint8_t frame, param, offset, parity, checksum, checksum_param;
    uint8_t frameOK;
    int32_t lat;
    int32_t lon;
    float latf;
    float lonf;
    int16_t alt;
    uint16_t hdop;
    int16_t course;
    uint16_t speed;
    uint8_t fix;
    uint8_t sats;
    int32_t time;
    uint32_t date;

    printf("Enter NMEA GGA or RMC packet: ");
    scanf("%s",string);
    printf("Entered: %s -> Pointer*: %u - Size=%u\n", string, &string, strlen(string));
    for (offset=0; offset<NMEA_BUF_LENGTH; offset++) {
        c = string[offset];
//        uint8_t c = string[offset];
//        printf("c%d=%c\n", offset, c);
        int32_t nmea = parse_NMEA_char(c, &string[0]);
    }
    GPS_calc_longitude_scaling(msg.nmea.lat);

    lat    = msg.nmea.lat;
    lon    = msg.nmea.lon;
//      gps_norm(latf, msg.nmea.lat);
//     gps_norm(lonf, msg.nmea.lon);
    latf   = ((float)msg.nmea.lat)/GPS_MUL;
    lonf   = ((float)msg.nmea.lon)/GPS_MUL;
    alt    = msg.nmea.alt * 10;
    hdop   = msg.nmea.hdop * 10;
    course = msg.nmea.course;
    speed  = msg.nmea.speed;
    fix    = msg.nmea.fix;
    sats   = msg.nmea.sats;
    time   = msg.nmea.time;
    date   = msg.nmea.date;

    printf("Return = %d\nTime = %d\nLatitude = %d - F = %.7f , %.7f\n", nmea, time, lat, latf, msg.nmea.lat/GPS_MUL);
    printf("Longitude = %d - F = %.7f , %.7f\nAlt = %d - %.2fm\n", lon, lonf, msg.nmea.lon/GPS_MUL, alt, ((float)alt)/100);
    printf("Scale = %f\nSpeed = %d\nCourse = %d\n", GPS_scaleLonDown, speed, course);
    printf("FixT = %d\nSats = %d\nHdop = %d\nDate = %d\n", fix, sats, hdop, date);
//    printf("FrameT = %d\nParam = %d\nOffset = %d\nParity = %X\nChkSum = %X\nCS_Param = %d\nFrameOK = %d\n", frame, param, offset, parity, checksum, checksum_param, frameOK );
    return 0;
}
