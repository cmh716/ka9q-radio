// Positions for display: Maidenhead locators, and distance and bearing
// SPDX-License-Identifier: GPL-3.0-or-later
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "locator.h"

static double const Earth_radius_km = 6371.0;

void latlon_to_locator(double lat,double lon,char *out,size_t size){
  if(size < 7 || !(fabs(lat) <= 90) || !(fabs(lon) <= 180)){
    if(size > 0)
      out[0] = '\0';
    return;
  }
  double x = lon + 180, y = lat + 90;
  if(x >= 360) x = 360 - 1e-9; // the east and north edges
  if(y >= 180) y = 180 - 1e-9;
  out[0] = (char)('A' + (int)(x / 20));
  out[1] = (char)('A' + (int)(y / 10));
  x = fmod(x,20); y = fmod(y,10);
  out[2] = (char)('0' + (int)(x / 2));
  out[3] = (char)('0' + (int)y);
  x = fmod(x,2); y = fmod(y,1);
  out[4] = (char)('a' + (int)(x * 12)); // 5' of longitude
  out[5] = (char)('a' + (int)(y * 24)); // 2.5' of latitude
  out[6] = '\0';
}

// A locator's centre
static bool locator_to_latlon(char const *s,double *lat,double *lon){
  size_t const n = strlen(s);
  if(n != 4 && n != 6 && n != 8)
    return false;
  int const a = toupper((unsigned char)s[0]) - 'A', b = toupper((unsigned char)s[1]) - 'A';
  if(a < 0 || a > 17 || b < 0 || b > 17 || !isdigit((unsigned char)s[2]) || !isdigit((unsigned char)s[3]))
    return false;
  double x = a * 20 + (s[2] - '0') * 2, y = b * 10 + (s[3] - '0');
  double w = 2, h = 1; // the square's size
  if(n >= 6){
    int const c = tolower((unsigned char)s[4]) - 'a', d = tolower((unsigned char)s[5]) - 'a';
    if(c < 0 || c > 23 || d < 0 || d > 23)
      return false;
    w /= 24; h /= 24;
    x += c * w; y += d * h;
  }
  if(n == 8){
    if(!isdigit((unsigned char)s[6]) || !isdigit((unsigned char)s[7]))
      return false;
    w /= 10; h /= 10;
    x += (s[6] - '0') * w; y += (s[7] - '0') * h;
  }
  *lon = x + w / 2 - 180;
  *lat = y + h / 2 - 90;
  return true;
}

bool parse_location(char const *s,double *lat,double *lon){
  if(s == NULL)
    return false;
  char const *comma = strchr(s,',');
  if(comma == NULL)
    return locator_to_latlon(s,lat,lon);
  char *end;
  double const y = strtod(s,&end);
  if(end != comma)
    return false;
  double const x = strtod(comma + 1,&end);
  while(isspace((unsigned char)*end))
    end++;
  if(*end != '\0' || !(fabs(y) <= 90) || !(fabs(x) <= 180))
    return false;
  *lat = y;
  *lon = x;
  return true;
}

void distance_bearing(double lat1,double lon1,double lat2,double lon2,double *km,double *bearing){
  double const p1 = lat1 * M_PI / 180, p2 = lat2 * M_PI / 180, dl = (lon2 - lon1) * M_PI / 180;
  double const a = sin((p2 - p1) / 2) * sin((p2 - p1) / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  *km = 2 * Earth_radius_km * atan2(sqrt(a),sqrt(1 - a));
  double const t = atan2(sin(dl) * cos(p2),cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl)) * 180 / M_PI;
  *bearing = fmod(t + 360,360);
}
