// Maidenhead locators, location parsing, distance and bearing (locator.c)
// SPDX-License-Identifier: GPL-3.0-or-later
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "locator.h"

static int Failures;
static void check(int ok,char const *what){
  printf("%s %s\n",ok ? "ok  " : "FAIL",what);
  Failures += !ok;
}
int main(void){
  char g[8];
  latlon_to_locator(42.8353,-78.6075,g,sizeof g); // Alden, NY
  check(strcmp(g,"FN02qu") == 0,"Alden NY is FN02qu");
  latlon_to_locator(51.4769,-0.0005,g,sizeof g);  // Greenwich, just west of the meridian
  check(strcmp(g,"IO91xl") == 0,"Greenwich is IO91xl");
  latlon_to_locator(-33.8688,151.2093,g,sizeof g); // Sydney
  check(strcmp(g,"QF56od") == 0,"Sydney is QF56od");
  latlon_to_locator(90,180,g,sizeof g);
  check(strcmp(g,"RR99xx") == 0,"the north-east corner stays inside");
  latlon_to_locator(NAN,0,g,sizeof g);
  check(g[0] == '\0',"no position, no locator");

  double lat,lon;
  check(parse_location("42.8353,-78.6075",&lat,&lon) && fabs(lat - 42.8353) < 1e-9 && fabs(lon + 78.6075) < 1e-9,"lat,lon");
  check(parse_location("FN02",&lat,&lon) && fabs(lat - 42.5) < 1e-9 && fabs(lon + 79) < 1e-9,"a 4-character locator's centre");
  check(parse_location("fn02qu",&lat,&lon) && fabs(lat - (42 + 20.0 / 24 + 1.0 / 48)) < 1e-9 && fabs(lon - (-80 + 2 * 16.0 / 24 + 1.0 / 24)) < 1e-9,"a 6-character locator (its centre), either case");
  latlon_to_locator(lat,lon,g,sizeof g);
  check(strcmp(g,"FN02qu") == 0,"and back to itself");
  check(parse_location("FN02qu55",&lat,&lon),"an 8-character locator");
  check(!parse_location("ZZ99",&lat,&lon) && !parse_location("FN0",&lat,&lon) && !parse_location("91,0",&lat,&lon)
        && !parse_location("1,2x",&lat,&lon) && !parse_location("",&lat,&lon),"rejects bad ones");

  double km,b;
  distance_bearing(42.8353,-78.6075,42.8864,-78.8784,&km,&b); // Alden to Buffalo
  check(fabs(km - 22.8) < 0.5 && fabs(b - 284) < 2,"Alden to Buffalo: 23 km, 284 degrees");
  distance_bearing(51.5,0,40.7,-74,&km,&b); // London to New York
  check(fabs(km - 5570) < 30 && fabs(b - 288) < 2,"London to New York: 5570 km, 288 degrees");
  distance_bearing(0,0,0,0,&km,&b);
  check(km == 0,"no distance to oneself");
  puts(Failures ? "Locator tests FAILED" : "Locator tests passed");
  return Failures != 0;
}
