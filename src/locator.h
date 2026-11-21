// Positions for display: Maidenhead locators, and distance and bearing
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef _LOCATOR_H
#define _LOCATOR_H 1
#include <stdbool.h>
#include <stddef.h>

// The 6-character locator (FN02qu) of a position, degrees north and east
void latlon_to_locator(double lat,double lon,char *out,size_t size);
// "42.835,-78.607" or a 4-, 6- or 8-character locator (its centre)
bool parse_location(char const *s,double *lat,double *lon);
// Great circle distance (km) and initial bearing (degrees from north) from 1 to 2
void distance_bearing(double lat1,double lon1,double lat2,double lon2,double *km,double *bearing);
#endif
