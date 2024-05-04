#include <stdio.h>
#include <math.h>


#define R 6371 // Radius of Earth in kilometers
#define TO_RAD (3.1415926536 / 180) // Conversion factor to convert degrees to radians
#define TO_FEET 3280.84 // Conversion factor to convert kilometers to feet

double haversine(double lat1, double lon1, double lat2, double lon2) {
    // Convert latitude and longitude from degrees to radians
    lat1 *= TO_RAD;
    lon1 *= TO_RAD;
    lat2 *= TO_RAD;
    lon2 *= TO_RAD;

    // Haversine formula
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = R * c;

    // Convert the distance to feet
    return distance * TO_FEET;
}