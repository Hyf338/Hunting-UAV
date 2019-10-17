#pragma once

#include "AC_Math.h"

//经纬度转平面坐标算法
//摘自PX4

typedef struct
{
	double phi_1;
	double sin_phi_1;
	double cos_phi_1;
	double lambda_0;
	double scale;
}Map_Projection;

static inline void map_projection_init(double lat_0, double lon_0 , Map_Projection* m) //lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
{
	/* notation and formulas according to: http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html */
	m->phi_1 =  degree2rad_double( lat_0 );
	m->lambda_0 = degree2rad_double( lon_0 );

	m->sin_phi_1 = sin(m->phi_1);
	m->cos_phi_1 = cos(m->phi_1);

	/* calculate local scale by using the relation of true distance and the distance on plane */ //TODO: this is a quick solution, there are probably easier ways to determine the scale

	/* 1) calculate true distance d on sphere to a point: http://www.movable-type.co.uk/scripts/latlong.html */
	const double r_earth = 6371000;

	double lat1 = m->phi_1;
	double lon1 = m->lambda_0;

	double lat2 = m->phi_1 + degree2rad_double( 0.5 );
	double lon2 = m->lambda_0 + degree2rad_double( 0.5 );
	double sin_lat_2 = sin(lat2);
	double cos_lat_2 = cos(lat2);
	double d = acos(sin(lat1) * sin_lat_2 + cos(lat1) * cos_lat_2 * cos(lon2 - lon1)) * r_earth;

	/* 2) calculate distance rho on plane */
	double k_bar = 0;
	double c =  acos(m->sin_phi_1 * sin_lat_2 + m->cos_phi_1 * cos_lat_2 * cos(lon2 - m->lambda_0));

	if (0 != c)
		k_bar = c / sin(c);

	double x2 = k_bar * (cos_lat_2 * sin(lon2 - m->lambda_0)); //Projection of point 2 on plane
	double y2 = k_bar * ((m->cos_phi_1 * sin_lat_2 - m->sin_phi_1 * cos_lat_2 * cos(lon2 - m->lambda_0)));
	double rho = sqrt(pow(x2, 2) + pow(y2, 2));

	m->scale = d / rho;

}

/**
 * Transforms a point in the geographic coordinate system to the local azimuthal equidistant plane
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567? not 471234567?
 * @param lon in degrees (8.1234567? not 81234567?
 */
static void map_projection_project(double lat, double lon, double *x, double *y , const Map_Projection* m)
{
	/* notation and formulas accoring to: http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html */
	double phi = degree2rad_double( lat );
	double lambda = degree2rad_double( lon );

	double sin_phi = sin(phi);
	double cos_phi = cos(phi);

	double k_bar = 0;
	/* using small angle approximation (formula in comment is without aproximation) */
	double c =  acos(m->sin_phi_1 * sin_phi + m->cos_phi_1 * cos_phi * (1 - pow((lambda - m->lambda_0), 2) / 2)); //double c =  acos( sin_phi_1 * sin_phi + cos_phi_1 * cos_phi * cos(lambda - lambda_0) );

	if (0 != c)
		k_bar = c / sin(c);

	/* using small angle approximation (formula in comment is without aproximation) */
	*y = k_bar * (cos_phi * (lambda - m->lambda_0)) * m->scale;//*y = k_bar * (cos_phi * sin(lambda - lambda_0)) * scale;
	*x = k_bar * ((m->cos_phi_1 * sin_phi - m->sin_phi_1 * cos_phi * (1 - pow((lambda - m->lambda_0), 2) / 2))) * m->scale; //	*x = k_bar * ((cos_phi_1 * sin_phi - sin_phi_1 * cos_phi * cos(lambda - lambda_0))) * scale;

//	printf("%phi_1=%.10f, lambda_0 =%.10f\n", phi_1, lambda_0);
}

/**
 * Transforms a point in the local azimuthal equidistant plane to the geographic coordinate system
 *
 * @param x north
 * @param y east
 * @param lat in degrees (47.1234567? not 471234567?
 * @param lon in degrees (8.1234567? not 81234567?
 */
static void map_projection_reproject(float x, float y, double *lat, double *lon , const Map_Projection* m)
{
	/* notation and formulas accoring to: http://mathworld.wolfram.com/AzimuthalEquidistantProjection.html */

	double x_descaled = x / m->scale;
	double y_descaled = y / m->scale;

	double c = sqrt(pow(x_descaled, 2) + pow(y_descaled, 2));
	double sin_c = sin(c);
	double cos_c = cos(c);

	double lat_sphere = 0;

	if (c != 0)
		lat_sphere = asin(cos_c * m->sin_phi_1 + (x_descaled * sin_c * m->cos_phi_1) / c);
	else
		lat_sphere = asin(cos_c * m->sin_phi_1);

//	printf("lat_sphere = %.10f\n",lat_sphere);

	double lon_sphere = 0;

	if (m->phi_1  == M_PI * 0.5) {
		//using small angle approximation (formula in comment is without aproximation)
		lon_sphere = (m->lambda_0 - y_descaled / x_descaled); //lon_sphere = (lambda_0 + atan2(-y_descaled, x_descaled));

	} else if (m->phi_1 == -M_PI * 0.5) {
		//using small angle approximation (formula in comment is without aproximation)
		lon_sphere = (m->lambda_0 + y_descaled / x_descaled); //lon_sphere = (lambda_0 + atan2(y_descaled, x_descaled));

	} else {

		lon_sphere = (m->lambda_0 + atan2(y_descaled * sin_c , c * m->cos_phi_1 * cos_c - x_descaled * m->sin_phi_1 * sin_c));
		//using small angle approximation
//    	double denominator = (c * cos_phi_1 * cos_c - x_descaled * sin_phi_1 * sin_c);
//    	if(denominator != 0)
//    	{
//    		lon_sphere = (lambda_0 + (y_descaled * sin_c) / denominator);
//    	}
//    	else
//    	{
//    	...
//    	}
	}
//	printf("lon_sphere = %.10f\n",lon_sphere);

	*lat = lat_sphere * 180.0 / M_PI;
	*lon = lon_sphere * 180.0 / M_PI;

}