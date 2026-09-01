#pragma once
#define _CRT_SECURE_NO_DEPRECATE

#ifndef RAYTRACING_H
#define RAYTRACING_H

#include "aspherical.h"
#include <math.h>
#include <iostream>
#include <stdio.h>   
#include <stdlib.h>
#include <cstring>

#define OPTICS_MAX_CHILD_RAYS 0
#define OPTICS_MAX_REFLECTIONS 0
static double backplane_Z = -30.0;
static double frontplane_Z = 40.0;
static double sideplanes_X = 50.0;
static double verticalplanes_Y = 60.0;

//Sphere (spherical cap) centered on optical (z-) axis; cutoff above a certain radius from the axis and by z_min,z_max bounds
struct axisphere {
	double radius;
	double center_z;
	double enclosing_cylinder_radius;
	double z_min;
	double z_max;
	double n_1 = 1.0;
	double n_2 = 1.0;
	double normal_multiplier;
	bool bbox_active = false;
	double boundingbox_details[6];
};

//Cylinder of radius r on and aligned with optical (z-) axis; from z_min to z_max
struct axicylinder {
	double radius;
	double z_min;
	double z_max;
	double n_1 = 1.0;
	double n_2 = 1.0;
	double normal_multiplier;
	bool bbox_active = false;
	double boundingbox_details[6];
};

//Plane is defined by NORMALIZED perpendicular vector u and vector to point on plane v
struct plane {
	double u_x;
	double u_y;
	double u_z;
	double v_x;
	double v_y;
	double v_z;
	double n_1 = 1.0;
	double n_2 = 1.0;
	double normal_multiplier;
	bool bbox_active = false;
	double boundingbox_details[6];
};

//Axisymmetric flat circle/annulus from r_in to r_out; normal_mulitplier: +1 if facing increasing in Z, -1 if facing decreasing Z
struct axicircle {
	double r_in;
	double r_out;
	double z_pos;
	double n_1 = 1.0;
	double n_2 = 1.0;
	double normal_multiplier;
	bool bbox_active = false;
	double boundingbox_details[6];
};

void setup_ray_trace(
	bool *&ray_running,
	bool *&ray_alive,
	double *&intensity, 
	int *&num_reflections,
	double *&out_orig,
	double *&out_dir
);

void cleanup_ray_trace(
	bool *&ray_running,
	bool *&ray_alive,
	double *&intensity, 
	int *&num_reflections,
	double *&out_orig,
	double *&out_dir
);

void ray_trace_optic(
	double *ray_orig,
	double *ray_dir,
	int &num_rays,
	bool *ray_running,
	bool *ray_alive,
	double *intensity,
	int *num_reflections, 
	double *&out_orig,
	double *&out_dir,
	int num_axispheres,
	axisphere *axisphere_properties,
	int num_axicylinders,
	axicylinder *axicylinder_properties,
	int num_planes,
	plane *plane_properties,
	int num_axicircles,
	axicircle *axicircle_properties,
	int num_axiaspheres,
	axiasphere *axiasphere_properties
);

void process_ray_end(
	double &R_intensity,
	double &G_intensity,
	double &B_intensity,
	bool *ray_alive,
	double *intensity,
	int *num_reflections, 
	double *&out_orig,
	double *&out_dir
);

void asphere_example1(
	axiasphere &properties
);

void asphere_example2(
	axiasphere **properties_a,
	axicylinder **properties_y,
	axicircle **properties_i
);

void asphere_example3(
	axiasphere **properties_a,
	axicylinder **properties_y,
	axicircle **properties_i
);

void setup_walls(
	int &num_planes,
	plane *&plane_properties
);

void cleanup_walls(
	plane *&plane_properties
);

#endif //RAYTRACING_H

