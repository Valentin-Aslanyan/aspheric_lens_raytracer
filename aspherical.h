#ifndef ASPHERICAL_H
#define ASPHERICAL_H


#define MIN_HIT_DISTANCE 0.00001
#define DEFAULT_STEP_SIZE 0.02
#include <math.h>


struct axiasphere {
	double conic_constant;
	double curvature_radius;
	double surface_radius;
	double z_offset;
	int num_aspheric_coefficients;
	double *aspheric_coefficients;
	int *aspheric_powers;
	double n_1;
	double n_2;
	double normal_multiplier;

	double bound_min_z;
	double bound_max_z;
	double bound_radius;
	double fractional_step_size;
};


double aspheric_surface_height(
	double rho_current,
	const axiasphere properties
);


bool rayintersect_axiasphere(
	double *ray_orig,
	double *ray_dir,	
	double &t_hit,
	double *hit_pos,
	double *hit_normal,
	const axiasphere properties
);


void set_axiasphere_bounds(
	axiasphere *properties
);

#endif //ASPHERICAL_H

