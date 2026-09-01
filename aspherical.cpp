/*
Functions to set up (once) the necessary parameters and to find the ray intersection point, normal 
*/


#include "aspherical.h"


//Cylinder centered on the optical (z-) axis, used as part of the bounding box around aspheres
static void rayintersect_boundingaxicylinder(
	double *ray_orig,
	double *ray_dir,	
	double &t_hit_min,
	double &t_hit_max,
	double radius,
	double z_min,
	double z_max
	) {

	t_hit_min = 0.0;
	t_hit_max = 0.0;
	double a = ray_dir[0]*ray_dir[0] + ray_dir[1]*ray_dir[1];
	double b = 2.0*(ray_orig[0]*ray_dir[0]+ray_orig[1]*ray_dir[1]);
	double c = ray_orig[0]*ray_orig[0] + ray_orig[1]*ray_orig[1] - radius*radius;
	double discriminant = b*b-4.0*a*c;
	double t_temp1, t_temp2;
	double z_temp1, z_temp2;

	if((discriminant<0.0) || (a==0.0)){
		return;	//No hit
	}
	else if(discriminant==0.0){	//Ray hits cylinder in just 1 location, at a tangent
		t_temp1 = -0.5*b/a;
		if(t_temp1>MIN_HIT_DISTANCE){
			z_temp1=ray_orig[2]+t_temp1*ray_dir[2];
			if(z_temp1<z_min || z_temp1>z_max){	//Check if within length of cylinder
				return;	//No hit
			}
		}
		else{
			return;	//No hit
		}
	}
	else{
		t_temp1=0.5/a*(sqrt(discriminant)-b);
		t_temp2=-0.5/a*(sqrt(discriminant)+b);
		z_temp1=ray_orig[2]+t_temp1*ray_dir[2];
		z_temp2=ray_orig[2]+t_temp2*ray_dir[2];
		if((t_temp1>MIN_HIT_DISTANCE) && (z_temp1>=z_min && z_temp1<=z_max)){
			if((t_temp2>MIN_HIT_DISTANCE) && (z_temp2>=z_min && z_temp2<=z_max)){
				if (t_temp2>t_temp1){
					t_hit_min = t_temp1;
					t_hit_max = t_temp2;
					return;
				}
				else{
					t_hit_min = t_temp2;
					t_hit_max = t_temp1;
					return;	
				}
			}
			else{
				t_hit_min = t_temp1;
				return;
			}
		}
		else{
			if((t_temp2>MIN_HIT_DISTANCE) && (z_temp2>=z_min && z_temp2<=z_max)){
				t_hit_min = t_temp2;
				return;
			}
			else{
				return;
			}
		}
	}

}


//Circle (competely filled) centered on the optical (z-) axis, used as part of the bounding box around aspheres
static bool rayintersect_boundingaxicircle(
	double *ray_orig,
	double *ray_dir,	
	double &t_hit,
	double r_out,
	double z_pos
	) {

	double t_temp;
	double x_temp, y_temp;
	if(ray_dir[2]==0.0){
		return false;
	}
	t_temp = (z_pos-ray_orig[2])/ray_dir[2];
	x_temp=ray_orig[0]+t_temp*ray_dir[0];
	y_temp=ray_orig[1]+t_temp*ray_dir[1];
	if((t_temp>MIN_HIT_DISTANCE) && (x_temp*x_temp + y_temp*y_temp <= r_out*r_out)){
		t_hit = t_temp;
		return true;
	}
	else{
		return false;
	}

}


double aspheric_surface_height(
	double rho_current,
	const axiasphere properties
	){

	double rho_squared = rho_current*rho_current;
	double height = rho_squared / (1.0 + sqrt(1.0 - (1.0+properties.conic_constant)*(rho_squared) / (properties.curvature_radius*properties.curvature_radius))) / properties.curvature_radius;

	for(int i=0; i<properties.num_aspheric_coefficients; ++i){
		height += pow(rho_current,properties.aspheric_powers[i])*properties.aspheric_coefficients[i];
	}

	return properties.z_offset + height;

}


//Whichever t_current causes this function to return zero is an intersection of the ray with an aspheric surface
static double aspheric_surface_distance(
	double *ray_orig,
	double *ray_dir,
	double t_current,
	const axiasphere properties
	){

	double z_current = ray_orig[2]+t_current*ray_dir[2];
	double rho_squared = (ray_orig[0]+t_current*ray_dir[0])*(ray_orig[0]+t_current*ray_dir[0]) + (ray_orig[1]+t_current*ray_dir[1])*(ray_orig[1]+t_current*ray_dir[1]);
	double rho_current = sqrt(rho_squared);

	double distance = rho_squared / (1.0 + sqrt(1.0 - (1.0+properties.conic_constant)*(rho_squared) / (properties.curvature_radius*properties.curvature_radius))) / properties.curvature_radius;

	for(int i=0; i<properties.num_aspheric_coefficients; ++i){
		distance += pow(rho_current,properties.aspheric_powers[i])*properties.aspheric_coefficients[i];
	}

	return properties.z_offset + distance - z_current;

}


static bool brent_solve_aspheric_surface_distance(
	double *ray_orig,
	double *ray_dir,
	double a,
	double b,
	double &result,
	const axiasphere properties,
	double max_tolerance=1E-6,
	int max_iterations=100
	){ 	

	double s, c, d, f_a, f_b, f_c, f_s, swap, f_swap;
	int flag=1,iteration_number=0; 

	f_a=aspheric_surface_distance(ray_orig, ray_dir, a, properties);
	f_b=aspheric_surface_distance(ray_orig, ray_dir, b, properties);
	if (f_a*f_b>0){	//Error - no (single) root between bounds
		return false;
	}
	if (fabs(f_b)>fabs(f_a)){ //Swap a and b
		swap=a;
		f_swap=f_a;
		a=b;
		f_a=f_b;
		b=swap;
		f_b=f_swap;
	}
	c=a;
	while (iteration_number<max_iterations && abs((b-a)/std::min(a,b))>max_tolerance && f_b!=0.0)		
	{
		if (f_a!=f_c && f_b!=f_c){
			s=a*f_b*f_c/((f_a-f_b)*(f_a-f_c))+b*f_a*f_c/((f_b-f_a)*(f_b-f_c))+c*f_a*f_b/((f_c-f_a)*(f_c-f_b));
		}
		else{
			s=b-f_b*(b-a)/(f_b-f_a);
		}
		if (
			(a>b && s<b && s>(3.0*a+b)/4.0) || 
			(a<b && s>b && s<(3.0*a+b)/4.0) || 
			(flag==1 && fabs(s-b)>=fabs(b-c)/2.0) || 
			(flag==0 && fabs(s-b)>=fabs(c-d)/2.0) ||
			(flag==1 && fabs(b-c)<max_tolerance) ||
			(flag==0 && fabs(c-d)<max_tolerance)
		){
			s=(a+b)/2.0;flag=1;
		}
		else{
			flag=0;
		}
		f_s=aspheric_surface_distance(ray_orig, ray_dir, s, properties);
		d=c;
		c=b;
		f_c=f_b;
		if (f_a*f_s<0){
			b=s;
			f_b=f_s;
		}
		else{
			a=s;
			f_a=f_s;
		}
		if (fabs(f_b)>fabs(f_a)){	//Swap a and b
			swap=a;
			f_swap=f_a;
			a=b;
			f_a=f_b;
			b=swap;
			f_b=f_swap;
		} 
		iteration_number++;
	}
	result=b;
	return true;
}


static void aspheric_surface_normal(
	double x_pos,
	double y_pos,
	double *hit_normal,
	const axiasphere properties
	){

	double rho_squared = x_pos*x_pos + y_pos*y_pos;
	double rho_current = sqrt(rho_squared);
	double temp = 1.0 / sqrt(1.0 - (1.0+properties.conic_constant)*(rho_squared) / (properties.curvature_radius*properties.curvature_radius)) / properties.curvature_radius;
	double x_part = x_pos*temp;
	double y_part = y_pos*temp;

	for(int i=0; i<properties.num_aspheric_coefficients; ++i){
		if(properties.aspheric_powers[i]>0){
			temp = properties.aspheric_powers[i]*pow(rho_current,properties.aspheric_powers[i]-1)*properties.aspheric_coefficients[i];
			x_part += x_pos*temp;
			y_part += y_pos*temp;
		}
	}

	temp = sqrt(1.0 + x_part*x_part + y_part*y_part);
	hit_normal[0] = x_part/temp;
	hit_normal[1] = y_part/temp;
	hit_normal[2] = 1.0/temp;

}


//Aspherical surface symmetric about the optical (z) axis
bool rayintersect_axiasphere(
	double *ray_orig,
	double *ray_dir,	
	double &t_hit,
	double *hit_pos,
	double *hit_normal,
	const axiasphere properties
	){

	double t_start, t_end;
	double bound_t1, bound_t2, bound_t3, bound_t4;
	bool bound_hit1, bound_hit2;
	if(	//Check if ray starts inside bounding cylindrical box
		ray_orig[2]>=properties.bound_min_z &&
		ray_orig[2]<=properties.bound_max_z &&
		ray_orig[0]*ray_orig[0] + ray_orig[1]*ray_orig[1] <= properties.bound_radius*properties.bound_radius
	){
		t_start = 0.0;
		if(rayintersect_boundingaxicircle(ray_orig, ray_dir, bound_t1, properties.bound_radius, properties.bound_min_z)) {
			t_end = bound_t1;
		}
		else if(rayintersect_boundingaxicircle(ray_orig, ray_dir, bound_t2, properties.bound_radius, properties.bound_max_z)){
			t_end = bound_t2;
		}
		else {
			rayintersect_boundingaxicylinder(ray_orig, ray_dir, bound_t3, bound_t4, properties.bound_radius, properties.bound_min_z, properties.bound_max_z);
			if(bound_t3 == 0.0) { //no valid hits
				return false;
			}
			else {
				t_end = bound_t3;
			}
		}
	}
	else {	//Check ray intersection of bounding cylindrical box - should be 2 hits (entrance and exit)
		bound_hit1 = rayintersect_boundingaxicircle(ray_orig, ray_dir, bound_t1, properties.bound_radius, properties.bound_min_z);
		bound_hit2 = rayintersect_boundingaxicircle(ray_orig, ray_dir, bound_t2, properties.bound_radius, properties.bound_max_z);
		if (bound_hit1) {
			if (bound_hit2) {	//Ray hits both circles
				if (bound_t2>bound_t1) {
					t_start=bound_t1;
					t_end=bound_t2;
				}
				else {
					t_start=bound_t2;
					t_end=bound_t1;
				}
			}
			else {
				rayintersect_boundingaxicylinder(ray_orig, ray_dir, bound_t3, bound_t4, properties.bound_radius, properties.bound_min_z, properties.bound_max_z);
				if(bound_t3 == 0.0) { //Ray only hits the bottom circle
					return false;
				}
				else{	//Ray hits the bottom circle and cylinder
					if (bound_t3>bound_t1) {
						t_start=bound_t1;
						t_end=bound_t3;
					}
					else {
						t_start=bound_t3;
						t_end=bound_t1;
					}
				}
			}
		}
		else {
			rayintersect_boundingaxicylinder(ray_orig, ray_dir, bound_t3, bound_t4, properties.bound_radius, properties.bound_min_z, properties.bound_max_z);
			if (bound_hit2) {
				if(bound_t3 == 0.0) { //Ray only hits the top circle
					return false;
				}
				else{	//Ray hits the top circle and cylinder
					if (bound_t3>bound_t2) {
						t_start=bound_t2;
						t_end=bound_t3;
					}
					else {
						t_start=bound_t3;
						t_end=bound_t2;
					}
				}
			}
			else {
				if(bound_t4 == 0.0) { //Ray either misses all surfaces, or only hits cylinder once, so effectively a miss
					return false;
				}
				else {	//Ray hits cylinder twice
					t_start=bound_t3;
					t_end=bound_t4;
				}
			}
		}
	}

	double delta_t = properties.fractional_step_size*(t_end-t_start);
	double t_current, t_old, dist_current, dist_old;

	bool solution_reached = false;
	t_current = t_start;
	dist_current = aspheric_surface_distance(ray_orig, ray_dir, t_current, properties);
	while(t_current<t_end && !solution_reached){
		t_old = t_current;
		t_current += delta_t;
		dist_old = dist_current;
		dist_current = aspheric_surface_distance(ray_orig, ray_dir, t_current, properties);
		if(dist_old*dist_current<0.0){	//Change of sign of distance function, so there is a solution between old and current
			if(!brent_solve_aspheric_surface_distance(ray_orig, ray_dir, t_old, t_current, t_hit, properties)){
				//Brent method has failed for some reason, return an estimate
				t_hit = 0.5*(t_old+t_current);
			}
			hit_pos[0]=ray_orig[0]+t_hit*ray_dir[0];
			hit_pos[1]=ray_orig[1]+t_hit*ray_dir[1];
			hit_pos[2]=ray_orig[2]+t_hit*ray_dir[2];
			aspheric_surface_normal(hit_pos[0], hit_pos[1], hit_normal, properties);
			return true;
		}
	}

	//If it reaches this far, the ray has marched through the bounding box and hit nothing
	return false;

}


void set_axiasphere_bounds(
	axiasphere *properties
	){

	double srad_squared = properties[0].surface_radius*properties[0].surface_radius;
	double srad_current = sqrt(srad_squared);
	double principal = srad_squared / (1.0 + sqrt(1.0 - (1.0+properties[0].conic_constant)*(srad_squared) / (properties[0].curvature_radius*properties[0].curvature_radius))) / properties[0].curvature_radius;
	double surface_max = std::max(principal, 0.0);
	double surface_min = std::min(principal, 0.0);

	for(int i=0; i<properties[0].num_aspheric_coefficients; ++i){
		if(properties[0].aspheric_coefficients[i]>0.0){
			surface_max += pow(srad_current,properties[0].aspheric_powers[i])*properties[0].aspheric_coefficients[i];
		}
		else{
			surface_min += pow(srad_current,properties[0].aspheric_powers[i])*properties[0].aspheric_coefficients[i];	
		}
	}

	properties[0].bound_min_z = surface_min-MIN_HIT_DISTANCE;
	properties[0].bound_max_z = surface_max+MIN_HIT_DISTANCE;
	properties[0].bound_radius = properties[0].surface_radius+MIN_HIT_DISTANCE;

}





