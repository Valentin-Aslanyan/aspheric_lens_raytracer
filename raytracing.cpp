/*
Functions to carry out basic raytracing, including some simple primitives
These are deliberately low-level, so use them in low-level high performance code, or simplify them with abstract 
data types as preferred 

Rays are defined as ray_orig (or p) + t*ray_dir (or t*q) where t is the distance
DO NOT assume q or surface normals are normalized unless stated otherwise

There is a custom struct containing the basic properties of every primitive

All "intersect" functions check if there is at least satisfactory hit between ray and primitive
Returns true if at least one, otherwise false
Satisfactory hit means that t strictly > 0
Returns smallest satisfactory t, the corresponding hit location and corresponding normal of primitive at that point
Note, all values returned by reference are only valid if actual return value is true

The normals will point "out" from the primitive (e.g. out from center of circle)
use the normal_mulitplier variable as follows:
	+1.0 - material is "inside" primitive e.g. convex lens
	 0.0 - optic is absorbing; no reflection or refraction
	-1.0 - material is "outside" primitive e.g. concave lens

The prefix "axi-", such as "axisphere" implies that the object is symmetric about the optical (z) axis
*/


#include "raytracing.h"


//Sphere (spherical cap) centered on optical (z-) axis; cutoff above a certain radius from the axis and by z_min,z_max bounds
static bool rayintersect_axisphere(
	double *ray_orig,
	double *ray_dir,	
	double &t_hit,
	double *hit_pos,
	double *hit_normal,
	const axisphere properties
	) {

	double p_x_shift=ray_orig[0];
	double p_y_shift=ray_orig[1];
	double p_z_shift=ray_orig[2]-properties.center_z;
	double a = ray_dir[0]*ray_dir[0] + ray_dir[1]*ray_dir[1] + ray_dir[2]*ray_dir[2];
	double b = 2.0*(p_x_shift*ray_dir[0]+p_y_shift*ray_dir[1]+p_z_shift*ray_dir[2]);
	double c = p_x_shift*p_x_shift + p_y_shift*p_y_shift + p_z_shift*p_z_shift - properties.radius*properties.radius;
	double discriminant = b*b-4.0*a*c;
	double t_temp1, t_temp2;
	double hit_temp1[3], hit_temp2[3];

	if(discriminant<0.0){
		return false;
	}
	else if(discriminant==0.0){	//Ray hits sphere in just 1 location, at a tangent
		t_temp1 = -0.5*b/a;
		if(t_temp1>0.00001){
			hit_temp1[0]=ray_orig[0]+t_temp1*ray_dir[0];
			hit_temp1[1]=ray_orig[1]+t_temp1*ray_dir[1];
			hit_temp1[2]=ray_orig[2]+t_temp1*ray_dir[2];
			if(	(hit_temp1[2] < properties.z_min) || (hit_temp1[2] > properties.z_max) || 
				(hit_temp1[0]*hit_temp1[0] + hit_temp1[1]*hit_temp1[1] > properties.enclosing_cylinder_radius*properties.enclosing_cylinder_radius)){	//Check if inside cylinder and z bounds
				return false;
			}
		}
		else{
			return false;
		}
	}
	else{
		t_temp1=0.5/a*(sqrt(discriminant)-b);
		t_temp2=-0.5/a*(sqrt(discriminant)+b);
		hit_temp1[0]=ray_orig[0]+t_temp1*ray_dir[0];
		hit_temp1[1]=ray_orig[1]+t_temp1*ray_dir[1];
		hit_temp1[2]=ray_orig[2]+t_temp1*ray_dir[2];
		hit_temp2[0]=ray_orig[0]+t_temp2*ray_dir[0];
		hit_temp2[1]=ray_orig[1]+t_temp2*ray_dir[1];
		hit_temp2[2]=ray_orig[2]+t_temp2*ray_dir[2];
		if(	(t_temp1>0.00001) && (hit_temp1[2] >= properties.z_min) && (hit_temp1[2] <= properties.z_max) &&
			(hit_temp1[0]*hit_temp1[0] + hit_temp1[1]*hit_temp1[1] <= properties.enclosing_cylinder_radius*properties.enclosing_cylinder_radius)){
			if(	(t_temp2>0.00001) && (t_temp2<t_temp1) && (hit_temp2[2] >= properties.z_min) && (hit_temp2[2] <= properties.z_max) &&
				(hit_temp2[0]*hit_temp2[0] + hit_temp2[1]*hit_temp2[1] <= properties.enclosing_cylinder_radius*properties.enclosing_cylinder_radius)){
				hit_temp1[0] = hit_temp2[0];
				hit_temp1[1] = hit_temp2[1];
				hit_temp1[2] = hit_temp2[2];
				t_temp1 = t_temp2;
			}
		}
		else{
			if(	(t_temp2>0.00001) && (t_temp2<t_temp1) && (hit_temp2[2] >= properties.z_min) && (hit_temp2[2] <= properties.z_max) &&
				(hit_temp2[0]*hit_temp2[0] + hit_temp2[1]*hit_temp2[1] <= properties.enclosing_cylinder_radius*properties.enclosing_cylinder_radius)){
				hit_temp1[0] = hit_temp2[0];
				hit_temp1[1] = hit_temp2[1];
				hit_temp1[2] = hit_temp2[2];
				t_temp1 = t_temp2;
			}
			else{
				return false;
			}
		}
	}

	//If it hasn't returned false by this point, t_hit has been set and hit_temp is correct
	t_hit = t_temp1; 
	hit_pos[0] = hit_temp1[0];
	hit_pos[1] = hit_temp1[1];
	hit_pos[2] = hit_temp1[2];
	hit_normal[0] = hit_temp1[0]*properties.normal_multiplier;
	hit_normal[1] = hit_temp1[1]*properties.normal_multiplier;
	hit_normal[2] = (hit_temp1[2]-properties.center_z)*properties.normal_multiplier;
	return true;

}


//Cylinder of radius r on and aligned with optical (z-) axis; from z_min to z_max
static bool rayintersect_axicylinder(
	double *ray_orig,
	double *ray_dir,	
	double &t_hit,
	double *hit_pos,
	double *hit_normal,
	const axicylinder properties
	) {

	double a = ray_dir[0]*ray_dir[0] + ray_dir[1]*ray_dir[1];
	double b = 2.0*(ray_orig[0]*ray_dir[0]+ray_orig[1]*ray_dir[1]);
	double c = ray_orig[0]*ray_orig[0] + ray_orig[1]*ray_orig[1] - properties.radius*properties.radius;
	double discriminant = b*b-4.0*a*c;
	double t_temp1, t_temp2;
	double hit_temp1[3], hit_temp2[3];

	if((discriminant<0.0) || (a==0.0)){
		return false;
	}
	else if(discriminant==0.0){	//Ray hits cylinder in just 1 location, at a tangent
		t_temp1 = -0.5*b/a;
		if(t_temp1>0.00001){
			hit_temp1[0]=ray_orig[0]+t_temp1*ray_dir[0];
			hit_temp1[1]=ray_orig[1]+t_temp1*ray_dir[1];
			hit_temp1[2]=ray_orig[2]+t_temp1*ray_dir[2];
			if(hit_temp1[2]<properties.z_min || hit_temp1[2]>properties.z_max){	//Check if within length of cylinder
				return false;
			}
		}
		else{
			return false;
		}
	}
	else{
		t_temp1=0.5/a*(sqrt(discriminant)-b);
		t_temp2=-0.5/a*(sqrt(discriminant)+b);
		hit_temp1[0]=ray_orig[0]+t_temp1*ray_dir[0];
		hit_temp1[1]=ray_orig[1]+t_temp1*ray_dir[1];
		hit_temp1[2]=ray_orig[2]+t_temp1*ray_dir[2];
		hit_temp2[0]=ray_orig[0]+t_temp2*ray_dir[0];
		hit_temp2[1]=ray_orig[1]+t_temp2*ray_dir[1];
		hit_temp2[2]=ray_orig[2]+t_temp2*ray_dir[2];
		if((t_temp1>0.00001) && (hit_temp1[2]>=properties.z_min && hit_temp1[2]<=properties.z_max)){
			if((t_temp2>0.00001) && (t_temp2<t_temp1) && (hit_temp2[2]>=properties.z_min && hit_temp2[2]<=properties.z_max)){
				hit_temp1[0] = hit_temp2[0];
				hit_temp1[1] = hit_temp2[1];
				hit_temp1[2] = hit_temp2[2];
				t_temp1 = t_temp2;
			}
		}
		else{
			if((t_temp2>0.00001) && (hit_temp2[2]>=properties.z_min && hit_temp2[2]<=properties.z_max)){
				hit_temp1[0] = hit_temp2[0];
				hit_temp1[1] = hit_temp2[1];
				hit_temp1[2] = hit_temp2[2];
				t_temp1 = t_temp2;
			}
			else{
				return false;
			}
		}
	}

	//If it hasn't returned false by this point, t_hit has been set and hit_temp is correct
	t_hit = t_temp1; 
	hit_pos[0] = hit_temp1[0];
	hit_pos[1] = hit_temp1[1];
	hit_pos[2] = hit_temp1[2];
	hit_normal[0] = hit_temp1[0]*properties.normal_multiplier;
	hit_normal[1] = hit_temp1[1]*properties.normal_multiplier;
	hit_normal[2] = 0.0;
	return true;

}


//Plane is defined by NORMALIZED perpendicular vector u and vector to point on plane v
static bool rayintersect_plane(
	double *ray_orig,
	double *ray_dir,	
	double &t_hit,
	double *hit_pos,
	double *hit_normal,
	const plane properties
	) {

	double t_temp;
	double u_dot_dir = properties.u_x*ray_dir[0] + properties.u_y*ray_dir[1] + properties.u_z*ray_dir[2];
	if(u_dot_dir==0.0){
		return false;
	}
	t_temp = (properties.u_x*(properties.v_x-ray_orig[0])+properties.u_y*(properties.v_y-ray_orig[1])+properties.u_z*(properties.v_z-ray_orig[2]))/u_dot_dir;
	if(t_temp>0.00001){
		t_hit = t_temp;
		hit_pos[0]=ray_orig[0]+t_temp*ray_dir[0];
		hit_pos[1]=ray_orig[1]+t_temp*ray_dir[1];
		hit_pos[2]=ray_orig[2]+t_temp*ray_dir[2];
		hit_normal[0] = properties.u_x*properties.normal_multiplier;
		hit_normal[1] = properties.u_y*properties.normal_multiplier;
		hit_normal[2] = properties.u_z*properties.normal_multiplier;
		return true;
	}
	else{
		return false;
	}

}


//Axisymmetric flat circle/annulus from r_in to r_out; normal_mulitplier: +1 if facing increasing in Z, -1 if facing decreasing Z
static bool rayintersect_axicircle(
	double *ray_orig,
	double *ray_dir,	
	double &t_hit,
	double *hit_pos,
	double *hit_normal,
	const axicircle properties
	) {

	double t_temp;
	double hit_temp[3];
	if(ray_dir[2]==0.0){
		return false;
	}
	t_temp = (properties.z_pos-ray_orig[2])/ray_dir[2];
	hit_temp[0]=ray_orig[0]+t_temp*ray_dir[0];
	hit_temp[1]=ray_orig[1]+t_temp*ray_dir[1];
	hit_temp[2]=ray_orig[2]+t_temp*ray_dir[2];
	if((t_temp>0.00001) 	&& (hit_temp[0]*hit_temp[0] + hit_temp[1]*hit_temp[1] >= properties.r_in*properties.r_in)
					&& (hit_temp[0]*hit_temp[0] + hit_temp[1]*hit_temp[1] <= properties.r_out*properties.r_out)){
		t_hit = t_temp;
		hit_pos[0] = hit_temp[0];
		hit_pos[1] = hit_temp[1];
		hit_pos[2] = hit_temp[2];
		hit_normal[0] = 0.0;
		hit_normal[1] = 0.0;
		hit_normal[2] = properties.normal_multiplier;
		return true;
	}
	else{
		return false;
	}

}


static inline void process_hit_refractive(
	bool &hit_found,
	double &t_min,
	double *hit_min,
	double *normal_min,
	double &n_1_min,
	double &n_2_min,
	const double t_curr,
	double *hit_curr,
	double *normal_curr,
	const double n_1_curr,
	const double n_2_curr
	) {

	if(hit_found){
		if(t_min>t_curr){
			t_min = t_curr;
			hit_min[0] = hit_curr[0];
			hit_min[1] = hit_curr[1];
			hit_min[2] = hit_curr[2];
			normal_min[0] = normal_curr[0];
			normal_min[1] = normal_curr[1];
			normal_min[2] = normal_curr[2];
			n_1_min = n_1_curr;
			n_2_min = n_2_curr;
		}
	}
	else{
		t_min = t_curr;
		hit_min[0] = hit_curr[0];
		hit_min[1] = hit_curr[1];
		hit_min[2] = hit_curr[2];
		normal_min[0] = normal_curr[0];
		normal_min[1] = normal_curr[1];
		normal_min[2] = normal_curr[2];
		n_1_min = n_1_curr;
		n_2_min = n_2_curr;
		hit_found = true;
	}
	return;
}


static inline bool process_hit_absorbing(
	bool &hit_found,
	bool &ray_alive,
	double &t_min,
	double *hit_min,
	double *normal_min,
	const double t_curr,
	double *hit_curr,
	double *normal_curr
	) {

	if(hit_found){
		if(t_min>t_curr){
			t_min = t_curr;
			hit_min[0] = hit_curr[0];
			hit_min[1] = hit_curr[1];
			hit_min[2] = hit_curr[2];
			normal_min[0] = normal_curr[0];
			normal_min[1] = normal_curr[1];
			normal_min[2] = normal_curr[2];
			ray_alive = false;
			return true;
		}
	}
	else{
		t_min = t_curr;
		hit_min[0] = hit_curr[0];
		hit_min[1] = hit_curr[1];
		hit_min[2] = hit_curr[2];
		normal_min[0] = normal_curr[0];
		normal_min[1] = normal_curr[1];
		normal_min[2] = normal_curr[2];
		ray_alive = false;
		hit_found = true;
		return true;
	}

	return false;

}


//boundingbox_details contains: [min_X,max_X,min_Y,max_Y,min_Z,max_Z]
//Must ensure every min strictly <= max
//Find t-values where ray intersects each bounding plane, e.g. t_min_x is intersection with plane at min_X
//Do z-axis first, most likely to differentiate based on optical axis
static bool check_bounding_box(
	double *&ray_orig,
	double *&ray_dir,
	const double *boundingbox_details
	){

	double t_min_x, t_min_y, t_min_z, t_max_x, t_max_y, t_max_z;

	if(ray_dir[2]==0.0){
		if((ray_orig[2] < boundingbox_details[4]) || (ray_orig[2] > boundingbox_details[5])){
			return false;
		}
		t_min_z = -7E10;
		t_max_z = 7E10;
	}
	else if(ray_dir[2]>0.0){
		t_min_z = (boundingbox_details[4]-ray_orig[2])/ray_dir[2];
		t_max_z = (boundingbox_details[5]-ray_orig[2])/ray_dir[2];
	}
	else{
		t_min_z = (boundingbox_details[5]-ray_orig[2])/ray_dir[2];
		t_max_z = (boundingbox_details[4]-ray_orig[2])/ray_dir[2];
	}
	if(t_max_z<0.0){
		return false;
	}

	if(ray_dir[1]==0.0){
		if((ray_orig[1] < boundingbox_details[2]) || (ray_orig[1] > boundingbox_details[3])){
			return false;
		}
		t_min_y = -8E10;
		t_max_y = 8E10;
	}
	else if(ray_dir[1]>0.0){
		t_min_y = (boundingbox_details[2]-ray_orig[1])/ray_dir[1];
		t_max_y = (boundingbox_details[3]-ray_orig[1])/ray_dir[1];
	}
	else{
		t_min_y = (boundingbox_details[3]-ray_orig[1])/ray_dir[1];
		t_max_y = (boundingbox_details[2]-ray_orig[1])/ray_dir[1];
	}
	if((t_max_y<0.0) || (t_min_z>t_max_y) || (t_min_y>t_max_z)){
		return false;
	}

	if(ray_dir[0]==0.0){
		if((ray_orig[0] < boundingbox_details[0]) || (ray_orig[0] > boundingbox_details[1])){
			return false;
		}
		t_min_x = -9E10;
		t_max_x = 9E10;
	}
	else if(ray_dir[0]>0.0){
		t_min_x = (boundingbox_details[0]-ray_orig[0])/ray_dir[0];
		t_max_x = (boundingbox_details[1]-ray_orig[0])/ray_dir[0];
	}
	else{
		t_min_x = (boundingbox_details[1]-ray_orig[0])/ray_dir[0];
		t_max_x = (boundingbox_details[0]-ray_orig[0])/ray_dir[0];
	}
	if((t_max_x<0.0) || (t_min_y>t_max_x) || (t_min_x>t_max_y) || (t_min_z>t_max_x) || (t_min_x>t_max_z)){
		return false;
	}

	return true;

}


//Loop over all optical primitives by type
static bool ray_intersect_all_primitives(
	double *ray_orig,
	double *ray_dir,
	bool &ray_alive,
	double &t_hit,
	double *hit_pos,
	double *hit_normal, 
	double &hit_n_1,
	double &hit_n_2,
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
	) {

	bool hit_found = false;
	double t_min;
	double hit_min[3], normal_min[3];
	double n_1_min = 1.0, n_2_min = 1.0;
	double t_curr;
	double hit_curr[3], normal_curr[3];
	ray_alive = true;

	//Loop over axisymmetric spheres
	for(int i = 0; i<num_axispheres; ++i){
		if(
			(axisphere_properties[i].bbox_active && 
			check_bounding_box(ray_orig, ray_dir, axisphere_properties[i].boundingbox_details)) ||
			!axisphere_properties[i].bbox_active
		){
			if(rayintersect_axisphere(ray_orig, ray_dir, t_curr, hit_curr, normal_curr, axisphere_properties[i])) {
				process_hit_refractive(hit_found, t_min, hit_min, normal_min, n_1_min, n_2_min, t_curr, hit_curr, normal_curr, axisphere_properties[i].n_1, axisphere_properties[i].n_2);
			}
		}
	}

	//Loop over axial cylinders
	for(int i = 0; i<num_axicylinders; ++i){
		if(
			(axicylinder_properties[i].bbox_active && 
			check_bounding_box(ray_orig, ray_dir, axicylinder_properties[i].boundingbox_details)) ||
			!axicylinder_properties[i].bbox_active
		){		
			if(rayintersect_axicylinder(ray_orig, ray_dir, t_curr, hit_curr, normal_curr, axicylinder_properties[i])) {
				process_hit_refractive(hit_found, t_min, hit_min, normal_min, n_1_min, n_2_min, t_curr, hit_curr, normal_curr, axicylinder_properties[i].n_1, axicylinder_properties[i].n_2);
			}			
		}
	}

	//Loop over axicircles
	for(int i = 0; i<num_axicircles; ++i){
		if(
			(axicircle_properties[i].bbox_active && 
			check_bounding_box(ray_orig, ray_dir, axicircle_properties[i].boundingbox_details)) || 
			!axicircle_properties[i].bbox_active
		){
			if(rayintersect_axicircle(ray_orig, ray_dir, t_curr, hit_curr, normal_curr, axicircle_properties[i])) {
				process_hit_refractive(hit_found, t_min, hit_min, normal_min, n_1_min, n_2_min, t_curr, hit_curr, normal_curr, axicircle_properties[i].n_1, axicircle_properties[i].n_2);
			}
		}
	}

	//Loop over axiaspheres - these have their own bounding box
	for(int i = 0; i<num_axiaspheres; ++i){
		if(rayintersect_axiasphere(ray_orig, ray_dir, t_curr, hit_curr, normal_curr, axiasphere_properties[i])) {
			process_hit_refractive(hit_found, t_min, hit_min, normal_min, n_1_min, n_2_min, t_curr, hit_curr, normal_curr, axiasphere_properties[i].n_1, axiasphere_properties[i].n_2);
			//process_hit_absorbing(hit_found, ray_alive, t_min, hit_min, normal_min, t_curr, hit_curr, normal_curr);
		}

	}

	//Loop over planes - assume that these are the absorbing walls in this case
	for(int i = 0; i<num_planes; ++i){
		if(
			(plane_properties[i].bbox_active && 
			check_bounding_box(ray_orig, ray_dir, plane_properties[i].boundingbox_details)) || 
			!plane_properties[i].bbox_active
		){
			if(rayintersect_plane(ray_orig, ray_dir, t_curr, hit_curr, normal_curr, plane_properties[i])) {
				process_hit_absorbing(hit_found, ray_alive, t_min, hit_min, normal_min, t_curr, hit_curr, normal_curr);
			}
		}
	}


	t_hit = t_min;
	hit_pos[0] = hit_min[0];
	hit_pos[1] = hit_min[1];
	hit_pos[2] = hit_min[2];
	hit_normal[0] = normal_min[0];
	hit_normal[1] = normal_min[1];
	hit_normal[2] = normal_min[2];
	hit_n_1 = n_1_min;
	hit_n_2 = n_2_min;
	return hit_found;

}


//n_1 must be the refractive index "outside", namely in the direction the normal points
//n_2 the refractive index inside
static void resolve_refraction(
	double *ray_dir,
	double *optic_normal,
	double n_1,
	double n_2,
	bool &TIR,
	double *refracted_dir,
	double *reflected_dir,
	double &reflected_fraction
	){

	double n_ratio;
	double i_norm[3], n_norm[3];
	double norm_factor1 = sqrt(ray_dir[0]*ray_dir[0] + ray_dir[1]*ray_dir[1] + ray_dir[2]*ray_dir[2]);
	i_norm[0] = ray_dir[0]/norm_factor1;
	i_norm[1] = ray_dir[1]/norm_factor1;
	i_norm[2] = ray_dir[2]/norm_factor1;
	double norm_factor2 = sqrt(optic_normal[0]*optic_normal[0] + optic_normal[1]*optic_normal[1] + optic_normal[2]*optic_normal[2]);
	n_norm[0] = optic_normal[0]/norm_factor2;
	n_norm[1] = optic_normal[1]/norm_factor2;
	n_norm[2] = optic_normal[2]/norm_factor2;
	double cos_theta_i=i_norm[0]*n_norm[0] + i_norm[1]*n_norm[1] + i_norm[2]*n_norm[2];

	reflected_dir[0] = i_norm[0]-2.0*cos_theta_i*n_norm[0];
	reflected_dir[1] = i_norm[1]-2.0*cos_theta_i*n_norm[1];
	reflected_dir[2] = i_norm[2]-2.0*cos_theta_i*n_norm[2];
	//n_ratio = n_1/n_2;
	if(cos_theta_i>0.0){
		n_ratio = n_2/n_1;
		n_norm[0] = -1.0*n_norm[0];
		n_norm[1] = -1.0*n_norm[1];
		n_norm[2] = -1.0*n_norm[2];
	}
	else{
		n_ratio = n_1/n_2;

	}
	cos_theta_i = abs(cos_theta_i);

	double cos_theta_t, temp1, temp2, temp3, temp4;
	double sin2_theta_t = n_ratio*n_ratio*(1.0-cos_theta_i*cos_theta_i);
	if(sin2_theta_t > 1.0){	//Total internal reflection condition
		TIR = true;
		reflected_fraction = 1.0;
	}
	else{				//Ordinary refraction + reflection
		TIR = false;
		cos_theta_t = sqrt(1.0-sin2_theta_t);
		temp1 = n_1*cos_theta_i-n_2*cos_theta_t;
		temp2 = n_1*cos_theta_i+n_2*cos_theta_t;
		temp3 = n_2*cos_theta_i-n_1*cos_theta_t;
		temp4 = n_2*cos_theta_i+n_1*cos_theta_t;
		reflected_fraction = 0.5*(temp1*temp1)/(temp2*temp2) + 0.5*(temp3*temp3)/(temp4*temp4);
		refracted_dir[0] = n_ratio*i_norm[0] + (n_ratio*cos_theta_i - sqrt(1.0 - sin2_theta_t))*n_norm[0];
		refracted_dir[1] = n_ratio*i_norm[1] + (n_ratio*cos_theta_i - sqrt(1.0 - sin2_theta_t))*n_norm[1];
		refracted_dir[2] = n_ratio*i_norm[2] + (n_ratio*cos_theta_i - sqrt(1.0 - sin2_theta_t))*n_norm[2];
	}

}


void setup_ray_trace(
	bool *&ray_running,
	bool *&ray_alive,
	double *&intensity, 
	int *&num_reflections,
	double *&out_orig,
	double *&out_dir
	) {

	ray_running = (bool*)malloc((OPTICS_MAX_CHILD_RAYS+1)*sizeof(bool));
	ray_alive = (bool*)malloc((OPTICS_MAX_CHILD_RAYS+1)*sizeof(bool));
	intensity = (double*)malloc((OPTICS_MAX_CHILD_RAYS+1)*sizeof(double));
	num_reflections = (int*)malloc((OPTICS_MAX_CHILD_RAYS+1)*sizeof(int));
	out_orig = (double*)malloc(3*(OPTICS_MAX_CHILD_RAYS+1)*sizeof(double));
	out_dir = (double*)malloc(3*(OPTICS_MAX_CHILD_RAYS+1)*sizeof(double));

}


void cleanup_ray_trace(
	bool *&ray_running,
	bool *&ray_alive,
	double *&intensity, 
	int *&num_reflections,
	double *&out_orig,
	double *&out_dir
	) {

	free(ray_running);
	free(ray_alive);
	free(intensity);
	free(num_reflections);
	free(out_orig);
	free(out_dir);

}


//Trace the ray through the optic, spawning reflected child rays
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
	){

	int active_rays = 1;
	int current_rays;
	bool keep_running = true; //Do this while some rays have hit surfaces that need to be resolved

	bool temp_alive, temp_TIR;
	double temp_t, temp_frac, n_1, n_2;
	double temp_pos[3], temp_normal[3], temp_refra[3], temp_refle[3];

	ray_running[0] = true;
	ray_alive[0] = true;
	intensity[0] = 1.0;
	num_reflections[0] = 0;
	out_orig[0] = ray_orig[0];
	out_orig[1] = ray_orig[1];
	out_orig[2] = ray_orig[2];
	out_dir[0] = ray_dir[0];
	out_dir[1] = ray_dir[1];
	out_dir[2] = ray_dir[2];
	for(int idx = 1; idx<OPTICS_MAX_CHILD_RAYS+1; ++idx){
		ray_running[idx] = false;
		ray_alive[idx] = false;
		num_reflections[idx] = 0;
		intensity[idx] = 0.0;
	}

	int iteration = 0;
	while(keep_running && iteration < 200){
		keep_running = false;
		++iteration;
		current_rays = active_rays;
		for (int idx_r = 0; idx_r<current_rays; ++idx_r){
			if(ray_running[idx_r]){
				keep_running = true;
				if(ray_intersect_all_primitives(&out_orig[3*idx_r], &out_dir[3*idx_r], temp_alive, temp_t, temp_pos, temp_normal, n_1, n_2, num_axispheres, axisphere_properties, num_axicylinders, axicylinder_properties, num_planes, plane_properties, num_axicircles, axicircle_properties, num_axiaspheres, axiasphere_properties)){
					if(temp_alive){
						resolve_refraction(&out_dir[3*idx_r], temp_normal, n_1, n_2, temp_TIR, temp_refra, temp_refle, temp_frac);
						if(temp_TIR){	//Only reflection
							intensity[idx_r] = temp_frac*intensity[idx_r];
							out_orig[3*idx_r] = temp_pos[0];
							out_orig[3*idx_r+1] = temp_pos[1];
							out_orig[3*idx_r+2] = temp_pos[2];
							out_dir[3*idx_r] = temp_refle[0];
							out_dir[3*idx_r+1] = temp_refle[1];
							out_dir[3*idx_r+2] = temp_refle[2];
							//Increment number of reflections here?
						}
						else{		//Reflection and refraction have occurred
							if(active_rays<=OPTICS_MAX_CHILD_RAYS){	//Spawn reflected ray
								ray_running[active_rays]=true;
								ray_alive[active_rays]=true;
								intensity[active_rays] = temp_frac*intensity[idx_r];
								num_reflections[active_rays] = num_reflections[idx_r]+1;
								out_orig[3*active_rays] = temp_pos[0];
								out_orig[3*active_rays+1] = temp_pos[1];
								out_orig[3*active_rays+2] = temp_pos[2];
								out_dir[3*active_rays] = temp_refle[0];
								out_dir[3*active_rays+1] = temp_refle[1];
								out_dir[3*active_rays+2] = temp_refle[2];
								++active_rays;
							}
							//Process refracted (original) ray
							intensity[idx_r] = (1.0-temp_frac)*intensity[idx_r];
							out_orig[3*idx_r] = temp_pos[0];
							out_orig[3*idx_r+1] = temp_pos[1];
							out_orig[3*idx_r+2] = temp_pos[2];
							out_dir[3*idx_r] = temp_refra[0];
							out_dir[3*idx_r+1] = temp_refra[1];
							out_dir[3*idx_r+2] = temp_refra[2];
						}
					}
					else{
						out_orig[3*idx_r] = temp_pos[0];
						out_orig[3*idx_r+1] = temp_pos[1];
						out_orig[3*idx_r+2] = temp_pos[2];
						ray_alive[idx_r] = false;
						ray_running[idx_r] = false;
					}
				}
				else{
					ray_running[idx_r] = false;
				}
			}
		}
	}

	num_rays = active_rays;

}


void process_ray_end(
	double &R_intensity,
	double &G_intensity,
	double &B_intensity,
	bool *ray_alive,
	double *intensity,
	int *num_reflections, 
	double *&out_orig,
	double *&out_dir
	){

	for(int idx_r = 0; idx_r < OPTICS_MAX_CHILD_RAYS+1; ++idx_r){
		if(std::abs(out_orig[3*idx_r+2]-backplane_Z)<0.01){ //Back plane
			if(	((std::fmod(out_orig[3*idx_r]+sideplanes_X,2.0) <1.0) && (std::fmod(out_orig[3*idx_r+1]+verticalplanes_Y,2.0) >1.0)) ||
				((std::fmod(out_orig[3*idx_r]+sideplanes_X,2.0) >1.0) && (std::fmod(out_orig[3*idx_r+1]+verticalplanes_Y,2.0) <1.0))){
				G_intensity += intensity[idx_r];
			}
		}
		else if(std::abs(out_orig[3*idx_r+2]-frontplane_Z)<0.01){ //Front plane
			if(	((std::fmod(out_orig[3*idx_r]+sideplanes_X,2.0) <1.0) && (std::fmod(out_orig[3*idx_r+1]+verticalplanes_Y,2.0) >1.0)) ||
				((std::fmod(out_orig[3*idx_r]+sideplanes_X,2.0) >1.0) && (std::fmod(out_orig[3*idx_r+1]+verticalplanes_Y,2.0) <1.0))){
				R_intensity += intensity[idx_r];
				G_intensity += intensity[idx_r];
				B_intensity += intensity[idx_r];
			}
		}
		else if(std::abs(out_orig[3*idx_r]+sideplanes_X)<0.01){ //Left plane
			if(	((std::fmod(out_orig[3*idx_r+2]-backplane_Z,2.0) <1.0) && (std::fmod(out_orig[3*idx_r+1]+verticalplanes_Y,2.0) >1.0)) ||
				((std::fmod(out_orig[3*idx_r+2]-backplane_Z,2.0) >1.0) && (std::fmod(out_orig[3*idx_r+1]+verticalplanes_Y,2.0) <1.0))){
				B_intensity += intensity[idx_r];
			}	
		}
		else if(std::abs(out_orig[3*idx_r]-sideplanes_X)<0.01){ //Right plane
				B_intensity += intensity[idx_r];
		}
		else{
			G_intensity += intensity[idx_r];
			B_intensity += intensity[idx_r];
		}
	}

}


void asphere_example1(
	axiasphere &properties
	){

	properties.conic_constant = -2.1201;
	properties.curvature_radius = 1.952;
	properties.surface_radius = 0.7347;
	properties.z_offset = 0.0;
	properties.n_1 = 1.0;
	properties.n_2 = 1.5;
	properties.normal_multiplier = 1.0;
	properties.fractional_step_size = DEFAULT_STEP_SIZE;
	properties.num_aspheric_coefficients = 6;
	properties.aspheric_coefficients = (double*)malloc(properties.num_aspheric_coefficients*sizeof(double));
	properties.aspheric_coefficients[0] = 3.5988E-03;
	properties.aspheric_coefficients[1] = 3.7387E-01;
	properties.aspheric_coefficients[2] = -1.3929E+00;
	properties.aspheric_coefficients[3] = 1.4094E+00;
	properties.aspheric_coefficients[4] = 2.0282E+00;
	properties.aspheric_coefficients[5] = -3.6199E+00;
	properties.aspheric_powers = (int*)malloc(properties.num_aspheric_coefficients*sizeof(int));
	properties.aspheric_powers[0] = 4;
	properties.aspheric_powers[1] = 6;
	properties.aspheric_powers[2] = 8;
	properties.aspheric_powers[3] = 10;
	properties.aspheric_powers[4] = 12;
	properties.aspheric_powers[5] = 14;
	
	set_axiasphere_bounds(&properties);

}


void asphere_example2(
	axiasphere **properties_a,
	axicylinder **properties_y,
	axicircle **properties_i
	){

	properties_a[0] = (axiasphere*)malloc(2*sizeof(axiasphere));

	properties_a[0][0].conic_constant = -2.1201;
	properties_a[0][0].curvature_radius = 1.952;
	properties_a[0][0].surface_radius = 0.7347;
	properties_a[0][0].z_offset = 0.0;
	properties_a[0][0].n_1 = 1.0;
	properties_a[0][0].n_2 = 1.5;
	properties_a[0][0].normal_multiplier = 1.0;
	properties_a[0][0].fractional_step_size = DEFAULT_STEP_SIZE;
	properties_a[0][0].num_aspheric_coefficients = 6;
	properties_a[0][0].aspheric_coefficients = (double*)malloc(properties_a[0][0].num_aspheric_coefficients*sizeof(double));
	properties_a[0][0].aspheric_coefficients[0] = 3.5988E-03;
	properties_a[0][0].aspheric_coefficients[1] = 3.7387E-01;
	properties_a[0][0].aspheric_coefficients[2] = -1.3929E+00;
	properties_a[0][0].aspheric_coefficients[3] = 1.4094E+00;
	properties_a[0][0].aspheric_coefficients[4] = 2.0282E+00;
	properties_a[0][0].aspheric_coefficients[5] = -3.6199E+00;
	properties_a[0][0].aspheric_powers = (int*)malloc(properties_a[0][0].num_aspheric_coefficients*sizeof(int));
	properties_a[0][0].aspheric_powers[0] = 4;
	properties_a[0][0].aspheric_powers[1] = 6;
	properties_a[0][0].aspheric_powers[2] = 8;
	properties_a[0][0].aspheric_powers[3] = 10;
	properties_a[0][0].aspheric_powers[4] = 12;
	properties_a[0][0].aspheric_powers[5] = 14;
	
	set_axiasphere_bounds(&properties_a[0][0]);

	properties_a[0][1].conic_constant = -20.0;
	properties_a[0][1].curvature_radius = 2.978;
	properties_a[0][1].surface_radius = 0.8095;
	properties_a[0][1].z_offset = 0.069;
	properties_a[0][1].n_1 = 1.0;
	properties_a[0][1].n_2 = 1.5;
	properties_a[0][1].normal_multiplier = 1.0;
	properties_a[0][1].fractional_step_size = DEFAULT_STEP_SIZE;
	properties_a[0][1].num_aspheric_coefficients = 6;
	properties_a[0][1].aspheric_coefficients = (double*)malloc(properties_a[0][1].num_aspheric_coefficients*sizeof(double));
	properties_a[0][1].aspheric_coefficients[0] = -1.7492E-01;
	properties_a[0][1].aspheric_coefficients[1] = 9.8443E-02;
	properties_a[0][1].aspheric_coefficients[2] = 6.6244E-01;
	properties_a[0][1].aspheric_coefficients[3] = -1.7257E+00;
	properties_a[0][1].aspheric_coefficients[4] = 6.6955E-01;
	properties_a[0][1].aspheric_coefficients[5] = -3.4243E-01;
	properties_a[0][1].aspheric_powers = (int*)malloc(properties_a[0][1].num_aspheric_coefficients*sizeof(int));
	properties_a[0][1].aspheric_powers[0] = 4;
	properties_a[0][1].aspheric_powers[1] = 6;
	properties_a[0][1].aspheric_powers[2] = 8;
	properties_a[0][1].aspheric_powers[3] = 10;
	properties_a[0][1].aspheric_powers[4] = 12;
	properties_a[0][1].aspheric_powers[5] = 14;
	
	set_axiasphere_bounds(&properties_a[0][1]);

	properties_y[0] = (axicylinder*)malloc(sizeof(axicylinder));
	properties_y[0][0].radius = 0.8095;
	properties_y[0][0].z_min = aspheric_surface_height(properties_a[0][1].surface_radius, properties_a[0][1]);
	properties_y[0][0].z_max = aspheric_surface_height(properties_a[0][0].surface_radius, properties_a[0][0]);
	properties_y[0][0].n_1 = 1.0;
	properties_y[0][0].n_2 = 1.5;
	properties_y[0][0].normal_multiplier = 1.0;
	properties_y[0][0].bbox_active = false;

	properties_i[0] = (axicircle*)malloc(sizeof(axicircle));
	properties_i[0][0].r_in = 0.7347;
	properties_i[0][0].r_out = 0.8095;
	properties_i[0][0].z_pos = aspheric_surface_height(properties_a[0][0].surface_radius, properties_a[0][0]);
	properties_i[0][0].n_1 = 1.0;
	properties_i[0][0].n_2 = 1.5;
	properties_i[0][0].normal_multiplier = -1.0;
	properties_i[0][0].bbox_active = false;

}


void asphere_example3(
	axiasphere **properties_a,
	axicylinder **properties_y,
	axicircle **properties_i
	){

	double conic_constants[12] = {-2.1201, -20.0, -12.382, -3.006, 1.0, 0.0, -1.4213, -3.0569, -6.3368, -4.2091, -20.0, -5.1631};
	double curvature_radii[12] = {1.952, 2.978, 2.003, 2.167, 4.143, -3.947, -0.969, -2.089, 3.867, -0.847, -16.711, 0.785};
	double surface_radii[12] = {0.7347, 0.8095, 0.8163, 0.9320, 0.9660, 1.0748, 1.0952, 1.2925, 1.3741, 1.5986, 1.8435, 2.4218};
	double relative_offsets[12] = {0.398, 0.069, 0.280, 0.205, 0.551, 0.211, 0.421, 0.03, 1.105, 0.03, 0.440};
	double n_1_curr = 1.0, n_2_curr = 1.5;
	int aspheric_coefficients_curr = 6;
	int aspheric_powers_curr[6] = {4, 6, 8, 10, 12, 14};
	double aspheric_coefficients_all[72] = {	//Each set of 6 is for each successive surface
		3.5988E-03, 3.7387E-01, -1.3929E+00, 1.4094E+00, 2.0282E+00, -3.6199E+00,
		-1.7492E-01, 9.8443E-02, 6.6244E-01, -1.7257E+00, 6.6955E-01, -3.4243E-01,
		-2.6551E-01, 5.7642E-01, -1.1253E+00, 1.1555E+00, 1.4439E-02, -1.9141E+00,
		-2.5306E-01, 3.1573E-01, -2.1258E-01, 9.4103E-02, -2.7001E-01, 1.1110E-01,
		-1.4680E-01, -1.9199E-02, 1.1489E-01, -1.6910E-01, -1.1287E-01, 1.2615E-01,
		-4.4328E-02, -1.6599E-01, 4.8223E-02, 9.4653E-03, 1.2137E-02, -8.1558E-03,
		3.3847E-01, -8.7891E-01, 1.1445E+00, -9.1578E-01, 5.9482E-01, -2.0143E-01,
		1.4102E-01, -2.7755E-01, 3.2138E-01, -2.4968E-01, 1.2901E-01, -2.7976E-02,
		-1.6984E-01, 1.4944E-01, -9.9124E-02, 1.3353E-02, 1.6198E-03, 1.2524E-03,
		-6.7672E-02, -3.0645E-02, 1.0444E-01, -8.3869E-02, 2.6536E-02, -2.7869E-03,
		-1.3442E-01, 3.8933E-02, -6.2383E-03, 7.5356E-04, 2.2005E-04, -5.2957E-05,
		-7.5325E-02, 2.4095E-02, -5.8920E-03, 1.0269E-03, -1.1564E-04, 5.9417E-06};
	double offset_current = 0.0;

	properties_a[0] = (axiasphere*)malloc(12*sizeof(axiasphere));
	properties_y[0] = (axicylinder*)malloc(6*sizeof(axicylinder));
	properties_i[0] = (axicircle*)malloc(6*sizeof(axicircle));

	for(int i = 0; i<12; ++i){
		properties_a[0][i].conic_constant = conic_constants[i];
		properties_a[0][i].curvature_radius = curvature_radii[i];
		properties_a[0][i].surface_radius = surface_radii[i];
		properties_a[0][i].z_offset = offset_current;
		offset_current += relative_offsets[i];
		properties_a[0][i].n_1 = n_1_curr;
		properties_a[0][i].n_2 = n_2_curr;
		properties_a[0][i].normal_multiplier = 1.0;
		properties_a[0][i].fractional_step_size = DEFAULT_STEP_SIZE;
		properties_a[0][i].num_aspheric_coefficients = aspheric_coefficients_curr;
		properties_a[0][i].aspheric_coefficients = (double*)malloc(aspheric_coefficients_curr*sizeof(double));
		properties_a[0][i].aspheric_powers = (int*)malloc(aspheric_coefficients_curr*sizeof(int));
		for(int j = 0; j<aspheric_coefficients_curr; ++j){
			properties_a[0][i].aspheric_coefficients[j] = aspheric_coefficients_all[i*aspheric_coefficients_curr+j];
			properties_a[0][i].aspheric_powers[j] = aspheric_powers_curr[j];
		}
		set_axiasphere_bounds(&properties_a[0][i]);
		
		if(i%2 ==1){
			properties_a[0][i].normal_multiplier = 1.0;

			properties_y[0][(i-1)/2].radius = properties_a[0][i].surface_radius;
			properties_y[0][(i-1)/2].z_min = aspheric_surface_height(properties_a[0][i-1].surface_radius, properties_a[0][i-1]);
			properties_y[0][(i-1)/2].z_max = aspheric_surface_height(properties_a[0][i].surface_radius, properties_a[0][i]);
			properties_y[0][(i-1)/2].n_1 = n_1_curr;
			properties_y[0][(i-1)/2].n_2 = n_2_curr;
			properties_y[0][(i-1)/2].normal_multiplier = 1.0;
			properties_y[0][(i-1)/2].bbox_active = false;

			properties_i[0][(i-1)/2].r_in = properties_a[0][i-1].surface_radius;
			properties_i[0][(i-1)/2].r_out = properties_a[0][i].surface_radius;
			properties_i[0][(i-1)/2].z_pos = aspheric_surface_height(properties_a[0][i-1].surface_radius, properties_a[0][i-1]);
			properties_i[0][(i-1)/2].n_1 = n_1_curr;
			properties_i[0][(i-1)/2].n_2 = n_2_curr;
			properties_i[0][(i-1)/2].normal_multiplier = -1.0;
			properties_i[0][(i-1)/2].bbox_active = false;
		}

	}

}


void setup_walls(
	int &num_planes,
	plane *&plane_properties
	){

	num_planes = 6;
	plane_properties = (plane*)malloc(num_planes*sizeof(plane));

	//Front plane
	plane_properties[0].u_x = 0.0;
	plane_properties[0].u_y = 0.0;
	plane_properties[0].u_z = -1.0;
	plane_properties[0].v_x = 0.0;
	plane_properties[0].v_y = 0.0;
	plane_properties[0].v_z = frontplane_Z;
	plane_properties[0].normal_multiplier = 1.0;

	//Back plane
	plane_properties[1].u_x = 0.0;
	plane_properties[1].u_y = 0.0;
	plane_properties[1].u_z = 1.0;
	plane_properties[1].v_x = 0.0;
	plane_properties[1].v_y = 0.0;
	plane_properties[1].v_z = backplane_Z;
	plane_properties[1].normal_multiplier = 1.0;

	//Right plane
	plane_properties[2].u_x = -1.0;
	plane_properties[2].u_y = 0.0;
	plane_properties[2].u_z = 0.0;
	plane_properties[2].v_x = sideplanes_X;
	plane_properties[2].v_y = 0.0;
	plane_properties[2].v_z = 0.0;
	plane_properties[2].normal_multiplier = 1.0;

	//Left plane
	plane_properties[3].u_x = 1.0;
	plane_properties[3].u_y = 0.0;
	plane_properties[3].u_z = 0.0;
	plane_properties[3].v_x = -sideplanes_X;
	plane_properties[3].v_y = 0.0;
	plane_properties[3].v_z = 0.0;
	plane_properties[3].normal_multiplier = 1.0;

	//Up plane
	plane_properties[4].u_x = 0.0;
	plane_properties[4].u_y = -1.0;
	plane_properties[4].u_z = 0.0;
	plane_properties[4].v_x = 0.0;
	plane_properties[4].v_y = verticalplanes_Y;
	plane_properties[4].v_z = 0.0;
	plane_properties[4].normal_multiplier = 1.0;

	//Down plane
	plane_properties[5].u_x = 0.0;
	plane_properties[5].u_y = 1.0;
	plane_properties[5].u_z = 0.0;
	plane_properties[5].v_x = 0.0;
	plane_properties[5].v_y = -verticalplanes_Y;
	plane_properties[5].v_z = 0.0;
	plane_properties[5].normal_multiplier = 1.0;

}

void cleanup_walls(
	plane *&plane_properties
	) {

	free(plane_properties);

}


