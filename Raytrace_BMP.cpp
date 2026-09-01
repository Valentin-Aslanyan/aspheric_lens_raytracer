#include "raytracing.h"

#define IMAGE_WIDTH 1500
#define IMAGE_HEIGHT 1000
#define NUMBER_OF_SUPERSAMPLES 1


//Set these variables and recompile, or use the command line

//Optic types:
// 0: no optic
// 1: single glass sphere
// 2: single convex lens defined via two spheres and a cylinder
// 3: single aspherical surface
// 3: single aspherical lens defined via two aspherical surfaces, a plane and cylinder
// 4: multiple aspherical lenses
int optic_type=2;

//Angle in degrees about the y-axis
double view_angle=0.0;

//Size of horizontal viewport in mm
double view_size=10.0;


double *pixels_clean;
double camera_offset = 4.975;
double R_temp, G_temp, B_temp, t_temp, n1, n2, temp_x_pos, temp_y_pos, rot_angle;
bool alive_temp;
double ray_orig[3], ray_dir[3];
bool *ray_running, *ray_alive;
double *intensity;
int *num_reflections;
double *out_orig, *out_dir;
int num_rays;
axisphere single_sphere_properties;
axicylinder blank_cylinder_properties;
axicircle blank_axicircle_properties;
axiasphere blank_axiasphere_properties;
axisphere lens_sphere_properties[2];
axicylinder lens_cylinder_properties;
axiasphere example1_axiasphere_properties;
axiasphere *example2_axiasphere_properties;
axicylinder *example2_axicylinder_properties;
axicircle *example2_axicircle_properties;
axiasphere *example3_axiasphere_properties;
axicylinder *example3_axicylinder_properties;
axicircle *example3_axicircle_properties;
int num_walls;
plane *wall_properties;


FILE *bitmap_outfile;
unsigned char *m_image;
int bitmap_filesize;
unsigned char bitmap_header[14];
unsigned char bitmap_info[40];
unsigned char bitmap_pad[3];


void prepare_bitmap(
	int width,
	int height,
	int bitmap_filesize,
	unsigned char *bitmap_header,
	unsigned char *bitmap_info,
	unsigned char *bitmap_pad
	){

	memset(bitmap_header,0,14);
	memset(bitmap_info,0,40);
	memset(bitmap_pad,0,3);

	bitmap_header[ 0] = 'B';
	bitmap_header[ 1] = 'M';
	bitmap_header[ 2] = (unsigned char)(bitmap_filesize    );
	bitmap_header[ 3] = (unsigned char)(bitmap_filesize>> 8);
	bitmap_header[ 4] = (unsigned char)(bitmap_filesize>>16);
	bitmap_header[ 5] = (unsigned char)(bitmap_filesize>>24);
	bitmap_header[10] = 54;

	bitmap_info[ 0] = 40;
	bitmap_info[ 4] = (unsigned char)(   width    );
	bitmap_info[ 5] = (unsigned char)(   width>> 8);
	bitmap_info[ 6] = (unsigned char)(   width>>16);
	bitmap_info[ 7] = (unsigned char)(   width>>24);
	bitmap_info[ 8] = (unsigned char)(  height    );
	bitmap_info[ 9] = (unsigned char)(  height>> 8);
	bitmap_info[10] = (unsigned char)(  height>>16);
	bitmap_info[11] = (unsigned char)(  height>>24);
	bitmap_info[12] = 1;
	bitmap_info[14] = 24;

}


void write_bitmap(
	int width,
	int height,
	FILE *bitmap_outfile,
	unsigned char *m_image,
	unsigned char *bitmap_header,
	unsigned char *bitmap_info,
	unsigned char *bitmap_pad
	){

	bitmap_outfile = fopen("Raytrace_BMP.bmp","wb");
	fwrite(bitmap_header,1,14,bitmap_outfile);
	fwrite(bitmap_info,1,40,bitmap_outfile);
	for(int i=0; i<height; i++)
	{
	    fwrite(m_image+(width*(height-i-1)*3),3,width,bitmap_outfile);
	    fwrite(bitmap_pad,1,(4-(width*3)%4)%4,bitmap_outfile);
	}
	fclose(bitmap_outfile);

}


void cleanup_bitmap(
	unsigned char *m_image
	){

	free(m_image);

}


void initialize() {

	bitmap_filesize = 54 + 3*IMAGE_WIDTH*IMAGE_HEIGHT;
	m_image = (unsigned char *)malloc(3*IMAGE_WIDTH*IMAGE_HEIGHT*sizeof(unsigned char));
	memset(m_image,0,3*IMAGE_WIDTH*IMAGE_HEIGHT);
	prepare_bitmap(IMAGE_WIDTH, IMAGE_HEIGHT, bitmap_filesize, bitmap_header, bitmap_info, bitmap_pad);

	pixels_clean = (double*)malloc(sizeof(double)*3*IMAGE_HEIGHT*IMAGE_WIDTH);
	setup_walls(num_walls, wall_properties);
	setup_ray_trace(ray_running, ray_alive, intensity, num_reflections, out_orig, out_dir);

	//Setting up Optic Type 1
	single_sphere_properties.radius = 2.0;
	single_sphere_properties.center_z = 0.0;
	single_sphere_properties.enclosing_cylinder_radius = 2.1;
	single_sphere_properties.z_min = -2.1;
	single_sphere_properties.z_max = 2.1;
	single_sphere_properties.normal_multiplier = 1.0;
	single_sphere_properties.n_2 = 1.5;
	single_sphere_properties.bbox_active = true;
	single_sphere_properties.boundingbox_details[0] = -2.1;
	single_sphere_properties.boundingbox_details[1] = 2.1;
	single_sphere_properties.boundingbox_details[2] = -2.1;
	single_sphere_properties.boundingbox_details[3] = 2.1;
	single_sphere_properties.boundingbox_details[4] = -2.1;
	single_sphere_properties.boundingbox_details[5] = 2.1;

	//Setting up Optic Type 2
	lens_sphere_properties[0].radius = 10.5;
	lens_sphere_properties[0].center_z = 10.5;
	lens_sphere_properties[0].enclosing_cylinder_radius = 3.2;
	lens_sphere_properties[0].z_min = 0.0;
	lens_sphere_properties[0].z_max = 0.4995;
	lens_sphere_properties[0].normal_multiplier = 1.0;
	lens_sphere_properties[0].n_2 = 1.5;
	lens_sphere_properties[1].radius = 11.0;
	lens_sphere_properties[1].center_z = -9.0;
	lens_sphere_properties[1].enclosing_cylinder_radius = 3.2;
	lens_sphere_properties[1].z_min = 1.52426;
	lens_sphere_properties[1].z_max = 2.0;
	lens_sphere_properties[1].normal_multiplier = 1.0;
	lens_sphere_properties[1].n_2 = 1.5;
	lens_cylinder_properties.radius = 3.2;
	lens_cylinder_properties.z_min = 0.4995;
	lens_cylinder_properties.z_max = 1.52426;
	lens_cylinder_properties.normal_multiplier = 1.0;
	lens_cylinder_properties.n_2 = 1.5;

	//Setting up Optic Type 3
	asphere_example1(example1_axiasphere_properties);

	//Setting up Optic Type 4
	asphere_example2(&example2_axiasphere_properties,&example2_axicylinder_properties,&example2_axicircle_properties);

	//Setting up Optic Type 5
	asphere_example3(&example3_axiasphere_properties,&example3_axicylinder_properties,&example3_axicircle_properties);

}


void run() {

	double delta_supersample = view_size/static_cast<double>(IMAGE_HEIGHT*(NUMBER_OF_SUPERSAMPLES+1));

	for(int idx_y = 0; idx_y<IMAGE_HEIGHT; ++idx_y){
		for(int idx_x = 0; idx_x<IMAGE_WIDTH; ++idx_x){
			R_temp = 0.0;
			G_temp = 0.0;
			B_temp = 0.0;

			for(int ss_y = 1; ss_y<=NUMBER_OF_SUPERSAMPLES; ++ss_y){
				for(int ss_x = 1; ss_x<=NUMBER_OF_SUPERSAMPLES; ++ss_x){
					//Parallel rays
					rot_angle = static_cast<double>(view_angle)*3.14159/180.0;
					temp_x_pos = view_size*(static_cast<double>(idx_x-IMAGE_WIDTH/2)-0.5)/static_cast<double>(IMAGE_HEIGHT)+delta_supersample*static_cast<double>(ss_x);
					temp_y_pos = view_size*(static_cast<double>(idx_y-IMAGE_HEIGHT/2)-0.5)/static_cast<double>(IMAGE_HEIGHT)+delta_supersample*static_cast<double>(ss_y);
					ray_orig[0] = temp_x_pos*std::cos(rot_angle)-camera_offset*std::sin(rot_angle);
					ray_orig[1] = temp_y_pos;
					ray_orig[2] = -temp_x_pos*std::sin(rot_angle)-camera_offset*std::cos(rot_angle);
					ray_dir[0] = std::sin(rot_angle);
					ray_dir[1] = 0.0;
					ray_dir[2] = std::cos(rot_angle);

					if(optic_type == 0){
						ray_trace_optic(ray_orig, ray_dir, num_rays, ray_running, ray_alive, intensity, num_reflections, out_orig, out_dir, 0, &single_sphere_properties, 0, &blank_cylinder_properties, num_walls, wall_properties, 0, &blank_axicircle_properties, 0, &blank_axiasphere_properties);
					}
					if(optic_type == 1){
						ray_trace_optic(ray_orig, ray_dir, num_rays, ray_running, ray_alive, intensity, num_reflections, out_orig, out_dir, 1, &single_sphere_properties, 0, &blank_cylinder_properties, num_walls, wall_properties, 0, &blank_axicircle_properties, 0, &blank_axiasphere_properties);
					}
					if(optic_type == 2){
						ray_trace_optic(ray_orig, ray_dir, num_rays, ray_running, ray_alive, intensity, num_reflections, out_orig, out_dir, 2, lens_sphere_properties, 1, &lens_cylinder_properties, num_walls, wall_properties, 0, &blank_axicircle_properties, 0, &blank_axiasphere_properties);
					}
					if(optic_type == 3){
						ray_trace_optic(ray_orig, ray_dir, num_rays, ray_running, ray_alive, intensity, num_reflections, out_orig, out_dir, 0, &single_sphere_properties, 0, &blank_cylinder_properties, num_walls, wall_properties, 0, &blank_axicircle_properties, 1, &example1_axiasphere_properties);
					}
					if(optic_type == 4){
						ray_trace_optic(ray_orig, ray_dir, num_rays, ray_running, ray_alive, intensity, num_reflections, out_orig, out_dir, 0, &single_sphere_properties, 1, example2_axicylinder_properties, num_walls, wall_properties, 1, example2_axicircle_properties, 2, example2_axiasphere_properties);
					}
					if(optic_type == 5){
						ray_trace_optic(ray_orig, ray_dir, num_rays, ray_running, ray_alive, intensity, num_reflections, out_orig, out_dir, 0, &single_sphere_properties, 6, example3_axicylinder_properties, num_walls, wall_properties, 6, example3_axicircle_properties, 12, example3_axiasphere_properties);
					}
					process_ray_end(R_temp, G_temp, B_temp, ray_alive, intensity, num_reflections, out_orig, out_dir);
				}
			}
			pixels_clean[3*(idx_x+idx_y*IMAGE_WIDTH)] = 200.0*R_temp/static_cast<double>(NUMBER_OF_SUPERSAMPLES*NUMBER_OF_SUPERSAMPLES);
			pixels_clean[3*(idx_x+idx_y*IMAGE_WIDTH)+1] = 200.0*G_temp/static_cast<double>(NUMBER_OF_SUPERSAMPLES*NUMBER_OF_SUPERSAMPLES);
			pixels_clean[3*(idx_x+idx_y*IMAGE_WIDTH)+2] = 200.0*B_temp/static_cast<double>(NUMBER_OF_SUPERSAMPLES*NUMBER_OF_SUPERSAMPLES);
			m_image[3*(idx_x+(IMAGE_HEIGHT-1-idx_y)*IMAGE_WIDTH)] = (unsigned char)(pixels_clean[3*(idx_x+idx_y*IMAGE_WIDTH)]);
			m_image[3*(idx_x+(IMAGE_HEIGHT-1-idx_y)*IMAGE_WIDTH)+1] = (unsigned char)(pixels_clean[3*(idx_x+idx_y*IMAGE_WIDTH)+1]);
			m_image[3*(idx_x+(IMAGE_HEIGHT-1-idx_y)*IMAGE_WIDTH)+2] = (unsigned char)(pixels_clean[3*(idx_x+idx_y*IMAGE_WIDTH)+2]);
		}
	}

	write_bitmap(IMAGE_WIDTH, IMAGE_HEIGHT, bitmap_outfile, m_image, bitmap_header, bitmap_info, bitmap_pad);

}


void finish() {

	cleanup_ray_trace(ray_running, ray_alive, intensity, num_reflections, out_orig, out_dir);
	cleanup_walls(wall_properties);
	free(pixels_clean);
	cleanup_bitmap(m_image);

}


int main(int argc, char* argv[]) {

	if(argc>1){optic_type=atoi(argv[1]);}
	if(argc>2){view_angle=atof(argv[2]);}
	if(argc>3){view_size=atof(argv[3]);}

	initialize();
	run();
	finish();

}
