#include "raytracing.h"
#include <vector>
#include <SDL2/SDL.h>

#define IMAGE_WIDTH 1500
#define IMAGE_HEIGHT 1000
#define NUMBER_OF_SUPERSAMPLES 1


//Optic types:
// 0: no optic
// 1: single glass sphere
// 2: single convex lens defined via two spheres and a cylinder
// 3: single aspherical surface
// 4: single aspherical lens defined via two aspherical surfaces, a plane and cylinder
// 5: multiple aspherical lenses
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


class Image {
    public:
        Image();

        ~Image();

        void Initialise(const int xSize, const int ySize, SDL_Renderer *pRenderer);

        void SetPixel(const int x, const int y, const double red, const double green, const double blue);

        void Display();

    private:
        Uint32 ConvertColour(const double red, const double green, const double blue);

        void InitTexture();

        std::vector<std::vector<double>> m_rChannel;
        std::vector<std::vector<double>> m_gChannel;
        std::vector<std::vector<double>> m_bChannel;

        int m_xSize, m_ySize;

        SDL_Renderer *m_pRenderer;
        SDL_Texture *m_pTexture;
};


Image::Image() {
    m_xSize = 0;
    m_ySize = 0;
    m_pTexture = NULL;
}


Image::~Image() {
    if (m_pTexture != NULL) {
        SDL_DestroyTexture(m_pTexture);
    }
}

void Image::Initialise(const int xSize, const int ySize, SDL_Renderer *pRenderer) {
    // resize image arrays
    m_rChannel.resize(xSize, std::vector<double>(ySize, 0.0));
    m_gChannel.resize(xSize, std::vector<double>(ySize, 0.0));
    m_bChannel.resize(xSize, std::vector<double>(ySize, 0.0));

    // store dimensions
    m_xSize = xSize;
    m_ySize = ySize;

    // store the pointer to the renderer
    m_pRenderer = pRenderer;

    // init texture
    InitTexture();
}


void Image::SetPixel(const int x, const int y, const double red, const double green, const double blue) {
    m_rChannel.at(x).at(y) = red;
    m_gChannel.at(x).at(y) = green;
    m_bChannel.at(x).at(y) = blue;
}


void Image::Display() {
    // allocate ememory for a pixel buffer
    Uint32 *tempPixels = new Uint32[m_xSize * m_ySize];

    // clear the pixel buffer
    memset(tempPixels, 0, m_xSize * m_ySize * sizeof(Uint32));

    for (int x = 0; x < m_xSize; ++x) {
        for (int y = 0; y < m_ySize; ++y) {
            tempPixels[(y * m_xSize) + x] = ConvertColour(m_rChannel.at(x).at(y), m_gChannel.at(x).at(y), m_bChannel.at(x).at(y));
        }
    }

    // update the texture with this pixel buffer
    SDL_UpdateTexture(m_pTexture, NULL, tempPixels, m_xSize * sizeof(Uint32));

    delete[] tempPixels;

    // copy texture to the renderer
    SDL_Rect srcRect, bounds;
    srcRect.x = 0;
    srcRect.y = 0;
    srcRect.w = m_xSize;
    srcRect.h = m_ySize;

    bounds = srcRect;

    SDL_RenderCopy(m_pRenderer, m_pTexture, &srcRect, &bounds);
}


void Image::InitTexture() {
    // initialise texture
    Uint32 rmask, gmask, bmask, amask;

    #if SDL_BYTEORDER == SDL_BIG_ENDIAN
        rmask = 0xff000000;
        gmask = 0x00ff0000;
        bmask = 0x0000ff00;
        amask = 0x000000ff;
    #else
        rmask = 0x000000ff;
        gmask = 0x0000ff00;
        bmask = 0x00ff0000;
        amask = 0xff000000;
    #endif

    // delete any previously created textures

    if (m_pTexture != NULL) {
        SDL_DestroyTexture(m_pTexture);
    }

    SDL_Surface *tempSurface = SDL_CreateRGBSurface(0, m_xSize, m_ySize, 32, rmask, gmask, bmask, amask);
    m_pTexture = SDL_CreateTextureFromSurface(m_pRenderer, tempSurface);
    SDL_FreeSurface(tempSurface);
}


Uint32 Image::ConvertColour(const double red, const double green, const double blue) {
    unsigned char r = static_cast<unsigned char>(red);
    unsigned char g = static_cast<unsigned char>(green);
    unsigned char b = static_cast<unsigned char>(blue);

    #if SDL_BYTEORDER == SDL_BIG_ENDIAN
        Uint32 pixelColour = (r << 24) + (g << 16) + (b << 8) + 255;
    #else
        Uint32 pixelColour = (255 << 24) + (r << 16) + (g << 8) + b;
    #endif

    return pixelColour;
}


class Raytracer {
	public:
		Raytracer();

		int OnExecute();
		bool OnInit();
		void OnEvent(SDL_Event *event);
		void OnLoop();
		void OnExit();
		void OnRender();

	private:
		// an instance of the Image class to store the image
		Image m_image;
		// SDL2 stuff
		bool isRunning;
		SDL_Window *pWindow;
		SDL_Renderer *pRenderer;
};


Raytracer::Raytracer() {

	isRunning = true;
	pWindow = NULL;
	pRenderer = NULL;

}


bool Raytracer::OnInit() {

	if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
		return false;
	}

	pWindow = SDL_CreateWindow(
		"Raytracer",
		SDL_WINDOWPOS_CENTERED,
		SDL_WINDOWPOS_CENTERED,
		IMAGE_WIDTH, IMAGE_HEIGHT,
		SDL_WINDOW_SHOWN
	);

	if (pWindow != NULL) {
		pRenderer = SDL_CreateRenderer(pWindow, -1, 0);

		m_image.Initialise(IMAGE_WIDTH, IMAGE_HEIGHT, pRenderer);

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
	else {
		return false;
	}

	return true;

}


int Raytracer::OnExecute() {
	SDL_Event event;

	if (OnInit() == false) {
		return -1;
	}

	while (isRunning) {
		while (SDL_PollEvent(&event) != 0) {
			OnEvent(&event);
		}

		OnLoop();
		OnRender();
	}

	return 0;
}


void Raytracer::OnEvent(SDL_Event *event) {
    switch (event->type) {

    case SDL_QUIT:
        isRunning = false;
        break;

    case SDL_KEYDOWN:
        switch (event->key.keysym.scancode) {

        case SDL_SCANCODE_W:
            view_size-=1.0;
            break;

        case SDL_SCANCODE_S:
            view_size+=1.0;
            break;

        case SDL_SCANCODE_A:
            view_angle-=5;
            break;

        case SDL_SCANCODE_D:
            view_angle+=5;
            break;

        case SDL_SCANCODE_0:
            optic_type=0;
            break;

        case SDL_SCANCODE_1:
            optic_type=1;
            break;

        case SDL_SCANCODE_2:
            optic_type=2;
            break;

        case SDL_SCANCODE_3:
            optic_type=3;
            break;

        case SDL_SCANCODE_4:
            optic_type=4;
            break;

        case SDL_SCANCODE_5:
            optic_type=5;
            break;

        default:
            break;
        }
        break;

    case SDL_KEYUP:
        break;

    default:
        break;
    }
}


void Raytracer::OnLoop() {

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
			m_image.SetPixel(
				idx_x,
				idx_y,
				int(pixels_clean[3*(idx_x+idx_y*IMAGE_WIDTH)]),
				int(pixels_clean[3*(idx_x+idx_y*IMAGE_WIDTH)+1]),
				int(pixels_clean[3*(idx_x+idx_y*IMAGE_WIDTH)+2])
			);
		}
	}

}


void Raytracer::OnRender() {

	SDL_SetRenderDrawColor(pRenderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
	SDL_RenderClear(pRenderer);

	m_image.Display();

	SDL_RenderPresent(pRenderer);

}


void Raytracer::OnExit() {

	cleanup_ray_trace(ray_running, ray_alive, intensity, num_reflections, out_orig, out_dir);
	cleanup_walls(wall_properties);
	free(pixels_clean);

	SDL_DestroyRenderer(pRenderer);
	SDL_DestroyWindow(pWindow);
	pWindow = NULL;
	SDL_Quit();

}


int main(int argc, char* argv[]) {

	Raytracer rt;
	return rt.OnExecute();

}
