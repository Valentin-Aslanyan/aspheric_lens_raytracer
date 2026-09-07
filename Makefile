all:
	g++ aspherical.cpp raytracing.cpp Raytrace_GUI.cpp -o Raytrace_GUI -lSDL2
	g++ aspherical.cpp raytracing.cpp Raytrace_BMP.cpp -o Raytrace_BMP
