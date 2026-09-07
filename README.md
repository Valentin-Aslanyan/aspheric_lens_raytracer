# Aspheric Lens Raytracer

Routines to raytrace aspheric surfaces (intended to be used in your own project) are packaged with a simple raytracer as a demonstration.

<details><summary>Definition of an aspheric surface</summary>
  Certain lenses make use of refractive surfaces defined by
  
  $$z = c+\dfrac{\rho^2}{R\left[1+\sqrt{1-(1+k)\left(\dfrac{\rho}{R}\right)^2}\right]} + \sum_i A_i \rho^i$$
  
  where we assume that the surface is symmetric about the optical ($z$-) axis, $\rho^2 = x^2+y^2$, $c$ is the intercept of the optical axis, $R$ is the radius of curvature, $k$ is the conic constant, and all $A_i$ are constants (see, for example, US Patent Application 2020/0333565A1 which we are considering in this codebase).
</details>

## Quick demonstration
<details><summary>Compiling</summary>
  This project only requires the basic C/C++ libraries. The Graphical User Interface version requires SDL2. You can compile it on UNIX by typing
  
  ```
  make
  ```

  or in a command line on most OS by typing
  
  ```
  cmake .
  cmake --build .
  ```

  If you do not have SDL2, you can still use the version that makes a Bitmap image from a command line input.
</details>

If you run `Raytrace_GUI(.exe)` you will be able to select one of the following demonstration optics using the ordinary number keys:

```
0 = No optic
1 = Glass sphere
2 = Convex lens
3 = Single aspheric surface
4 = Aspheric lens
5 = Compound aspheric lens consisting of multiple optics
```
You can then use the `A` and `D` keys to rotate the view angle left/right and the `W` and `S` keys to zoom in/out.

If you run `Raytrace_BMP.(exe)` you must supply optional command-line arguments, in order, for the Optic Type (see above), View Angle (degrees), view size (smaller is more zoomed in). For example, on Linux the following would be optic type 3, 90 degree viewing angle, zoom size 6:

```
./Raytrace_BMP 3 90 6
```


## Using this in your own project




## How it works
