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

We assume a ray takes the form 

$$\vec{r}=\vec{p}+t\vec{q}$$

Any position along the ray can be parameterized via the distance $t$. When raytracing, a given ray must be tested against all primitives (such as an aspheric surface) in a scene to check for any intersections. If an intersection (hit) occurs, it is useful to have the position of the hit and the normal to the primitive $\vec{N}$ (for refraction calculations). The solutions to intersections with primitives are also parameterized via $t$. A straight ray may intersect multiple primitives and therefore have multiple solutions for $t$; only the one with the lowest (positive) $t$, namely $t_{min}$ is desired. A general function representing a primitive therefore has:

**Inputs**

  - `ray_orig` $\Leftrightarrow$ $\vec{p}$
  
  - `ray_dir` $\Leftrightarrow$ $\vec{q}$
  
  - `properties` $\Leftrightarrow$ $R,k,A_i$ (for aspheres; whatever the other primitive properties are for others)

**Outputs**

  - `bool` (function's return value) Has a hit occurred at all?
  
  - `t_hit` $\Leftrightarrow$ $t_{min}$
  
  - `hit_pos` $\Leftrightarrow$ $\vec{p}$ $+$ $t_{min}$ $\vec{q}$
  
  - `hit_normal` $\Leftrightarrow$ $\vec{N}$

  - Note that if the function's first return value is `false`, the other returns are garbage.

Take the function `rayintersect_axiasphere` and adapt it to your own project. The floating point numbers are `double`s for simplicity, but should be modified as necessary. 3-vectors are written as elementary pointers, but should be replaced with custom structs used in your own project.

For the aspheric surface, after the remaining `properties` are set, the function `set_axiasphere_bounds` must be called on it.

There are supplementary functions to help set up a scene and so on, see the rest of the project for examples of how to use them.

## How it works
