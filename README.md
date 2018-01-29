# Extended Kalman Filter Project
## Building the project

.
The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF


Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

## Summary of the Project

After compiling the project and running it with the simulator, the values are within the required boundaries (i.e. RMSE
is within the expected boundaries after a few iterations).

## File structure

The basic file structure of the code has been kept, with the addition of a few helper functions in tools.cpp

## Comments on the code

### Eigen and auto keyword
I had serious problems with odd calculations during development. To follow a divide-and-conquer strategy, I focused on LASER
first. My code is peppered with debug info and assert statements (fail early approach to find problems). During 
production, both assert() and cout (within #ifdef DEBUG) would not impact runtime.

The reason for the problem where the following:

I had a statement like
auto X  = <eigenlib expression>
and it makes a huge difference, if you use auto as keyword for an lvale of an expression for Eigen. I had to debug and revert to actually specifying the type on the lvalue.....

For more comfortable debugging, I was printing the tables in HTML to be able to view them in parallel. An Eigen::IOFormat for that can be found in tools.h

### Fixing div_zero

To avoid division by zero, I am using a macro FIX_FLOAT in various places. Definition of that macro is
	#define FLOAT_FIX(x) ((fabs(x)) < 0.00001 ? 0.00001 : (x))
(in tools.h)

### Normalizing angle
To normalize the angles, I am using the following code:
  for(; y(1) < -M_PI; y(1) += 2*M_PI);
  for(; y(1) > M_PI;  y(1) -= 2*M_PI);
  
Alternative could be modulo operation or atan2 with the angle. This would need performance analysis to find the solution with better runtime. For simplicity, the loop version is being used.

### Google Test
When starting development, I actually used google test. Test cases are in tests.cpp. However, the relevant statements are disabled in CmakeLists.txt, since the requirement of the rubric is that the project should compile out-of-the box without installing additional stuff.


## Additional stuff
The project's dependencies in the future could also be managed with conan.io, which would make installation with apt-get unnecessary and would keep the operating system clean.

## Potential optimizations

### delta_t / Q matrix
If we were absolutely sure that we receive an update every delta_t, we could actually pre-calculate Q. However, a system needs to be fault tolerant. So an optimization might be to have Q pre-calculated for the expected delta_t and just calculate
a specific Q if delta_t is different

### inlining

updateCommon() might be inlined or refactored to avoid function calls


