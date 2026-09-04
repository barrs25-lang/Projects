This library of Matlab files was developed by Dr. S.C.Southward over multiple
years.  This library contains functions, scripts, and example waypoint data sets
for performing numerical operations using clothoid curves for autonomous vehicle
control system development.

Dr. S. C. Southward
Associate Professor
Mechanical Engineering
Virginia Tech
scsouth@vt.edu


Last Updated:  3/11/2023

---- These files are associated with empirical curvature rate limits

test_kplimit_search.m  (try me!)
    This script is used to validate the kplimit_search.m function for
    finding kpmin and kpmax for a randomly chosen target radius and a
    randomly chosen northing and easting coordinate.

kplimit_search.m
    This function finds extremal curvature rate limits kpmin and kpmax
    given a target radius and an initial curvature.

gen_kplimit_data.m  (try me!)
    This script evaluates clothoids over a grid of target radii and
    curvature values to determine a lookup table for the min and max
    curvature rates for each grid condition.

gen_kplim_include.m
    This script auto-generates a Matlab m-file script that defines the upper
    and lower limits for curvature rate as a function of target distance and
    initial curvature.

kplim_include.m
    This is auto-generated code that must be copied/pasted into the
    get_kp_limit.m function manually.

get_kp_limit.m  (intended for real-time implementation)
    This function computes the maximum and minimum values of curvature rate
    (kpmax, kpmin) as a function of target distance R (meters) and initial
    curvature k0 (rad/meter) using a lookup table 2D linear interpolation

find_nearest_grid.m  (intended for real-time implementation)
    This function determines the index of the 1D grid point closest to the
    input value.


---- These files are associated with general clothoid computation

clothoid_endpoint.m
    This function computes the (north,east) endpoint coordinates and the 
    path length of a clothoid that reaches a target radius R within
    a specified tolerance distance.

gen_clothoid.m
    This function generates a clothoid curve using a trapezoidal integral
    approximation to the clothoid integrals.

nearest_point_to_line.m
    This function determines the point on a line segment that is closest 
    to a test point (x0,y0).

nearest_point_to_wpts.m
    This function determines the point on a collection of line segments
    defined by waypoint vectors xwp and wyp that is closest to a test point



---- These files are associated with optimal curvature rate estimation

test_opt_curvature_rate.m  (try me!)
    This script validates the get_opt_curvature_rate.m function by allowing
    the user to select a number of target locations using the graphics 
    cursor and then plotting the resulting clothoid curve.

get_opt_curvature_rate.m  (intended for real-time implementation)
    This function determines the optimal curvature rate given an initial
    curvature, an initial pose, a final target, and an initial heading.


---- These files are associated with waypoint data generation and visualization

test_get_waypoint_data.m  (try me!)
    This script evaluates the get_waypoint_data.m function for every
    waypoint data set and generates a plot of the resulting output data.

get_waypoint_data.m
    This function loads waypoint data for simulation testing

velocity_waypoints.m  (intended for real-time implementation)
    This function defines a vector of velocity waypoints that attempt to use
    the maximum specified velocity, but derates the velocities based on
    acceleration limits.

plot_waypoint_data.m
    This function generates a plot of the waypoint data that also includes
    the computed reference velocity waypoints and the corresonding lateral
    and longitudinal accelerations.

plot_waypoint_matfiles.m  (try me!)
    This script plots the waypoint data from matfiles.

curvature_and_heading.m  (intended for real-time implementation)
    This function calculates the curvature at each waypoint by first fitting
    parametric polynomials to the northing and easting waypoints.  The
    analytic curvature is then computed from the coefficients.  This
    function also calculates the heading at each waypoint using the fitted 
    polynomial coefficients.


---- These files are associated with helper functions

test_waypoint_analysis.m. (try me!)
    This script validates the full set of clothoid library tools by moving
    the car to each waypoint pose in a specified data set, computing the
    target at a specified lookahead point, computing the optimal curvature
    rate, computing the optimal clothoid trajectory, and plotting everything
    in birdseye view animation.


test_vc_mod.m (try me!)
    This function evaluates the vc_mod.m function and compares its output to
    the built-in Matlab MOD() function.

LineNormals2D.m
    This function calculates the normals of the line points
    using the neighboring points of each contour point, and 
    forward an backward differences on the end points.
    Source code obtained from Mathworks.

vc_mod.m  (intended for real-time implementation)
    Alternative to the Matlab mod(x,y) function that does not require a
    divide and can be converted to C or C++ properly.

draw_car.m
    This function draws a wireframe car on the current figure at the
    desired pose specified by xcar (meters), ycar (meters), and head
    (radians).  The lookahead (meters) input specifies the length of a
    heading indicator starting at the center of the rear axle and pointing
    forward.

subplotfill.m
    This function expands the borders of a normal subplot to reduce the
    amount of whitespace around the edge of the axes.

printpng.m
    This function generates a .png file from the curent figure window at a
    user-specified dimension.




