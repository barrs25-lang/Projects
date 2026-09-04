function [xc,yc,uc,v0,ds] = nearest_point_to_line(x0,y0,x1,y1,x2,y2)
%------------------------------------------------------------------------
%  Copyright (C) 2023  S. C. Southward
% 
%  This program is free software: you can redistribute it and/or modify
%  it under the terms of the GNU General Public License as published by
%  the Free Software Foundation, either version 3 of the License, or
%  (at your option) any later version.
% 
%  This program is distributed in the hope that it will be useful,
%  but WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  GNU General Public License for more details.
% 
%  You should have received a copy of the GNU General Public License
%  along with this program.  If not, see <https://www.gnu.org/licenses/>.
% -----------------------------------------------------------------------
%  This function determines the point on a line segment that is closest 
%  to a test point (x0,y0).
%  
%  x0 = x-dir coordinate of test point
%  y0 = y-dir coordinate of test point
%  x1 = x-dir coordinate of starting point on segment
%  y1 = y-dir coordinate of starting point on segment
%  x2 = x-dir coordinate of ending point on segment
%  y2 = y-dir coordinate of ending point on segment
%
%  xc = x-dir coordinate of point on segment closest to (x0,y0)
%  yc = y-dir coordinate of point on segment closest to (x0,y0)
%  uc = u0 = tangential distance along the segment of (xc,yc) from (x1,y1)
%  v0 = perpendicular distance of (x0,y0) from the segment
%  ds = u2 = length of segment from (x1,y1) to (x2,y2)
%
%  This function does not call any handcoded functions.
%
%  Last Modified:  1/31/2023
%
%  https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line

%  Step 1:  Compute the coordinates of the closest point on the line

%  --define the coefficients of the line segment:  a*x + b*y + c = 0
%    This formulation enables the line to have any slope in the x-y plane
%    and is not restricted to finite slope (one-to-one) lines

a = y2 - y1;
b = x1 - x2;
c = x2*y1 - x1*y2;


%  --compute the coordinates of (xc,yc) that are on the segment line

d2 = (a^2 + b^2);  % squared length of segment

xc = ( b*(b*x0 - a*y0) - a*c) / d2;
yc = (-a*(b*x0 - a*y0) - b*c) / d2;


%  Step 2:  Compute radial and tangential components
%
%  The original Cartesian coordinate system is defined by points (x,y)
%  A new Cartesian coordinate system is defined by points (u,v)
%  The origin of the new coordinate system is at the starting point (x1,y1)
%    (u=0,v=0) => (x=x1,y=y1)
%  The u-axis of the new coordinate system is parallel with the segment
%  The v-axis of the new coordinate system is perpendicular to the segment
%  The transformation from (x,y) to (u,v) is given by:
%
%  |u| = | cos(theta)  sin(theta) | * |x - x1|
%  |v|   |-sin(theta)  cos(theta) |   |y - y1|
%
%  The transformation from (u,v) back to (x,y) is given by:
%
%  |x| = | cos(theta) -sin(theta) | * |u| + |x1|
%  |y|   | sin(theta)  cos(theta) |   |v|   |y1|
%
%  where theta is the rotation angle (rad) required to rotate the segment
%  to be parallel to a horizontal axis.  Note, there is NO need to actually
%  compute this angle or call the cos() or sin() functions.

%  --use trig identities to compute the rotation matrix elements

ds = sqrt(d2);    % actual segment length

cos_th = -b/ds;   % delta_x / segment_length
sin_th = a/ds;    % delta_y / segment_length


%  --compute uc:  tangential distance of (xc,yc) from (x1,y1)
%    uc is a measure of how far (xc,yc) is from (x1,y1)

uc = cos_th * (xc - x1) + sin_th * (yc - y1);  % also equal to u0


%  --compute v0:  transverse distance of (x0,y0) from the segment
%    v0 is a measure of how far (x0,y0) is from the segment

v0 = -sin_th * (x0 - x1) + cos_th * (y0 - y1);

return
