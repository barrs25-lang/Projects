function [final_index,xc,yc,uc,v0,ds] = ...
    nearest_point_to_wpts(initial_index,x0,y0,numwp,xwp,ywp)
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
%  This function determines the point on a collection of line segments
%  defined by waypoint vectors xwp and wyp that is closest to a test point
%  
%  initial_index = starting index to begin search
%  x0 = x-dir coordinate of test point
%  y0 = y-dir coordinate of test point
%  numwp = total number of valid waypoints in the arrays
%  xwp = x-dir coordinate vector of waypoints
%  ywp = y-dir coordinate vector of waypoints
%
%  final_index = final index of starting waypoint closest to (x0,y0)
%  xc = x-dir coordinate of point on segment closest to (x0,y0)
%  yc = y-dir coordinate of point on segment closest to (x0,y0)
%  uc = u0 = tangential distance along the segment of (xc,yc) from (x1,y1)
%  v0 = perpendicular distance of (x0,y0) from the segment
%  ds = u2 = length of segment from (x1,y1) to (x2,y2)
%
%  This function calls the following handcoded functions:
%       nearest_point_to_line.m
%
%  Last Modified:  2/18/2023

%  --initialize local variables for the search

NWP = numwp;

index = initial_index;
if (index < 1)
    index = 1;
elseif (index > (NWP-1))
    index = (NWP-1);
end

increment = 0;
search_complete = false;
xc = xwp(index);
yc = ywp(index);
uc = 0;
v0 = 0;
ds = 0;


%  --search one segment at a time

while (~search_complete)
    %  --get the endpoints of the next segment
    xstart = xwp(index);
    ystart = ywp(index);

    xend = xwp(index+1);
    yend = ywp(index+1);

    %  --compute the closest point on this segment
    [xc,yc,uc,v0,ds] = ...
        nearest_point_to_line(x0,y0,xstart,ystart,xend,yend);

    %  --check for interior point
    if (uc < 0)
        %  --test point is prior to this segment
        if (increment == 1)
            %  --stuck at a waypoint
            increment = 0;
            xc = xstart;  yc = ystart;  uc = 0;
            search_complete = true;
        else
            %  --ok to check previous segment
            increment = -1;
        end

    elseif (uc > ds)
        %  --test point is beyond this segment
        if (increment == -1)
            %  --stuck at a waypoint
            increment = 0;
            xc = xend;  yc = yend;  uc = ds;
            search_complete = true;
        else
            %  --ok to check next segment
            increment = +1;
        end

    else
        %  --test point is in the interior of this segment
        increment = 0;
        search_complete = true;
    end

    %  --increment the index
    index = index + increment;


    %  --check for index out of range
    if (index < 1)
        %  --index points to the very first point avaiable
        index = 1;
        search_complete = true;
    elseif (index > (NWP-1))
        %  --index points to the very last point avaialble
        index = (NWP-1);
        search_complete = true;
    end

end

%  --transfer the final index
final_index = index;

return
