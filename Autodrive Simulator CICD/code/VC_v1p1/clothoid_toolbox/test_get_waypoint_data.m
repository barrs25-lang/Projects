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
%  This script evaluates the get_waypoint_data.m function for every
%  waypoint data set and generates a plot of the resulting output data.
%
%  This script calls the following functions:
%       get_waypoint_data.m
%       plot_waypoint_data.m

clear, clc

%  --define the configuration parameters

vstart = 0.1;
vmax = 10;
vfinal = 0;

max_alat = 3.25;
max_along = 3.25;

randrotate = false;
plotflag = true;

%  --check to see if the plots folder exists
if (exist('waypoint_plots','dir') == 0)
    %  --create the plots folder
    mkdir('waypoint_plots')
end


%  --loop through each testcase data set

for testcase = 1:12
    %  --get the waypoint data

    [NWP, EWP, HWP, VWP, KWP, ZWP, SWP, GWP, coursename] = get_waypoint_data( ...
        testcase, vstart, vmax, vfinal, max_alat, max_along, randrotate);

    %  --plot the waypoint data
    plotfilename = sprintf('./waypoint_plots/vref_data_%d.png',testcase);
    plot_waypoint_data(NWP, EWP, HWP, VWP, KWP, ZWP, SWP, coursename, ...
        vmax, max_along, max_alat, plotfilename);
end

