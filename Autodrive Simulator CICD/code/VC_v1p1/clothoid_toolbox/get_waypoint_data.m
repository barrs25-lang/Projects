function [NWP, EWP, HWP, VWP, KWP, ZWP, SWP, GWP, coursename] = get_waypoint_data( ...
    testcase, vstart, vmax, vfinal, max_alat, max_along, randrotate)
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
%
%  This function loads waypoint data for simulation testing
%
%  testcase = enum input for selecting a waypoint data set
%  vstart = initial velocity at first waypoint (m/s)
%  vmax = maximum velocity (m/s)
%  vfinal = final velocity at last waypoint (m/s)
%  max_alat = maximum lateral acceleration (m/s/s)
%  max_along = maximum longitudinal acceleration (m/s/s)
%  randrotate = Boolean request to randomly rotate the data around the Z axis
%  clothoid = Boolean request to use waypoint data specifically for clothoids
%
%  NWP = vector of northing direction waypoints (m)
%  EWP = vector of easting direction waypoints (m)
%  HWP = vector of heading waypoints (deg)
%  VWP = vector of velocity waypoints (m/s)
%  KWP = vector of curvature waypoints (rad/m)
%  ZWP = vector of elevation waypoints (m)
%  SWP = vector of waypoint-to-waypoint spacing (m)
%  GWP = vector of grade angle at each waypoint (rad)
%  coursename = string defining the waypoint data set course name

%  This function calls the velocity_waypoints.m function

%  Last Updated:  3/2/2023

%  --define the defaults when not specified

if (nargin < 7), randrotate = false; end
if (nargin < 8), plotflag = false; end


%  --define the files for each testcase

files = {'lanechange.mat', ...          % 1
    'figure8.mat', ...                  % 2
    'Zandvoort_clothoid.mat', ...       % 3
    'Zandvoort.mat', ...                % 4
    'Yuma_clothoid.mat', ...            % 5
    'Yuma.mat', ...                     % 6
    'VIR_clothoid.mat', ...             % 7
    'VIR.mat', ...                      % 8
    'RoadAmerica_clothoid.mat', ...     % 9
    'RoadAmerica.mat', ...              % 10
    'PikesPeak_clothoid.mat', ...       % 11
    'PikesPeak.mat'};                   % 12


%  --get the folder containing waypoint data

waypoint_path = './waypoint_data';
if (exist(waypoint_path) ~= 7)
    %  --ask user to find the folder of waypoint data
    waypoint_path = uigetdir('./','Select the folder containing waypoint data');

    if (waypoint_path == 0), return, end
end


%  --prepare the requested data

file_to_load = strcat(waypoint_path,'/',files{testcase});

if (exist(file_to_load,'file') == 2)
    load(file_to_load)
else
    fprintf('File (%s) not found on path.\n',file_to_load);
    return;
end

NUMWP = length(NWP);


%  --check to see if a random rotation should be applied

if randrotate
    %  --apply a random rotation angle
    theta = pi * (2*rand(1)-1);  % -pi < theta < pi
    
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    for n = 1:NUMWP
        q = R * [NWP(n); EWP(n)];
        NWP(n) = q(1);
        EWP(n) = q(2);
    end
    HWP = HWP + theta*(180/pi);

    for m = 1:length(HWP)
        HWP(m) = vc_mod(HWP(m) + 360,360);
    end
end


%  --define the velocity waypoint data

VWP = velocity_waypoints(KWP,SWP,vstart,vmax,vfinal,max_alat,max_along);


%  --compute the road grade (pitch angle) at each waypoint
%    Note:  tan(grade) = dZ/ds

GWP = atan(gradient(ZWP,SWP));  % radians

return
