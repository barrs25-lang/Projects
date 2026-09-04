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
%  This script evaluates the nearest_point_to_wpts.m function.  The script
%  first allows the user to select a set of waypoints using a graphics
%  cursor.  After the last waypoint has been selected, the script
%  continuously checks the mouse location and finds the closest point to
%  the waypoint path.
%
%  This script calls the following functions:
%       nearest_point_to_wpts.m
%       nearest_point_to_line.m
%       mouse_pointer_callback.m

%  --define the configuration parameters

clear, clc
clear get_opt_curvature_rate
clear mouse_pointer_callback

ms = 20;  lw = 2;


%  --draw the initial test figure

figure('tag','testfig')
subplotfill(1,1,1);
h = plot(0,0,'.');
grid on
hold on
ylim(15*[-1,1])
xlim(25*[-1,1])
axis equal
set(gca,'xlimmode','manual','ylimmode','manual','tag','testaxes');
delete(h);
xlabel('Easting [m]');
ylabel('Northing [m]');


%  --get and plot a list of waypoints from the user

NUMWPTS = 5;  % change this for more or less waypoints

for n = 1:NUMWPTS
    [EWP(n),NWP(n)] = ginput(1);
    hwpt(n) = plot(EWP(n),NWP(n),'b.','markersize',ms);
    text(EWP(n),NWP(n),int2str(n), ...
        'horizontalalignment','center', ...
        'verticalalignment','top');

    if (n > 1)
        plot(EWP([n-1:n]),NWP([n-1:n]),'b','linewidth',lw);
    end
end

set(gca,'userdata',hwpt);
set(gcf,'Pointer','CrossHair', ...
    'WindowButtonMotionFcn','mouse_pointer_callback')


