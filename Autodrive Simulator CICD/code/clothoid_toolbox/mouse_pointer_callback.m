function mouse_pointer_callback()
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
%  This function handles callbacks whenever the mouse pointer is moved.
%  This function will determine the location of the mouse pointer, scale it
%  to the axis units, and extract the waypoints plotted in the axis.  Then,
%  it will call the nearest_point_to_wpts.m function to compute the closest
%  point on the waypoint path, and finally draw a line from the mouse
%  pointer to the closest point.

%  This function calls the nearest_point_to_wpts.m function

%  Last Updated:  11/12/2023

%  --declare the local persistent variable

persistent initial_index
if isempty(initial_index)
    initial_index = 1;
end


%  --get the handles to the figure and axes containing waypoint data

hfig = findobj('tag','testfig');
hax = findobj('tag','testaxes');


%  --get the mouse position in pixel coordinates

% innerpos = getpixelposition(hax);
set(hax,'units','pixels');
innerpos = tightPosition(hax);

pmin = innerpos(1);  pmax = pmin + innerpos(3);  % x-dir pixel limits
qmin = innerpos(2);  qmax = qmin + innerpos(4);  % y-dir pixel limits

mousepos = get(hfig,'CurrentPoint');   % figure pixels
pmouse = mousepos(1);  % x-dir pixel coordinate
qmouse = mousepos(2);  % y-dir pixel coordinate


%  --convert the mouse pixel coordinates to axis coordinates

xl = get(hax,'xlim');  % axes x-limit coordinates
yl = get(hax,'ylim');  % axes y-limit coordinates

xmin = xl(1);  xmax = xl(2);  % x-dir axis limits
ymin = yl(1);  ymax = yl(2);  % y-dir axis limits

mx = (xmax-xmin) / (pmax-pmin);  % xform slope in x-dir
my = (ymax-ymin) / (qmax-qmin);  % xform slope in y-dir

xmouse = xmin + mx * (pmouse - pmin);  % x-dir mouse position
ymouse = ymin + my * (qmouse - qmin);  % y-dir mouse position


%  --limit the range of the mouse position to be inside the axes

xmouse = min(xmouse,xmax);
xmouse = max(xmouse,xmin);
ymouse = min(ymouse,ymax);
ymouse = max(ymouse,ymin);


%  --extract the waypoints from the axes

hwp = get(hax,'userdata');
numwp = length(hwp);
xwp = zeros(1,numwp);
ywp = zeros(1,numwp);
for n = 1:numwp
    xwp(n) = get(hwp(n),'xdata');
    ywp(n) = get(hwp(n),'ydata');
end


%  --determine the distance from the mouse pointer to the waypoints

[final_index,xc,yc,uc,v0,ds] = ...
    nearest_point_to_wpts(initial_index,xmouse,ymouse,numwp,xwp,ywp);

if (initial_index ~= final_index), beep, end

initial_index = final_index;  % save for the next search


%  --plot a line from the cursor to the waypoint path

delete(findobj('tag','closestline'));

h = plot([xmouse,xc],[ymouse,yc],'r','linewidth',2);
set(h,'tag','closestline');

title({sprintf('Waypoint Index=%g,  Segment %g-%g Length=%g', ...
    final_index,final_index,final_index+1,ds), ...
    sprintf('Tangential Distance=%g,  Perpendicular Distance=%g',uc,v0)}, ...
    'tag','closestline');

end
