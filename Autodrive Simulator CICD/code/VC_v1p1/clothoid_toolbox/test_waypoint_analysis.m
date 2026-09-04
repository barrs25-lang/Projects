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
%  This script validates the full set of clothoid library tools by moving
%  the car to each waypoint pose in a specified data set, computing the
%  target at a specified lookahead point, computing the optimal curvature
%  rate, computing the optimal clothoid trajectory, and plotting everything
%  in birdseye view animation.
%
%  This script calls the following functions:
%       get_waypoint_data.m
%       draw_car.m
%       nearest_point_to_wpts.m
%       get_kp_limit.m
%       get_opt_curvature_rate.m
%       clothoid_endpoint.m
%       gen_clothoid.m

%  Last modified:   3/11/2023

clear, clc

clear draw_car
ms = 20;
lw = 1;


%  --load a set of waypoint data for testing

testcase = 2;

vstart = 0.1;
vmax = 10;
vfinal = 0;

max_alat = 3.25;
max_along = 3.25;

randrotate = true;
zoomview = true;

[NWP, EWP, HWP, VWP, KWP, ZWP, SWP, GWP, coursename] = get_waypoint_data( ...
    testcase, vstart, vmax, vfinal, max_alat, max_along, randrotate);

NUM_WP = length(NWP);


%  --plot a birdseye view

figure(1)
clf
plot(NWP,EWP,'b.-','markersize',ms,'linewidth',lw)
grid on
hold on
draw_car(NWP(1),EWP(1),(pi/180)*HWP(1),5,lw);
axis equal
xlabel('North (m)')
ylabel('East (m)')


%  --define configuration parameters for the simulation

R = 6.5;  % target lookahead radius (m)
dy = 14;
dx = 20;

index = 1;
maxiter = 10;
tol = 0.001;


%  --step through each waypoint

for n = 1:NUM_WP
    %  --place the car at the next waypoint

    ncar = NWP(n);  % northing (m)
    ecar = EWP(n);  % easting (m)
    kcar = KWP(n);
    psi = (pi/180)*HWP(n);  % heading (rad)
    draw_car(ncar,ecar,psi,R,lw);


    %  --get the next target location on the path

    nlookahead = ncar + R*cos(psi);
    elookahead = ecar + R*sin(psi);
    plot(nlookahead,elookahead,'g.','markersize',ms);

    [index,ntarget,etarget,~,~,~] = ...
        nearest_point_to_wpts(index,nlookahead,elookahead,NUM_WP,NWP,EWP);

    plot(ntarget,etarget,'c.','markersize',ms/2);
    plot([nlookahead,ntarget],[elookahead,etarget],'c','linewidth',lw)


    %  --determine the optimal curvature rate for this target

    [kpmax,kpmin] = get_kp_limit(R,kcar);

    kpdes = get_opt_curvature_rate(kcar, kpmax, kpmin, R, ...
        psi, ncar, ntarget, ecar, etarget, maxiter, tol);

    [~,~,L] = clothoid_endpoint(ncar,ecar,psi,kcar,kpdes,R,maxiter,tol);


    %  --generate and plot the clothoid at this waypoint

    [nclothoid,eclothoid] = gen_clothoid(ncar,ecar,psi,kcar,kpdes,L);
    if (exist('hc') == 1), delete(hc); end
    hc = plot(nclothoid,eclothoid,'color',[255 102 0]/255,'linewidth',3*lw);

    if zoomview
        %  --zoom in to the car
        xlim(NWP(n) + dx*[-1,1]);
        ylim(EWP(n) + dy*[-1,1]);
    end
    drawnow
    pause(0.1);  % uncomment this line to slow the animation down
end



