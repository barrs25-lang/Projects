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
%  This script validates the get_opt_curvature_rate.m function by allowing
%  the user to select a number of target locations using the graphics 
%  cursor and then plotting the resulting clothoid curve.
%
%  This script calls the following functions:
%       get_opt_curvature_rate.m
%       draw_car.m
%       get_kp_limit.m
%       clothoid_endpoint.m
%       gen_clothoid.m

%  S. C. Southward      1/16/2023

clear, clc
clear gen_clothoid
clear get_opt_curvature_rate


%  --define the constant parameters for each run

Ecar = 0;  % easting coordinate of car (m)
Ncar = 0;  % northing coordinate of car (m)

psi_deg = randi(180*[-1,1]);
psi = (pi/180)*psi_deg;  % heading angle of car (rad)

kcar = randi(500*[-1,1])/10000;  % curvature of vehicle path (rad/m)

NLOOPS = 10;  % total number of user-selectable target locations

ms = 50;  lw = 2;

tol = 0.001;
maxiter = 5;  % change to 5 for real-time implementation

lookahead = 5;  % lookahead vector (m)

dp = (pi/180)*60;
theta = linspace(psi-dp,psi+dp,1000);
circx = cos(theta);
circy = sin(theta);


%  --draw the initial figure

figure(1)
clf
clear draw_car
draw_car(Ncar,Ecar,psi,lookahead,lw)
hold on
grid on
xlim(Ncar+20*[-1,1]);
axis equal
ylabel('East (m)')
xlabel('North (m)')


%  --loop through each user selection

for n = 1:NLOOPS
    %  --get the next target position from the user

    [Ntarget,Etarget] = ginput(1);
    plot(Ntarget,Etarget,'b.','markersize',20);


    %  --draw a circle at the target radius

    R = sqrt( (Ntarget-Ncar)^2 + (Etarget-Ecar)^2 );
    plot(R*circx,R*circy,'color',0.5*[1 1 1],'linewidth',1);


    %  --get the max and min curvature rates for this condition

    [kpmax,kpmin] = get_kp_limit(R,kcar);


    %  --compute and plot the extremal clothoids

    [nf,ef,L] = clothoid_endpoint(Ncar,Ecar,psi,kcar,kpmin,R,maxiter,tol);
    plot(nf,ef,'c.','markersize',20)
    [nt,et] = gen_clothoid(Ncar,Ecar,psi,kcar,kpmin,L);
    plot(nt,et,'c--','linewidth',lw);

    [nf,ef,L] = clothoid_endpoint(Ncar,Ecar,psi,kcar,kpmax,R,maxiter,tol);
    plot(nf,ef,'m.','markersize',20)
    [nt,et] = gen_clothoid(Ncar,Ecar,psi,kcar,kpmax,L);
    plot(nt,et,'m--','linewidth',lw);


    %  --compute the optimal curvature rate for these conditions

    kpdes  = get_opt_curvature_rate(kcar,kpmax,kpmin,R, ...
        psi,Ncar,Ntarget,Ecar,Etarget,maxiter,tol);  % (rad/m/m)


    %  --calculate the optimal clothoid

    figure(1)
    [~,~,L] = clothoid_endpoint(Ncar,Ecar,psi,kcar,kpdes,R,maxiter,tol);
    [Nt,Et] = gen_clothoid(Ncar,Ecar,psi,kcar,kpdes,L);
    plot(Nt,Et,'b','linewidth',lw);


    title(strcat('$$\kappa''_{desired} = ',num2str(kpdes), ...
        '\:(',int2str(n),'/',int2str(NLOOPS),')$$'), ...
        'interpreter','latex')

    
    drawnow
end



