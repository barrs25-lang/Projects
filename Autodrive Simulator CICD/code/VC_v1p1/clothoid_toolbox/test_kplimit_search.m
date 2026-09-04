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
%  This script evaluates the kplimit_search.m function with randomized
%  inputs and plots the resulting extremal clothoids.
%
%  This script calls the following functions:
%       kplimit_search.m
%       clothoid_endpoint.m
%       gen_clothoid.m

clear, clc

%  --define randomized inputs

n0 = 30*(2*rand(1)-1);  % northing coordinate (m)
e0 = 30*(2*rand(1)-1);  % easting coordinate (m)
p0 = 2*pi*rand(1);      % heading coordinate (rad)

R = 3 + 22*rand(1);     % target radius (m)

k0 = 0.1 * (2*rand(1) - 1);  % initial curvature (rad/m)

kprange = 0.3*[-1 1];   % range of curvature rates to search (rad/m/m)

tol = 0.001;  % error tolerance on radial target (m)
maxiter = 500;  % maximum number of iterations in final endpoint step


%  --generate and plot a reference circle

theta = linspace(0,2*pi,1000);
cx = n0 + R*cos(theta);
cy = e0 + R*sin(theta);

figure(1)
clf
plot(cx,cy,'color',0.5*[1 1 1]);
hold on
plot(n0,e0,'g.','markersize',20)
grid on
axis equal
xlabel('North (m)')
ylabel('East (m)')
title(sprintf('R = %g (m)',R))


%  --evaluate the kp limits

[kpmax,kpmin] = kplimit_search(R,k0,kprange,tol);
fprintf('kpmax=%g, kpmin=%g\n',kpmax,kpmin);

ms = 10;


%  --plot the results for this randomized input data set

figure(1)
[nf,ef,L] = clothoid_endpoint(n0,e0,p0,k0,kpmax,R,maxiter,tol);
[north,east] = gen_clothoid(n0,e0,p0,k0,kpmax,L);
h1 = plot(north,east,'color','b','linewidth',2);
plot(nf,ef,'bo','markersize',ms,'linewidth',2);
e1 = abs(R - sqrt( (nf-n0)^2 + (ef-e0)^2 ));

[nf,ef,L] = clothoid_endpoint(n0,e0,p0,k0,kpmin,R,maxiter,tol);
[north,east] = gen_clothoid(n0,e0,p0,k0,kpmin,L);
h2 = plot(north,east,'color','r','linewidth',2);
plot(nf,ef,'ro','markersize',ms,'linewidth',2);
e2 = abs(R - sqrt( (nf-n0)^2 + (ef-e0)^2 ));

legend([h1,h2],sprintf('kpmax = %g (rad/m), err = %g (m)',kpmax,e1), ...
    sprintf('kpmin = %g (rad/m), err = %g (m)',kpmin,e2))


