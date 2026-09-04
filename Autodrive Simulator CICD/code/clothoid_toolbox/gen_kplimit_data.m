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
%  This script evaluates clothoids over a grid of target radii and
%  curvature values to determine a lookup table for the min and max
%  curvature rates for each grid condition.
%
%  This script calls the following functions:
%       kplimit_search.m
%       printpng.m
%
%  This script will generate the following outputs:
%       kplimdata.mat (resulting lookup table data)
%       curvature_rate_limits_2D.png
%       curvature_rate_limits_3D.png

clear, clc

%  --define the grid of R and kp values

NR = 49;
Rgrid = linspace(3,25,NR);  % target radius (m)

NK = 11;
Kgrid = linspace(-0.2,0.2,NK);  % initial curvature kappa (rad/m)

KPpos = zeros(NK,NR);  % maximum curvature rate kappa-prime (rad/m/m)
KPneg = zeros(NK,NR);  % minimum curvature rate kappa-prime (rad/m/m)

kprange = [-0.4,0.4];  % search range for curvature rate (rad/m/m)
tol = 0.001;  % error tolerance (m)

for k = 1:NK
    for r = 1:NR
        [kpmax,kpmin] = kplimit_search(Rgrid(r),Kgrid(k),kprange,tol);
        KPpos(k,r) = kpmax;
        KPneg(k,r) = kpmin;
    end
end

save kplimdata.mat KPpos KPneg Rgrid Kgrid


%%  --plot the KP limit data as a 2D plot

cpos = autumn(NK);
cneg = winter(NK);

figure(1)
clf
for n = 1:NK
    h(n) = plot(Rgrid,KPpos(n,:),'.-','color',cpos(n,:));
    hold on
    h(n+NK) = plot(Rgrid,KPneg(n,:),'.-','color',cneg(n,:));
end
grid on
ylabel('Curvature Rate Limit \kappa'' [rad/m^2]')
xlabel('Target Distance [m]')
xlim([min(Rgrid),max(Rgrid)])
title('Curvature Rate Limits vs. Target Distance')
printpng('curvature_rate_limits_2D.png',[12,10]);


%%  --plot the KP limit data as a 3D surface plot

figure(2)
clf
hp = surf(Rgrid,Kgrid,KPpos);
hold on
hn = surf(Rgrid,Kgrid,KPneg);
grid on
colormap(jet)
set(hp,'facealpha',0.5,'facecolor','interp')
set(hn,'facealpha',0.5,'facecolor','interp')
xlabel('Target Distance [m]')
ylabel('Initial Curvature \kappa_0 [rad/m]')
zlabel('Curvature Rate Limit \kappa'' [rad/m^2]')
printpng('curvature_rate_limits_3D.png',[12,10]);


