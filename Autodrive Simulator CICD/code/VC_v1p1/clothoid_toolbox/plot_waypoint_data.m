function plot_waypoint_data(NWP, EWP, HWP, VWP, KWP, ZWP, SWP, coursename, ...
    vmax, max_along, max_alat, plotfilename)
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
%  This function generates a plot of the waypoint data that also includes
%  the computed reference velocity waypoints and the corresonding lateral
%  and longitudinal accelerations.
%
%  This function calls the following functions:
%       printpng.m

%  --define a colormap for the breadcrumb markers on the plots

NPTS = 20;  % change this for more or less markers
switch NPTS
    case 6
        c = [0.8 0 0; 1 0.65 0; 0.8 0.8 0; 0 0.8 0; 0 0 0.8; 0.75 0.25 0.75];
    otherwise
        c = turbo(NPTS);
end
lw = 1;  ms = 6;  gray = 0.7*[1 1 1];


%  --generate the figure

figure(1), clf


%  --plot the 3D birdseye view

subplot(2,2,1)
plot3(EWP,NWP,ZWP,'k.-','markersize',ms,'linewidth',lw);
axis equal
grid on
hold on
M = floor(length(NWP) / (NPTS-1));
for n = 1:NPTS
    if (n < NPTS)
        k = min((n-1)*M + 1,length(NWP));
    else
        k = length(NWP);
    end
    plot3(EWP(k),NWP(k),ZWP(k),'.','color',c(n,:),'markersize',4*ms);
end
ylabel('North (m)')
xlabel('East (m)')
title(coursename,'interpreter','none')


%  --plot the heading

subplot(4,2,5)
plot(SWP,HWP,'b.-','linewidth',lw,'markersize',ms, ...
    'color',0.7*[0 0 1])
grid on
hold on
M = floor(length(NWP) / (NPTS-1));
for n = 1:NPTS
    if (n < NPTS)
        k = min((n-1)*M + 1,length(NWP));
    else
        k = length(NWP);
    end
    plot(SWP(k),HWP(k),'.','color',c(n,:),'markersize',4*ms);
end
set(gca,'ytick',45*[-8:8]);
xlim([min(SWP),max(SWP)])
ylim([0,360])
xlabel('S (m)')
ylabel('Heading:  HWP (deg)')


%  --plot the elevation

subplot(4,2,7)
plot(SWP,ZWP,'r.-','markersize',ms,'linewidth',lw);
grid on
hold on
M = floor(length(NWP) / (NPTS-1));
for n = 1:NPTS
    if (n < NPTS)
        k = min((n-1)*M + 1,length(NWP));
    else
        k = length(NWP);
    end
    plot(SWP(k),ZWP(k),'.','color',c(n,:),'markersize',4*ms);
end
xlim([min(SWP),max(SWP)])
xlabel('S (m)')
ylabel('Elevation:  ZWP (m)')


%  --plot the curvature

subplot(4,2,2)
plot(SWP,KWP,'.-','linewidth',lw,'markersize',ms,'color',0.6*[0 1 0])
grid on
hold on
M = floor(length(NWP) / (NPTS-1));
for n = 1:NPTS
    if (n < NPTS)
        k = min((n-1)*M + 1,length(NWP));
    else
        k = length(NWP);
    end
    plot(SWP(k),KWP(k),'.','color',c(n,:),'markersize',4*ms);
end
xlim([min(SWP),max(SWP)])
xlabel('S (m)')
ylabel('Curvature:  KWP (rad/m)')


%  --plot the velocity

subplot(4,2,4)
plot(SWP,VWP,'.-','linewidth',lw,'markersize',ms,'color',0.6*[1 0 1])
hold on
yline(vmax,'color',gray,'linewidth',2*lw);
grid on
ylim([0,1.02*vmax])
M = floor(length(NWP) / (NPTS-1));
for n = 1:NPTS
    if (n < NPTS)
        k = min((n-1)*M + 1,length(NWP));
    else
        k = length(NWP);
    end
    plot(SWP(k),VWP(k),'.','color',c(n,:),'markersize',4*ms);
end
xlim([min(SWP),max(SWP)])
xlabel('S (m)')
ylabel('Velocity:  VWP (m/s)')
yl = ylim * 2.23694;  % mph
yyaxis right
ylim(yl);
set(gca,'ycolor',[0 0 0]);
ylabel('(miles/hour)')


%  --plot the longitudinal acceleration

subplot(4,2,8)
along = VWP .* [diff(VWP) ./ diff(SWP); 0];
plot(SWP,along,'.-','linewidth',lw,'markersize',ms,'color','c')
hold on
yline(max_along,'color',gray,'linewidth',2*lw);
yline(-max_along,'color',gray,'linewidth',2*lw);
ylim(max_along*1.02*[-1,1]);
grid on
M = floor(length(NWP) / (NPTS-1));
for n = 1:NPTS
    if (n < NPTS)
        k = min((n-1)*M + 1,length(NWP));
    else
        k = length(NWP);
    end
    plot(SWP(k),along(k),'.','color',c(n,:),'markersize',4*ms);
end
xlim([min(SWP),max(SWP)])
xlabel('S (m)')
ylabel('a_{long} (m/s^2)')
yl = ylim / 9.81;  % g
yyaxis right
ylim(yl);
set(gca,'ycolor',[0 0 0]);
ylabel('(g)')


%  --plot the lateral acceleration

subplot(4,2,6)
alat = VWP.^2 .* KWP;
plot(SWP,alat,'.-','linewidth',lw,'markersize',ms,'color','m')
hold on
yline(max_alat,'color',gray,'linewidth',2*lw);
yline(-max_alat,'color',gray,'linewidth',2*lw);
ylim(max_alat*1.02*[-1,1]);
grid on
M = floor(length(NWP) / (NPTS-1));
for n = 1:NPTS
    if (n < NPTS)
        k = min((n-1)*M + 1,length(NWP));
    else
        k = length(NWP);
    end
    plot(SWP(k),alat(k),'.','color',c(n,:),'markersize',4*ms);
end
xlim([min(SWP),max(SWP)])
xlabel('S (m)')
ylabel('a_{lat} (m/s^2)')
yl = ylim / 9.81;  % g
yyaxis right
ylim(yl);
set(gca,'ycolor',[0 0 0]);
ylabel('(g)')


%  --save the plot

if (nargin > 11)
    printpng(plotfilename,[16,12]);
end

return

