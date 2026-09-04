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
%  This script plots the waypoint matfiles.
%
%  This script calls the following functions:
%       LineNormals2D.m
%       subplotfill.m
%       printpng.m

%  Last Updated:  3/1/2023

clear, clc

saveplot = false;

waypoint_data_path = './waypoint_data/';

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
NF = length(files);

if saveplot
    %  --check to see if the plots folder exists
    if (exist('waypoint_plots','dir') == 0)
        %  --create the plots folder
        mkdir('waypoint_plots')
    end
end


%  --loop through each matfile

for n = 1:NF
    fprintf('%g of %g\n',n,NF);

    %  --load the matfile

    nextfile = files{n};
    clear EWP NWP HWP KWP SWP ZWP coursename

    load(strcat(waypoint_data_path,nextfile));


    %  --compute the normals at each waypoint

    nrmls = LineNormals2D([EWP,NWP]);  % unity length
    en = nrmls(:,1);  % easting component of normal
    nn = nrmls(:,2);  % northing component of normal


    %  --plot the waypoints in birdseye view

    figure(n), clf

    NPTS = 20;  % change this for more or less markers
    M = floor(length(NWP) / (NPTS-1));
    switch NPTS
        case 6
            c = [0.8 0 0; 1 0.65 0; 0.8 0.8 0; 0 0.8 0; 0 0 0.8; 0.75 0.25 0.75];
        otherwise
            c = turbo(NPTS);
    end
    lw = 1;  ms = 6;  gray = 0.7*[1 1 1];

    dx = max(EWP) - min(EWP);
    dy = max(NWP) - min(NWP);
    ds = sqrt(dx^2 + dy^2);  % use this to scale normals

    clear h
    subplotfill(1,2,1);
    h(1) = plot(EWP,NWP,'b.-');
    grid on
    hold on
    h(2) = plot(EWP(1),NWP(1),'cx','markersize',20,'linewidth',3);
    plot([EWP EWP+ds*KWP.*en]',[NWP NWP+ds*KWP.*nn]', ...
        'color',0.5*[1 1 1]);
    xlabel('EWP (m)')
    ylabel('NWP (m)')
    title(files{n},'interpreter','none')
    axis equal
    for m = 1:NPTS
        if (m < NPTS)
            k = min((m-1)*M + 1,length(NWP));
        else
            k = length(NWP);
        end
        plot(EWP(k),NWP(k),'.','color',c(m,:),'markersize',4*ms);
    end
    legend(h,'Waypoints','Start','location','best');

    clear h
    subplotfill(3,2,2);
    h = plot(SWP,KWP,'linewidth',2);
    grid on
    hold on
    xlim([min(SWP),max(SWP)])
    xlabel('Path Position (m)')
    ylabel('Curvature:  \kappa (rad/m)')
    for m = 1:NPTS
        if (m < NPTS)
            k = min((m-1)*M + 1,length(NWP));
        else
            k = length(NWP);
        end
        plot(SWP(k),KWP(k),'.','color',c(m,:),'markersize',4*ms);
    end

    subplotfill(3,2,4);
    plot(SWP,HWP,'color',[0 0.6 0],'linewidth',2)
    grid on
    hold on
    xlim([min(SWP),max(SWP)])
    ylim([0,360])
    set(gca,'ytick',45*[-8:8])
    xlabel('Path Position (m)')
    ylabel('Heading:  \psi (deg)')
    for m = 1:NPTS
        if (m < NPTS)
            k = min((m-1)*M + 1,length(NWP));
        else
            k = length(NWP);
        end
        plot(SWP(k),HWP(k),'.','color',c(m,:),'markersize',4*ms);
    end

    subplotfill(3,2,6);
    plot(SWP,ZWP,'color',[0 0.6 0.6],'linewidth',2)
    grid on
    hold on
    xlim([min(SWP),max(SWP)])
    xlabel('Path Position (m)')
    ylabel('Elevation (m)')
    for m = 1:NPTS
        if (m < NPTS)
            k = min((m-1)*M + 1,length(NWP));
        else
            k = length(NWP);
        end
        plot(SWP(k),ZWP(k),'.','color',c(m,:),'markersize',4*ms);
    end

    drawnow


    if saveplot
        %  --save the plot

        k = strfind(nextfile,'.mat');
        plotfile = strcat('./waypoint_plots/',nextfile(1:k-1),'.png');
        printpng(plotfile,[16,12]);
    end
end

clear M NF c den num ds dx dy en nn gray h k lw m ms n nrmls KWP0 nextfile
