%  This script plots the results of the VC_vxpx simulation

%  Last modified 5/30/2023

set(0,'defaultlinelinewidth',2);
lw = 2;
gray = 0.5*[1 1 1];
plotsize = [18,12];

%  1 = timing & configuration signals
%  2 = motion signals (velocity, acceleration, jerk)
%  3 = control signals (torque, brake, steer)
%  4 = force signals
%  5 = pose signals
%  6 = birdseye (static)
%  7 = torque, brake, velocity
%  8 = birdseye (animation)
%  9 = waypoint index

plottype = [1:9];

for p = 1:length(plottype)
    switch plottype(p)
        case 1  % plot the configuration signals
            figure(plottype(p))
            clf
            sigs = {'mode_select','action_select','AV_start_request', ...
                'AV_mode_active','VC_state','car_is_stopped', ...
                'speed_mode','steer_mode','path_mode','wpt_status_change', ...
                'end_of_data'};
            yt = [4,3,2,2,4,2,3,3,2,2,2];
            ys = {'ModeSel: Manual','ModeSel: Auto','ModeSel: Speed','ModeSel: Steer', ...
                'ActionSel: Manual','ActionSel: Init Auto','ActionSel: Run Hot', ...
                'AVStReq: False','AVStReq: True','AVactive: False','AVactive: True', ...
                'VCState: Manual','VCState: Init Auto','VCState: RDY Brake','VCState: Run Hot', ...
                'CarStop: False','CarStop: True', 'SpeedMode: Zero Torque','SpeedMode: Max Brake','SpeedMode: Internal', ...
                'SteerMode: Zero Steer','SteerMode: Hold Steer','SteerMode: Internal', ...
                'PathMode: Disable','PathMode: Enable', 'WPTchange: False','WPTchange: True', ...
                'EOD: False','EOD: True'};
            NS = length(sigs);
            c = turbo(NS);
            for n = 1:NS
                offset = sum(yt(1:n-1));
                stairs(sim.t,offset+eval(strcat('sim.',sigs{n})),'color',c(n,:), ...
                    'linewidth',lw);
                hold on
            end

            grid on
            M = length(ys);
            set(gca,'ylim',[0,M],'ytick',[0:M-1])
            xlim([min(sim.t),max(sim.t)])
            set(gca,'yticklabel',ys)
            xlabel('Time (sec)')
            title(coursename)
            drawnow

            if saveplots
                printpng(fullfile(results_root,'timing.png'),plotsize);
            end

        case 2  % plot the motion signals
            figure(plottype(p))
            clf
            sigs = {'vref','vdes','vcar', ...
                'alat','along','pitch','jlat','jlong'};
            yl = {'m/s','m/s','m/s','m/s^2','m/s^2','rad','m/s^3','m/s^3'};
            NS = length(sigs);
            for n = 1:NS
                labstr{n} = {sigs{n},yl{n}};
            end
            c = turbo(NS);

            %  --velocities
            subplotfill(5,1,1);
            for n = 1:3
                plot(sim.t,eval(strcat('sim.',sigs{n})),'color',c(n,:))
                hold on
                grid on
            end
            legend(sigs{1:3},'location','best','orientation','horizontal')
            ylabel(yl{1})
            title(coursename)
            xlim([min(sim.t),max(sim.t)])


            %  --everything else
            for n = 5:NS
                subplotfill(5,1,n-3);
                plot(sim.t,eval(strcat('sim.',sigs{n})),'color',c(n,:));
                ylabel(labstr{n})
                grid on
                xlim([min(sim.t),max(sim.t)])
            end
            xlabel('Time (sec)')
            drawnow

            if saveplots
                printpng(fullfile(results_root,'motion.png'),plotsize);
            end


        case 3  % plot the control signals
            figure(plottype(p))
            clf
            sigs = {'torque_cmd','torque_fbk','brake_cmd','brake_fbk', ...
                'steer_cmd','steer_fbk'};
            yl = {'Nm','Nm','Nm','Nm','deg','deg'};
            NS = length(sigs);
            c = turbo(NS);
            subplotfill(2,1,1);
            count = 1;
            for n = 1:4
                hh(n) = plot(sim.t,eval(strcat('sim.',sigs{n})),'color',c(n,:));
                hold on
            end
            grid on
            ylabel('Torque Controls [Nm]')
            title(coursename)
            xlim([min(sim.t),max(sim.t)])
            legend(hh,sigs(1:4),'interpreter','none','orientation','horizontal')

            subplotfill(2,1,2);
            clear hh
            for n = 5:6
                hh(n-4) = plot(sim.t,eval(strcat('sim.',sigs{n})),'color',c(n,:));
                hold on
            end
            grid on
            ylabel('Speed Controls [deg]')
            xlabel('Time (sec)')
            xlim([min(sim.t),max(sim.t)])
            legend(hh,sigs(5:6),'interpreter','none','orientation','horizontal')
            drawnow

            if saveplots
                printpng(fullfile(results_root,'control.png'),plotsize);
            end

        case 4  % plot the force signals
            figure(plottype(p))
            clf
            sigs = {'Fgrade','Fsum','Fnet','Fdamp'};
            yl = {'N','N','N','N'};
            NS = length(sigs);
            c = turbo(NS);
            for n = 1:NS
                subplotfill(4,1,n);
                plot(sim.t,eval(strcat('sim.',sigs{n})),'color',c(n,:));
                ylabel({sigs{n},'[N]'})
                xlim([min(sim.t),max(sim.t)])
                grid on
            end
            xlabel('Time (sec)')
            title(coursename)
            drawnow

            if saveplots
                printpng(fullfile(results_root,'forces.png'),plotsize);
            end


        case 5  % plot the pose signals
            figure(plottype(p))
            clf
            sigs = {'north_target','northcg','northr', ...
                'east_target','eastcg','eastr', ...
                'xterr','psi'};
            yl = {'m','m','m','m','m','m','m','rad'};
            NS = length(sigs);
            c = turbo(NS);

            subplotfill(4,1,1);
            for n = 1:3
                hh(n) = plot(sim.t,eval(strcat('sim.',sigs{n})),'color',c(n,:));
                hold on
            end
            ylabel({'Northing','[m]'})
            title(coursename)
            grid on
            xlim([min(sim.t),max(sim.t)])
            legend(hh,sigs(1:3),'orientation','horizontal','location','best', ...
                'interpreter','none')

            subplotfill(4,1,2);
            clear hh
            for n = 4:6
                hh(n-3) = plot(sim.t,eval(strcat('sim.',sigs{n})),'color',c(n,:));
                hold on
            end
            ylabel({'Easting','[m]'})
            grid on
            xlim([min(sim.t),max(sim.t)])
            legend(hh,sigs(4:6),'orientation','horizontal','location','best', ...
                'interpreter','none')

            subplotfill(4,1,3);
            plot(sim.t,eval(strcat('sim.',sigs{7})),'color',c(7,:));
            ylabel({'Cross Track Error','[m]'})
            grid on
            xlim([min(sim.t),max(sim.t)])

            subplotfill(4,1,4);
            plot(sim.t,eval(strcat('sim.',sigs{8})),'color',c(8,:));
            ylabel({'Heading','[deg]'})
            grid on
            xlim([min(sim.t),max(sim.t)])
            xlabel('Time (sec)')
            drawnow

            if saveplots
                printpng(fullfile(results_root,'pose.png'),plotsize);
            end


        case 6  % plot the birdseye signals
            figure(plottype(p))
            clf
            sigs = {'northr','eastr'};
            yl = {'m','m'};
            h(1) = plot(NWP,EWP,'.-','markersize',10,'linewidth',lw,'color',gray);
            hold on
            h(2) = plot(sim.northr,sim.eastr,'b.-','markersize',5,'linewidth',lw/2);
            xlabel('North (m)')
            ylabel('East (m)')
            title(coursename)
            grid on
            axis equal
            legend(h,'Waypoint Path','Vehicle Path')
            drawnow

            if saveplots
                printpng(fullfile(results_root,'birdseye.png'),plotsize);
            end

        case 7  % torque, brake, velocity

            figure(plottype(p))
            clf
            sigs = {'torque_cmd','torque_fbk','brake_cmd','brake_fbk', ...
                'vref','vdes','vcar'};
            NS = length(sigs);
            c = turbo(NS);
            subplotfill(2,1,1);
            for n = 1:4
                hh(n) = plot(sim.t,eval(strcat('sim.',sigs{n})),'color',c(n,:));
                hold on
            end
            grid on
            ylabel('Torque [Nm]')
            title(coursename)
            xlim([min(sim.t),max(sim.t)])
            legend(hh,sigs(1:4),'interpreter','none','orientation','horizontal')

            subplotfill(2,1,2);
            clear hh
            for n = 5:7
                hh(n-4) = plot(sim.t,eval(strcat('sim.',sigs{n})),'color',c(n,:));
                hold on
            end
            grid on
            legend(sigs{5:7},'location','best')
            ylabel('Velocity [m/s]')
            xlabel('Time (sec)')
            xlim([min(sim.t),max(sim.t)])
            legend(hh,sigs(5:7),'interpreter','none','orientation','horizontal')
            drawnow

            if saveplots
                printpng(fullfile(results_root,'speed_control.png'),plotsize);
            end


        % case 8  % birdseye animation
        % 
        %     %  --initialize the figure
        %     clear draw_car
        %     ms = 20;
        %     lw = 1;
        % 
        %     figure(plottype(p))
        %     clf
        % 
        %     %  --prepare for saving a movie
        %     if savemovie
        %         img = [];
        %         FX = 640;  FY = 480;
        %         set(gcf,'position',[21        1096        FX         FY])
        %         moviefile = strcat(folder,'birdseye_animate.mp4');
        %         vidObj = VideoWriter(moviefile,'MPEG-4');
        %         vidObj.Quality = 100;
        %         vidObj.FrameRate = 40;
        %         open(vidObj);
        %     end
        % 
        %     %  --save local variables for the bar chart
        %     xterr = sim.xterr;
        %     verr = sim.vdes - sim.vcar;
        %     alat = sim.alat/100;
        %     along = sim.along/100;
        %     jlat = sim.jlat/100;
        %     jlong = sim.jlong/100;
        %     kpdes = sim.kpdes*10;
        %     kdes = sim.kdes;
        %     data = [xterr(1),verr(1),alat(1),along(1),jlat(1), ...
        %         jlong(1),kpdes(1),kdes(1)];
        %     hax0 = subplotfill(2,2,3);
        %     hbar = barh(data);
        %     set(hbar,'facecolor',[0 0.6 0]);
        %     xlim(0.4*[-1,1])
        %     ylim([0.5,length(data)+0.5])
        %     grid on
        %     set(hax0,'ytick',[1:length(data)])
        %     labstr = {'\epsilon_{cross track} [m]','\epsilon_{velocity} [m/s]','a_{lat}/100 [m/s^2]', ...
        %         'a_{long}/100 [m/s^2]','j_{lat}/100 [m/s^3]','j_{long}/100 [m/s^3]', ...
        %         '\kappa''_{desired}*10 [rad/m^2]','\kappa_{desired} [rad/m]'};
        %     set(hax0,'yticklabel',labstr)
        % 
        %     %  --generate the initial birdseye overview
        %     hax1 = subplotfill(2,2,1);
        %     plot(NWP,EWP,'b.-','markersize',ms/2,'linewidth',lw)
        %     grid on
        %     hold on
        %     axis equal
        %     xlabel('North (m)')
        %     ylabel('East (m)')
        %     title(coursename)
        % 
        %     %  --generate the initial zoomed birdseye view
        %     hax2 = subplotfill(1,2,2);
        %     plot(NWP,EWP,'b.-','markersize',ms/2,'linewidth',lw)
        %     grid on
        %     hold on
        %     axis equal
        %     xlabel('North (m)')
        %     ylabel('East (m)')
        %     title('Birdseye Local')
        % 
        %     xl = xlim; dx = xl(2) - xl(1);
        %     yl = ylim; dy = yl(2) - yl(1);
        %     ratio = dx/dy;
        % 
        %     dx = 7;
        %     dy = dx/ratio;
        % 
        %     LA = 5;
        %     north = sim.northr;
        %     east = sim.eastr;
        %     northt = sim.north_target;
        %     eastt = sim.east_target;
        %     psi = (pi/180)*sim.psi;
        %     nla = north(1) + LA * cos(psi(1));
        %     ela = east(1) + LA * sin(psi(1));
        %     axes(hax1);
        %     hdot = plot(north(1),east(1),'r.','markersize',ms);
        % 
        %     axes(hax2);
        %     draw_car(north(1),east(1),psi(1),5,lw);
        %     hx = plot([nla,northt(1)],[ela,eastt(1)],'g');
        % 
        %     %  --save the initial frame data
        %     if savemovie
        %         f = getframe(gcf);
        %         writeVideo(vidObj,f);
        %     end
        % 
        %     %  --loop through time steps (frames)
        %     for n = 1:10:length(sim.t)
        %         %  --update the data for this frame
        %         axes(hax0)
        %         data = [xterr(n),verr(n),alat(n),along(n),jlat(n),jlong(n),kpdes(n),kdes(n)];
        %         set(hbar,'ydata',data);
        % 
        %         axes(hax1);
        %         set(hdot,'xdata',north(n),'ydata',east(n));
        % 
        %         %  --draw the car at the next point
        %         axes(hax2);
        %         draw_car(north(n),east(n),psi(n),LA,lw);
        %         nla = north(n) + LA * cos(psi(n));
        %         ela = east(n) + LA * sin(psi(n));
        %         set(hx,'xdata',[nla,northt(n)],'ydata',[ela,eastt(n)])
        % 
        %         %  --zoom in to the car
        %         xlim(north(n) + dx*[-1,1]);
        %         ylim(east(n) + dy*[-1,1]);
        % 
        %         drawnow
        % 
        %         %  --save the frame data
        %         if savemovie
        %             f = getframe(gcf);
        %             writeVideo(vidObj,f);
        %         end
        %     end
        % 
        %     %  --close the final movie
        %     if savemovie, close(vidObj); end


        case 9  % plot the waypoint index signal
            figure(plottype(p))
            clf
            subplotfill(2,1,1);
            h = stairs(sim.t,sim.wpt_index,'color','b');
            set(h,'linewidth',lw)
            ylabel('Waypoint Index')
            xlim([min(sim.t),max(sim.t)])
            grid on
            xlabel('Time (sec)')
            title(coursename)

            subplotfill(4,1,3);
            h = stairs(sim.t,sim.wpt_status_change,'color','r');
            set(h,'linewidth',lw)
            ylabel('status change')
            xlim([min(sim.t),max(sim.t)])
            grid on
            xlabel('Time (sec)')

            subplotfill(4,1,4);
            h = stairs(sim.t,sim.end_of_data,'color','g');
            set(h,'linewidth',lw)
            ylabel('end of data')
            xlim([min(sim.t),max(sim.t)])
            grid on
            xlabel('Time (sec)')
            drawnow


            if saveplots
                printpng(fullfile(results_root,'wpt_index.png'),plotsize);
            end


    end

    figs = findall(0,'Type','figure');

    for i = 1:length(figs)
        fname = fullfile(results_root, sprintf('plot_%02d.png', i));
        exportgraphics(figs(i), fname, 'Resolution', 300);
    end

    close all
end

