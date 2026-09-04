function run_VC_bench(k)
%  This script sets up and runs a simulation of the autonomous vehicle
%  controller

%  Last Updated:   5/30 /2023



%  --setup the temporary path assignments
addpath clothoid_toolbox -begin
addpath waypoint_data -begin

savemovie = false;
saveplots = true;

workspace = getenv("GITHUB_WORKSPACE");
if isempty(workspace)
    workspace = pwd; % local fallback
end

repoRoot = fileparts(fileparts(pwd));
run_id = sprintf('case_%d', k);
results_root = fullfile(repoRoot, 'Results',run_id);
if ~exist(results_root, 'dir')
    mkdir(results_root);
end



%
%results_root = fullfile(pwd, 'Results', run_id);

%if saveplots
 %   if exist(results_root, 'dir') == 0
  %      mkdir(results_root);
   % end
%end

%if saveplots
    %  --check to see if the results folder exists
   % if (exist('results','dir') == 0)
        %  --results folder does not exists so create it
       % mkdir results
   % end
%end


%  --initialize the simulation parameters
run('init_par.m');


%  --load the waypoint data
%  testcase = enum input for selecting a waypoint data set
%     lanechange.mat                % 1
%     figure8.mat                   % 2
%     Zandvoort_clothoid.mat        % 3
%     Zandvoort.mat                 % 4
%     Yuma_clothoid.mat             % 5
%     Yuma.mat                      % 6
%     VIR_clothoid.mat              % 7
%     VIR.mat                       % 8
%     RoadAmerica_clothoid.mat      % 9
%     RoadAmerica.mat               % 10
%     PikesPeak_clothoid.mat        % 11
%     PikesPeak.mat                 % 12

TESTS = [k];


%  --loop through each testcase

vstart = 1;    % initial reference velocity (m/s)
vmax = 10;     % maximum allowable reference velocity (m/s)
vfinal = 0;    % final reference velocity (m/s)

max_alat = 3.25;   % maximum lateral acceleration (m/s/s)
max_along = 3.25;  % maximum longitudinal acceleration (m/s/s)

randrotate = false;  % Boolean for applying a randomly rotating data

for testcase = TESTS

    clear NWP EWP HWP VWP KWP ZWP SWP GWP WPT_MTX TFINAL sim

    [NWP,EWP,HWP,VWP,KWP,ZWP,SWP,GWP,coursename] = get_waypoint_data( ...
        testcase, vstart, vmax, vfinal, max_alat, max_along, randrotate);

    NUM_WP = length(NWP);
    WPT_MTX = [NWP,EWP,HWP,VWP,KWP,SWP];


    %  --compute and display simulation metrics prior to running

    TFINAL = estimate_sim_duration(VWP,SWP);

    fprintf('Course name:                      %s\n',coursename);
    fprintf('Total Number of Waypoints:        %g\n',NUM_WP);
    fprintf('Total Path Length:                %g (m)\n',SWP(end));
    fprintf('Total Elevation Change:           %g (m)\n',max(ZWP)-min(ZWP));
    fprintf('Total estimated Simulation Time:  %g (s)\n',TFINAL);
    fprintf('Average Speed:                    %g (m/s)\n',SWP(end)/TFINAL);
    fprintf('Average VWP:                      %g (m/s)\n',mean(VWP));
    fprintf('Minimum WP Separation:            %g (m)\n',min(diff(SWP)));
    fprintf('Average WP Separation:            %g (m)\n',mean(diff(SWP)));
    fprintf('Maximum WP Separation:            %g (m)\n',max(diff(SWP)));


    %  --define the vehicle initial conditions

    north0 = NWP(1);   % initial northing coordinate (meters)
    east0 = EWP(1);    % initial easting coordinate (meters)
    psi0 = (pi/180)*HWP(1);     % initial heading (radians)
    vcg0 = 0;          % initial forward velocity (meters/sec)

    % return

    %  --run the closed-loop simulation

    assignin('base','vcg0',vcg0);
    assignin('base','NUM_WP',NUM_WP);
    assignin('base','EWP',EWP);
    assignin('base', 'east0', EWP(1));
    assignin('base', 'WPT_MTX', WPT_MTX);
    assignin('base', 'ZWP', ZWP);
    assignin('base', 'north0', NWP(1));
    assignin('base', 'psi0', (pi/180)*HWP(1));
    assignin('base', 'NWP', NWP);
    
    tstart = tic;
    sim('VC_v1p1.slx',TFINAL)
    toc(tstart)

    clear tstart K

    repack_simdata

    if saveplots
    %     %  --create the folder for test results
    %     folder = sprintf('./results/testcase%g/',testcase);
    %     if (exist(folder,'dir') == 0)
    %         %  --subfolder does not exist so create it
    %         mkdir(folder);
    %     end
    % 
    %     %  --save simulation results to the specific testcase folder
    %     save(strcat(folder,'simdata.mat'))
    % else
    %  --save simulation results to the specific testcase folder
        %caseNum = getenv('MATRIX_CASE');  % GitHub Actions env variable
        %resultsFolder = fullfile(pwd,'..','..','Results',sprintf('case_%s',caseNum));
        %if ~exist(resultsFolder,'dir')
        %    mkdir(resultsFolder);
        %end
        save(fullfile(results_root,'simdata.mat')); % <-- must save here
        %  --just save a local copy of the simulation results
        %
        % if ~exist(fullfile(results_root,'simdata.mat'),'file')
        %     % simdata = [];
        %     if ~exist(results_root,'dir')
        %         mkdir(results_root);
        %     end
        %     save(fullfile(results_root,'simdata.mat'))
        % end
    end

    
    %%  --plot the simulation response data
    plot_sim_results
end

%%  --remove the temporary path assignments
rmpath clothoid_toolbox
rmpath waypoint_data

end
