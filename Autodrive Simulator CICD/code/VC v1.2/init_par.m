%  init_par.m
%
%  This script defines a parameter vector (par) for the Vehicle Control
%  bench simulation.  This script also generates a .yaml file for loading
%  these parameters into ROS.
%
%  Note:  This script will automatically run init_car.m if it has not
%         already been run!

%  Last Updated:  1/25/2023

if (exist('car') == 0), init_car; end


%  --define the struct data array
%  DO NOT CHANGE THE ORDER OF THE SIGNALS IN THIS ARRAY!!!

field_value_string = {
    {'ts', 1/fs, 'Sample Period (seconds)'}
    {'max_drv_torque', 22534/10, 'Max Drive Torque (Nm)'}
    {'min_drv_torque',     0, 'Min Drive Torque (Nm)'}
    {'max_brk_torque',    -0, 'Max Brake Torque (Nm)'}
    {'min_brk_torque',-1000, 'Min Brake Torque (Nm)'}
    {'max_delta_torque', 100, 'Max Delta Torque (Nm)'}
    {'max_delta_brake',   100, 'Max Delta Brake (Nm)'}
    {'max_speed', 11.176, 'Max Vehicle Speed (m/s)'}
    {'min_speed',     0, 'Min Vehicle Speed (m/s)'}
    {'max_alat', 3.5, 'Max Lateral Acceleration (m/s/s)'}
    {'max_steer', car.MAX_STEER, 'Max Steering Command (deg)'}
    {'min_steer', car.MIN_STEER, 'Min Steering Command (deg)'}
    {'max_delta_steer', 100, 'Max delta steer (deg/sample)'}
    {'kp_speed', 3979, 'Proportional gain for Speed Control'}
    {'ki_speed', 25, 'Integral gain for Speed Control'}
    {'kd_speed', 95, 'Derivative gain for Speed Control'}
    {'fbreak_diff', 20, '1st order differentiator break freq. (Hz)'}
    {'kff_gain', car.MCAR * g * car.Rtire, 'Feedforward gain for pitch'}
    {'fbreak_fflpf', 5, '1st order LPF break freq. (Hz) for pitch'}
    {'fz_lead', 3.4, 'Zero frequency (Hz) for lead filter'}
    {'zz_lead', 1.7, 'Zero damping ratio for lead filter'}
    {'fp_lead', 2.5*3.4, 'Pole frequency (Hz) for lead filter'}
    {'kappa2theta', 2542.91, 'theta(deg) = kappa2theta * kappa(rad/m)'}
    {'fbreak_vref', 0.5, '1st order LPF break freq. (Hz) for Vref & Alat'}
    {'max_lookahead', 2.5, 'Max Lookahead Distance (m)'}
    {'min_lookahead', 1, 'Min Lookahead Distance (m)'}
    {'lookahead_ratelimit', 0.5, 'Lookahead Rate Limit (m/sample)'}
    {'vdead', 0.1, 'Dead Zone Velocity (m/s)'}
    {'wT', 2*pi*2.8648, 'Torque model cutoff frequency (rad/s)'}
    {'zT', 0.9, 'Torque model damping ratio (nondim)'}
    {'wB', 2*pi*1.512, 'Brake model cutoff frequency (rad/s)'}
    {'zB', 1.1, 'Brake model damping ratio (nondim)'}
    };

%     {'kp_speed', 3979/10, 'Proportional gain for Speed Control'}
%     {'ki_speed', 25/10, 'Integral gain for Speed Control'}
%     {'kd_speed', 95/2, 'Derivative gain for Speed Control'}


%  --assemble the actual parameter structure

N = length(field_value_string);
clear s par par_bus
for n = 1:N
    nextfield = field_value_string{n}{1};
    nextval = field_value_string{n}{2};
    nextstr = field_value_string{n}{3};

    s{n,1} = nextstr;

    %  --enter the next value into the parameter struct
    temp = sprintf('par_bus.%s = %14.7f;',nextfield,nextval);
    eval(temp)

    %  --enter the next value into the parameter vector
    temp = sprintf('par(%d,1) = %14.7f;',n,nextval);
    eval(temp)    
end


%%  --write out the .yaml file for loading configuration parameters

%  --open the file with todays' date in the filename

dt = string(datetime('today','format','MM_dd_yyyy'));
fid = fopen(sprintf('VC_params_%s.yaml',dt),'w');
fmt = '%14.7f';

fprintf(fid,'# This yaml file defines the VC (Vehicle Control) parameter vector.\n');
fprintf(fid,'#\n');
fprintf(fid,'# Generation date:  %s\n',datetime);
fprintf(fid,'#\n');

varname = 'par';
L = length(varname);
leader = blanks(L+3);

fields = fieldnames(par_bus);
N = length(fields);

maxlen = 0;
for n = 1:N
    maxlen = max(maxlen,length(fields{n}));
end
pad = blanks(maxlen);


for n = 1:N
    data = eval(strcat('par_bus.',fields{n},';'));
    fieldstr = [fields{n},pad];
    if (n == 1)
        fprintf(fid,'%s: [%s,  # %s  %2d) %s\n', ...
            varname,sprintf(fmt,data),fieldstr(1:maxlen),n-1,s{n});
    elseif (n == N)
        fprintf(fid,'%s%s]  # %s  %2d) %s\n', ...
            leader,sprintf(fmt,data),fieldstr(1:maxlen),n-1,s{n});
    else
        fprintf(fid,'%s%s,  # %s  %2d) %s\n', ...
            leader,sprintf(fmt,data),fieldstr(1:maxlen),n-1,s{n});
    end
end
fprintf(fid,'#\n');

fprintf(fid,'# end-of-parameters\n');

clear field_value_string n N nextfield nextval nextstr s temp
clear dt fid fmt varname L leader fields maxlen pad data fieldstr
