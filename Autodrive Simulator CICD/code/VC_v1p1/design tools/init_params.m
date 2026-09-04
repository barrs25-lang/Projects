%  init_params.m
%
%  This script defines a parameter vector (par) for the Vehicle Control
%  bench simulation.  This script also generates a .yaml file for loading
%  these parameters into ROS.
%
%  Note:  This script assumes that init_car.m has already been run!

%  Last Updated:  1/25/2023

if (exist('car') == 0), init_car; end

fs = 100;

%  --define the struct data array
%  DO NOT CHANGE THE ORDER OF THE SIGNALS IN THIS ARRAY!!!

field_value_string = {
    {'ts', 1/fs, 'Sample Period (seconds)'}
    {'max_drv_torque', 22534, 'Max Drive Torque (Nm)'}
    {'min_drv_torque',     0, 'Min Drive Torque (Nm)'}
    {'max_brk_torque',     0, 'Max Brake Torque (Nm)'}
    {'min_brk_torque',-10000, 'Min Brake Torque (Nm)'}
    {'max_delta_torque', 2000, 'Max Delta Torque (Nm)'}
    {'max_delta_brake',   100, 'Max Delta Brake (Nm)'}
    {'max_speed', 11.176, 'Max Vehicle Speed (m/s)'}
    {'min_speed',     0, 'Min Vehicle Speed (m/s)'}
    {'max_alat', 3.5, 'Max Lateral Acceleration (m/s/s)'}
    {'max_steer', car.MAX_STEER, 'Max Steering Command (deg)'}
    {'min_steer', car.MIN_STEER, 'Min Steering Command (deg)'}
    {'max_delta_steer', 100, 'Max delta steer (deg/sample)'}
    {'kp_speed', 0.7826, 'Proportional gain for Speed Control'}
    {'ki_speed', 11, 'Integral gain for Speed Control'}
    {'kd_speed', 0.0108, 'Derivative gain for Speed Control'}
    {'fbreak_diff', 18.87, '1st order differentiator break freq. (Hz)'}
    {'kff_gain', car.MCAR * g * car.Rtire, 'Feedforward gain for pitch'}
    {'fbreak_fflpf', 5, '1st order LPF break freq. (Hz) for pitch'}
    {'fz_lead', 1.2732, 'Zero frequency (Hz) for lead filter'}
    {'zz_lead', 1.1, 'Zero damping ratio for lead filter'}
    {'fp_lead', 7*1.2732, 'Pole frequency (Hz) for lead filter'}
    {'kappa2theta', 2542.91, 'theta(deg) = kappa2theta * kappa(rad/m)'}
    {'fbreak_vref', 0.5, '1st order LPF break freq. (Hz) for Vref & Alat'}
    {'max_lookahead', 15, 'Max Lookahead Distance (m)'}
    {'min_lookahead', 3, 'Min Lookahead Distance (m)'}
    {'lookahead_ratelimit', 0.5, 'Lookahead Rate Limit (m/sample)'}
    {'fbreak_jerk', 2, '1st order LPF break freq. (Hz) for Jerk'}
    {'vdead', 0.1, 'Dead Zone Velocity (m/s)'}
    {'wT', 2*pi*2.8648, 'Torque model cutoff frequency (rad/s)'}
    {'zT', 0.9, 'Torque model damping ratio (nondim)'}
    {'wB', 2*pi*1.512, 'Brake model cutoff frequency (rad/s)'}
    {'zB', 1.1, 'Brake model damping ratio (nondim)'}
    };


%  --assemble the actual parameter structure

N = length(field_value_string);
clear s par pars
for n = 1:N
    nextfield = field_value_string{n}{1};
    nextval = field_value_string{n}{2};
    nextstr = field_value_string{n}{3};

    s{n,1} = nextstr;

    %  --enter the next value into the parameter struct
    temp = sprintf('pars.%s = %14.7f;',nextfield,nextval);
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

fields = fieldnames(pars);
N = length(fields);

maxlen = 0;
for n = 1:N
    maxlen = max(maxlen,length(fields{n}));
end
pad = blanks(maxlen);


for n = 1:N
    data = eval(strcat('pars.',fields{n},';'));
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
