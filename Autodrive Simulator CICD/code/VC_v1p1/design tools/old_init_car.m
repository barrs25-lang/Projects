%  init_car.m
%
%  This script initializes parameters for the Bolt EUV car model
%  Last Updated:  1/5/2023

%  --define misc constants
g = 9.81;       % gravity (m/s^2)


%  --define the simulation sampling parameters
%  Note:  Do NOT change!  This rate matches the Bolt CAN bus rate.
fs = 100;       % data sample rate (Hz)
ts = 1/fs;      % sample period (sec)


%  --define the vehicle kinematic model parameters
%  (https://media.chevrolet.com/media/us/en/chevrolet/2022-bolt-euv-bolt-ev.detail.html/content/Pages/news/us/en/2021/feb/0214-boltev-bolteuv-specifications.html)
WB = 2.675;     % wheelbase (m)
TR = 1.511;     % track (m)

Lcar = 4.306;   % length (m)
Wcar = 1.770;   % width (m)
Hcar = 1.616;   % height (m)


%  --assume CG is halfway between front and rear axles
LR = WB / 2;    % distance from cg to rear wheel (m)
LF = WB - LR;   % distance from cg to front wheel (m)


%  --define the vehicle dynamic model parameters
%  Note:  Anthropometric data was taken from:
%         www.cdc.gov/nchs/data/series/sr_11/sr11_252.pdf
Mmale = 124.1;          % mass of 95th percentile male, 20+ years old
Mfem = 113.8;           % mass of 95th percentile female, 20+ years old
Mcar = 1669;            % baseline mass of car (kg)

MCAR = Mcar + 1*Mmale + 1*Mfem;   % total mass of car


%  --define the tire parameters
%    (https://x-engineer.org/calculate-wheel-radius/)
%    Note:  Michelin Energy Saver A/S 215/50R17
Wtire = 215/1000;  % tire width (m)
aspect_ratio_pct = 50/100;
Htire = Wtire * aspect_ratio_pct;  % tire height (m)
inch_per_meter = 39.3701;  % conversion: inches in one meter
Drim = 17/inch_per_meter;  % diameter of rim (m)
Dtire = Drim + 2*Htire;    % total diameter of tire (m)

%  --correct the tire radius for loading (dynamic rolling radius)
%  http://www.stealthmotorsport.co.uk/wheeldiam.html
correction = 0.9709;
Rtire = (Dtire / 2) * correction; 


%  --define the remaining parameters
STEER_RATIO = 0.058221; % delta (deg) = STEER_RATIO * steer_cmd (deg)

MAX_TORQUE = 22534;     % maximum torque command (Nm)
MIN_TORQUE = 0;         % minimum torque command (Nm)

MAX_BRAKE = 0;          % maximum brake command (Nm)
MIN_BRAKE = -65534;     % minimum brake command (Nm)

MAX_STEER = 540;        % maximum steer command (deg)
MIN_STEER = -540;       % minimum steer command (deg)

tau = 145;              % time constant estimated (sec)
RRdamp = MCAR/tau;      % rolling resistance (linear viscous) damping (Ns/m)

Vdead = 0.05;           % deadzone velocity (m/s)
Tdead = 100;            % deadzone torque (Nm)


%  --torque dynamics
wn = 18;      % Natural Frequency (rad/s) for torque
zeta = 0.9;   % Damping Ratio for torque

TORQUE_NUMS = wn^2;
TORQUE_DENS = [1 2*zeta*wn wn^2];


%  --brake dynamics
wn = 9.5;     % Natural Frequency (rad/s) for brake
zeta = 1.1;   % Damping Ratio for brake

BRAKE_NUMS = wn^2;
BRAKE_DENS = [1 2*zeta*wn wn^2];


%  --steer dynamics
wn = 8;       % Natural Frequency (rad/s) for steer
zeta = 0.65;  % Damping Ratio for steer

STEER_NUMS = wn^2;
STEER_DENS = [1 2*zeta*wn wn^2];


%  --define the initial vehicle pose
%  Note:  Must always start with zero initial velocity!
east0 = 0;    % initial easting coordinate (meters)
north0 = 0;   % initial northing coordinate (meters)
psi0 = 0;    % initial heading (radians)
vcg0 = 0;      % initial forward velocity (meters/sec)

grade0 = 0;  % degrees


%  --clean up

clear Drim Dtire Hcar Htire Lcar TR Wcar Wtire
clear aspect_ratio_pct correction inch_per_meter
clear W WB Mmale Mfem Mcar tau wn zeta
