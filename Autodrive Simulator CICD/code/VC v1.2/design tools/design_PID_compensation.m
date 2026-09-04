%  This script is a tool for designing the PID compensation gains for speed
%  control.

%  2/2/2023

clear, clc
format compact
set(0,'defaultlinelinewidth',2);


%  --make sure the car and control parameters are pre-loaded

init_params


%  --define the sampling parameters

FS = 100;
TS = 1/FS;

fr = logspace(-4,log10(FS/2),1000);


%  --compute the frequency responses

wt = sqrt(car.TORQUE_NUMS);     % Natural Frequency (rad/s) for torque
ftorque = wt/2/pi
zt = car.TORQUE_DENS(2)/2/wt;   % Damping Ratio for torque

TORQUE_FILTER = tf(car.TORQUE_NUMS,car.TORQUE_DENS);

CAR_INERTIA = tf([1/car.Rtire],[car.MCAR, car.RRdamp]);

TORQUE = CAR_INERTIA * TORQUE_FILTER;

FRFT = squeeze(freqresp(TORQUE,2*pi*fr));

method = 1;

switch method
    case 0  % use PIDTUNE
        C = pidtune(TORQUE,'PID');

        KP = C.Kp;
        KI = C.Ki;
        KD = C.Kd;

        PID_NUMS = [KD KP KI];
        PID_DENS = [0 1 0];

    case 1  % use CLDESIGN (not available to students)
        %  a = 2*pi*fbreak_diff
        %  PID_NUMS = [(KD*a + KP) (KP*a + KI) (KI*a)]
        %  PID_DENS = [1 a 0]

        if (exist('PID_design1.mat') == 2)
            load PID_design1
            [PID_NUMS,PID_DENS] = tfdata(sys_controller,'v');
            PID_NUMS = PID_NUMS / PID_DENS(1);
            PID_DENS = PID_DENS / PID_DENS(1);

            a = PID_DENS(2);
            fbreak_diff = a/2/pi

            KI = PID_NUMS(3) / a
            KP = (PID_NUMS(2) - KI) / a
            KD = (PID_NUMS(1) - KP) / a

        else    
            cldesign(FRFT,fr,'PID_design1.mat');
        end

end

PID_CONTROLLER = tf(PID_NUMS,PID_DENS);

L = TORQUE * PID_CONTROLLER;
[Gm,Pm,Wcg,Wcp] = margin(TORQUE * PID_CONTROLLER);
Gm = 20*log10(Gm);
fPC = Wcg/2/pi;
fGC = Wcp/2/pi;

FRFPID = squeeze(freqresp(PID_CONTROLLER,2*pi*fr));

OL = FRFT .* FRFPID;

CL = OL ./ (1 + OL);


%  --plot the FRFs
T = [FRFT, OL, CL];

figure(1), clf
subplot(2,1,1);
h = semilogx(fr,(180/pi)*angle(T));
grid on
hold on
xline(FS/2)
yline(Pm-180,'m')
xline(fGC,'m')
xline(fPC,'c')
ylabel('Phase (deg)')
xlim([fr(2),fr(end)])
title('PID Compensator Design')

subplot(2,1,2);
h = semilogx(fr,20*log10(abs(T)));
grid on
hold on
xline(FS/2)
yline(Gm,'y')
yline(-3)
xline(fGC,'m')
xline(fPC,'c')
ylabel('Mag (dB)')
xlabel('Frequency (Hz)')
xlim([fr(2),fr(end)])
legend('Plant','Open Loop','Closed Loop', ...
    'location','southwest')
