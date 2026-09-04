%  This script is a tool for designing a brake lead compensation filter and
%  generating the Matlab code for implementation.

%  Last Modified:  2/4/2023

clear, clc
format compact
set(0,'defaultlinelinewidth',2);


%  --make sure the car and control parameters are pre-loaded

init_params


%  --define the sampling parameters

TS = pars.ts;
FS = 1/TS;

fr = logspace(-2,log10(FS/2),1000);


%  --symbolic design of DT brake compensator

syms K wT zT wB zB ts fs real
syms s z

BRAKE_s = (wB^2) / (s^2 + 2*zB*wB*s + wB^2);
TORQUE_s = (wT^2) / (s^2 + 2*zT*wT*s + wT^2);

BRAKE_LEAD_s = TORQUE_s / BRAKE_s;

pretty(BRAKE_LEAD_s)

BRAKE_LEAD_z = simplify(expand(subs(BRAKE_LEAD_s,s,((2/ts) * (z-1)/(z+1)))));

[numd,dend] = numden(BRAKE_LEAD_z);

numd = fliplr(coeffs(collect(numd,z),z));
dend = fliplr(coeffs(collect(dend,z),z));

a0 = simplify(dend(1));
a1 = simplify(dend(2));
a2 = simplify(dend(3));
b0 = simplify(numd(1));
b1 = simplify(numd(2));
b2 = simplify(numd(3));


%  --display the code to include in Matlab scripts

fprintf('    A0 = %s;\n',a0);
fprintf('    A1 = %s / A0;\n',a1);
fprintf('    A2 = %s / A0;\n',a2);
fprintf('    B0 = %s / A0;\n',b0);
fprintf('    B1 = %s / A0;\n',b1);
fprintf('    B2 = %s / A0;\n',b2);

fprintf('\n    yk = - A1*ykm1 - A2*ykm2 + ...\n');
fprintf('           B0*uk + B1*ukm1 + B2*ukm2;\n')
fprintf('\n    ykm2 = ykm1;\n    ykm1 = yk;\n')
fprintf('    ukm2 = ukm1;\n    ukm1 = uk;\n\n')


%  --extract the CT torque dynamics numerical parameters

WT = pars.wT;   % Natural Frequency (rad/s) for torque
ZT = pars.zT;   % Damping Ratio for torque


%  --extract the CT brake dynamics numerical parameters

WB = pars.wB;   % Natural Frequency (rad/s) for brake
ZB = pars.zB;   % Damping Ratio for brake


%  --substitute numerical values into the symbolic expressions

symvars = {wT,zT,wB,zB,ts};
numvars = {WT,ZT,WB,ZB,TS};

A0 = double(subs(a0,symvars,numvars));
A1 = double(subs(a1,symvars,numvars)) / A0;
A2 = double(subs(a2,symvars,numvars)) / A0;
B0 = double(subs(b0,symvars,numvars)) / A0;
B1 = double(subs(b1,symvars,numvars)) / A0;
B2 = double(subs(b2,symvars,numvars)) / A0;

BRAKE_LEAD_NUMD = [B0 B1 B2];
BRAKE_LEAD_DEND = [1 A1 A2];


%  --compute the frequency responses
%  Note:  TORQUE_NUMS and TORQUE_DENS are defined in init_car.m
%         BRAKE_NUMS and BRAKE_DENS are defined in init_car.m

FRFT = freqs(TORQUE_NUMS,TORQUE_DENS,2*pi*fr);
FRFB = freqs(BRAKE_NUMS,BRAKE_DENS,2*pi*fr);

FRFL = freqz(BRAKE_LEAD_NUMD,BRAKE_LEAD_DEND,fr,FS);
FRFC = FRFB .* FRFL;


%  --plot the FRFs
T = [FRFT; FRFB; FRFL; FRFC];

figure(1), clf
subplot(2,1,1);
h = semilogx(fr,(180/pi)*angle(T));
set(h(4),'color','k','linestyle','--')
grid on
ylabel('Phase (deg)')
title('Brake Lead Compensator Design')

subplot(2,1,2);
h = semilogx(fr,20*log10(abs(T)));
set(h(4),'color','k','linestyle','--')
grid on
ylabel('Mag (dB)')
xlabel('Frequency (Hz)')
legend('Torque','Brake','Lead','Compensated = Brake * Lead', ...
    'location','southwest')
