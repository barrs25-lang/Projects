%  This script is a tool for designing a steer lead compensation filter and
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


%  --symbolic design of DT steer compensator

syms wp zz wz ts fs real
syms s z

K = (wp^3) / (wz^2);

nums = K * (s^2 + 2*zz*wz*s + wz^2);
dens = (s + wp)^3;

STEER_LEAD_s = nums/dens;

pretty(STEER_LEAD_s)

STEER_LEAD_z = simplify(expand(subs(STEER_LEAD_s,s,((2/ts) * (z-1)/(z+1)))));

[numd,dend] = numden(STEER_LEAD_z);

numd = fliplr(coeffs(collect(numd,z),z));
dend = fliplr(coeffs(collect(dend,z),z));

a0 = simplify(dend(1));
a1 = simplify(dend(2));
a2 = simplify(dend(3));
a3 = simplify(dend(4));
b0 = simplify(numd(1));
b1 = simplify(numd(2));
b2 = simplify(numd(3));
b3 = simplify(numd(4));


%  --display the code to include in Matlab scripts

fprintf('    A0 = (%s);\n',a0);
fprintf('    A1 = (%s) / A0;\n',a1);
fprintf('    A2 = (%s) / A0;\n',a2);
fprintf('    A3 = (%s) / A0;\n',a3);
fprintf('    B0 = (%s) / A0;\n',b0);
fprintf('    B1 = (%s) / A0;\n',b1);
fprintf('    B2 = (%s) / A0;\n',b2);
fprintf('    B3 = (%s) / A0;\n',b3);

fprintf('\n    yk = - A1*ykm1 - A2*ykm2  - A3*ykm3 + ...\n');
fprintf('           B0*uk + B1*ukm1 + B2*ukm2 + B3*ukm3;\n')
fprintf('\n    ykm3 = ykm2;\n    ykm2 = ykm1;\n    ykm1 = yk;\n')
fprintf('    ukm3 = ukm2;\n    ukm2 = ukm1;\n    ukm1 = uk;\n\n')


%  --extract the numerical lead filter parameters

WZ = 2*pi*pars.fz_lead;
ZZ = pars.zz_lead;
WP = 2*pi*pars.fp_lead;


%  --substitute numerical values into the symbolic expressions

symvars = {wp,zz,wz,ts};
numvars = {WP,ZZ,WZ,TS};

A0 = double(subs(a0,symvars,numvars));
A1 = double(subs(a1,symvars,numvars)) / A0;
A2 = double(subs(a2,symvars,numvars)) / A0;
A3 = double(subs(a3,symvars,numvars)) / A0;
B0 = double(subs(b0,symvars,numvars)) / A0;
B1 = double(subs(b1,symvars,numvars)) / A0;
B2 = double(subs(b2,symvars,numvars)) / A0;
B3 = double(subs(b3,symvars,numvars)) / A0;

STEER_LEAD_NUMD = [B0 B1 B2 B3];
STEER_LEAD_DEND = [1 A1 A2 A3];


%  --compute the frequency responses
%  Note:  STEER_NUMS and STEER_DENS are defined in init_car.m

FRFS = freqs(STEER_NUMS,STEER_DENS,2*pi*fr);

FRFL = freqz(STEER_LEAD_NUMD,STEER_LEAD_DEND,fr,FS);
FRFC = FRFS .* FRFL;


%  --plot the FRFs
T = [FRFS; FRFL; FRFC];

figure(1), clf
subplot(2,1,1);
h = semilogx(fr,(180/pi)*angle(T));
% set(h(3),'color','k','linestyle','--')
grid on
ylabel('Phase (deg)')
xlim([fr(2),fr(end)])
title('Steer Lead Compensator Design')

subplot(2,1,2);
h = semilogx(fr,20*log10(abs(T)));
% set(h(3),'color','k','linestyle','--')
grid on
ylabel('Mag (dB)')
xlabel('Frequency (Hz)')
xlim([fr(2),fr(end)])
ylim([-60,30])
legend('Steer','Lead','Compensated = Steer * Lead', ...
    'location','southwest')
