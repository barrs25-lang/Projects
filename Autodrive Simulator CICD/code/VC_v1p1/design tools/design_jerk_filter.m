%  This script is a tool for designing a steer lead compensation filter and
%  generating the Matlab code for implementation.

%  1/28/2023

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

syms w0 z0 w1 ts fs real
syms s z

K = w1 * (w0^2);

nums = K * s;
dens = expand((s + w1) * (s^2 + 2*z0*w0*s + w0^2));

DIFF_s = nums/dens;

pretty(DIFF_s)

DIFF_z = simplify(expand(subs(DIFF_s,s,((2/ts) * (z-1)/(z+1)))));

[numd,dend] = numden(DIFF_z);



numd = fliplr(coeffs(collect(numd,z),z));
dend = fliplr(coeffs(collect(dend,z),z));

a0 = simplify(dend(1));
a1 = simplify(dend(2));
a2 = simplify(dend(3));
a3 = simplify(dend(4));
b0 = simplify(numd(1));


%  --display the code to include in Matlab scripts

fprintf('    A0 = (%s);\n',a0);
fprintf('    A1 = (%s) / A0;\n',a1);
fprintf('    A2 = (%s) / A0;\n',a2);
fprintf('    A3 = (%s) / A0;\n',a3);
fprintf('    B0 = (%s) / A0;\n',b0);

fprintf('\n    yk = - A1*ykm1lat - A2*ykm2lat  - A3*ykm3lat + ...\n');
fprintf('           B0*(alat + ukm1lat - ukm2lat - ukm3lat);\n')
fprintf('\n    ykm3lat = ykm2lat;\n    ykm2lat = ykm1lat;\n    ykm1lat = yk;\n')
fprintf('    ukm3lat = ukm2lat;\n    ukm2lat = ukm1lat;\n    ukm1lat = alat;\n\n')

fprintf('\n    zk = - A1*ykm1lon - A2*ykm2lon  - A3*ykm3lon + ...\n');
fprintf('           B0*(along + ukm1lon - ukm2lon - ukm3lon);\n')
fprintf('\n    ykm3lon = ykm2lon;\n    ykm2lon = ykm1lon;\n    ykm1lon = zk;\n')
fprintf('    ukm3lon = ukm2lon;\n    ukm2lon = ukm1lon;\n    ukm1lon = along;\n\n')



%  --extract the numerical jerk filter parameters

W0 = 2*pi*10;
Z0 = 0.35;
W1 = 2*pi*7;


%  --substitute numerical values into the symbolic expressions

symvars = {w0,z0,w1,ts};
numvars = {W0,Z0,W1,TS};

A0 = double(subs(a0,symvars,numvars));
A1 = double(subs(a1,symvars,numvars)) / A0;
A2 = double(subs(a2,symvars,numvars)) / A0;
A3 = double(subs(a3,symvars,numvars)) / A0;
B0 = double(subs(b0,symvars,numvars)) / A0;

DIFF_NUMD = B0 * [1 1 -1 -1];
DIFF_DEND = [1 A1 A2 A3];

K = W1 * (W0^2);

DIFF_NUMS = K * [1 0];
DIFF_DENS = [1 (2*Z0*W0 + W1) (W0^2 + 2*Z0*W0*W1) (K)];


%  --compute the frequency response

FRFS_IDEAL = freqs([1 0],[0 1],2*pi*fr);  % ideal CT differentiator

FRFS = freqs(DIFF_NUMS,DIFF_DENS,2*pi*fr); % approximate CT differentiator

FRFD = freqz(DIFF_NUMD,DIFF_DEND,fr,FS); % approximate DT differentiator


%  --plot the FRFs
T = [FRFS_IDEAL; FRFS; FRFD];

figure(1), clf
subplot(2,1,1);
h = semilogx(fr,(180/pi)*angle(T));
grid on
ylabel('Phase (deg)')
xlim([fr(2),fr(end)])
title('Differentiator (Jerk Filter) Design')

subplot(2,1,2);
h = semilogx(fr,20*log10(abs(T)));
grid on
ylabel('Mag (dB)')
xlabel('Frequency (Hz)')
xlim([fr(2),fr(end)])
ylim([-30,50])
legend('Ideal CT Design','Approximate CT Design','Approximate DT Design', ...
    'location','northwest')
