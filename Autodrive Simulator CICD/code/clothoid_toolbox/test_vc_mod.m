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
%  This script tests the vc_mod.m function.

clear, clc
clear vc_mod

%  --test vc_mod with radian angles

M = 1000;

N = 5;
x_rad = linspace(-N*pi,N*pi,M);

for m = 1:M
    y_rad(m) = vc_mod(x_rad(m)+pi,2*pi) - pi;
end
m_rad = mod(x_rad+pi,2*pi) - pi;


%  --test vc_mod with degree angles

x_deg = x_rad * 180/pi;

for m = 1:M
    y_deg(m) = rtc_mod(x_deg(m),360);
end
m_deg = mod(x_deg,360);


%  --plot the results

lw = 2;

figure(1)
clf
subplotfill(2,1,1);
plot(x_rad,y_rad,'r','linewidth',lw)
hold on
plot(x_rad,m_rad,'b--','linewidth',lw)
grid on
ylabel('vc-mod(theta) [rad]')
xlabel('theta [rad]')

subplotfill(2,1,2);
plot(x_deg,y_deg,'r','linewidth',lw)
hold on
plot(x_deg,m_deg,'b--','linewidth',lw)
grid on
ylabel('vc-mod(theta) [deg]')
xlabel('theta [deg]')



