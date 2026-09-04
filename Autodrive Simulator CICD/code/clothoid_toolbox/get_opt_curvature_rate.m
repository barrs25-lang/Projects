function kpdes  = get_opt_curvature_rate(kcar, kpmax, kpmin, R, ...
    psi, Ncar, Ntarget, Ecar, Etarget, maxiter, tol)
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
%
%  This function determines the optimal curvature rate given an initial
%  curvature, an initial pose, a final target, and an initial heading.
%
%  This function calls the following functions:
%       clothoid_endpoint.m
%       vc_mod.m

%  Last Updated:  1/16/2023

%   --define the local persistent variables

persistent JTOL MAXCOUNT

if (isempty(JTOL))
    JTOL = (pi/180)*0.1;  % angular target-to-clothoid error tolerance (rad)
    MAXCOUNT = 10;        % maximum number of bisection search steps
end

% kpsave = [];
% Jsave = [];


%  --compute the absolute angle (rad) of a line between car and target

theta_CT = atan2( (Etarget-Ecar), (Ntarget-Ncar) );


%  --initialize for bisection search

kp1 = kpmin;
[north,east,~] = clothoid_endpoint(Ncar,Ecar,psi,kcar,kp1,R,maxiter,tol);
theta_CE = atan2( (east-Ecar), (north-Ncar) );  % car to endpoint angle
J1 = vc_mod(theta_CT - theta_CE + pi, 2*pi) - pi;

kp2 = kpmax;
[north,east,~] = clothoid_endpoint(Ncar,Ecar,psi,kcar,kp2,R,maxiter,tol);
theta_CE = atan2( (east-Ecar), (north-Ncar) );  % car to endpoint angle
J2 = vc_mod(theta_CT - theta_CE + pi, 2*pi) - pi;

% J1save = J1;
% J2save = J2;


%  --check for out-of-range curvature rate
%  Note:  This analysis assumes that the cost function J defined over the
%  input interval [kpmin,kpmax] is monotonic (increasing or decreasing).
%  When the entire cost function is positive (or negative) over this
%  interval, it means that the optimal curvature rate is outside the
%  limited range.

if (sign(J1) == sign(J2))
    %  --monotonic cost function is only positive or negative
    if (abs(J1) < abs(J2))
        %  --cost J1 is closer to zero so choose this limit
        kpdes = kp1;
    else
        %  --cost J2 is closer to zero so choose this limit
        kpdes = kp2;
    end

else
    %  --begin bisection search for zero crossing

    count = 1;
    Jmid = 10*JTOL;  % dummy initial value
    kp = 0;

    while ((count < MAXCOUNT) && (abs(Jmid) > JTOL))
        %  --compute the next midpoint curvature rate
        %  Note:  The midpoint is computed by fitting a line between the
        %  two endpoints of this segment and explicitly computing the zero
        %  crossing of that line.

        if (J2 == J1), break; end   % terminate search when slope is zero
        
        minv = (kp2 - kp1) / (J2 - J1);  % inverse slope of segment line

        kp = kp1 - J1 * minv;  % curvature rate where segment crosses zero


        %  --saturate the curvature rate just in case

        if (kp > kpmax)
            kp = kpmax;
        elseif (kp < kpmin)
            kp = kpmin;
        end


        %  --get next cost at this curvature rate

        [north,east,~] = clothoid_endpoint(Ncar,Ecar,psi,kcar,kp,R,maxiter,tol);
        theta_CE = atan2( (east-Ecar), (north-Ncar) );  % car to endpoint angle
        Jmid = vc_mod(theta_CT - theta_CE + pi, 2*pi) - pi;


        %  --adjust the interval limits

        if (sign(J1) ~= sign(Jmid))
            %  --the first sub-interval has the zero-crossing
            J2 = Jmid;
            kp2 = kp;
        else
            %  --the second sub-interval has the zero-crossing
            J1 = Jmid;
            kp1 = kp;
        end

%         kpsave(count) = kp;
%         Jsave(count) = Jnew;

        count = count + 1;
    end

    %  --save the final curvature rate

    kpdes = kp;
end

% if exist('count','var')
%     fprintf('count = %g\n',count);
% end


%  --saturate the kp value

if (kpdes > kpmax)
    kpdes = kpmax;
elseif (kpdes < kpmin)
    kpdes = kpmin;
end


%=DEBUG=ONLY==DEBUG=ONLY==DEBUG=ONLY==DEBUG=ONLY=
% figure(2)
% clf
% plot(kpsave,Jsave,'r.-','markersize',10)
% grid on
% hold on
% plot(kpmin,J1save,'b.','markersize',20)
% plot(kpmax,J2save,'b.','markersize',20)
% xlabel('kp')
% ylabel('J')
% xlim([kpmin,kpmax])
% xline(kpdes,'g--','linewidth',2)
% yline(0,'g--','linewidth',2)
% yline(Jtol)
% yline(-Jtol)
%=DEBUG=ONLY==DEBUG=ONLY==DEBUG=ONLY==DEBUG=ONLY=

return
