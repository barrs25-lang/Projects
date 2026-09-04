function [north, east, Lpath] = clothoid_endpoint(north0, east0, phi0, ...
    kappa0, kp, Rtarget, maxiter, tol)
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
%  This function computes the (north,east) endpoint coordinates and the 
%  path length of a clothoid that reaches a target radius Rtarget within a 
%  specified tolerance distance.
%
%  north0 = initial northing-position (m)
%  east0 = initial easting-position (m)
%  phi0 = initial heading angle (rad) [positive from north to east]
%  kappa0 = initial curvature (rad/m)
%  kp = constant curvature rate (dkappa/ds) for this clothoid
%  Rtarget = target radius of the clothoid (m)
%  maxiter = maximum number of iterations to search in last segment
%  tol = tolerance threshold for exiting search (m)
%
%  north = final northing-position coordinate (m)
%  east = final easting-position coordinate (m)
%  Lpath = final clothoid path length (m)
%
%  This function does not call any external functions

%  Last Updated:  1/14/2023

%  --initialize constants

dS0 = 1;        % nominal clothoid path discrete step (m)


%  --initialize local variables

Lpath = 0;      % current length of clothoid path (m)
phi = phi0;     % heading angle at current point on clothoid (rad)
D = 0;          % current radius of clothoid path (m)
north = north0; % current northing coordinate of clothoid (m)
east = east0;   % current easting coordinate of clothoid (m)
dS = dS0;       % current length of clothoid path step (m)
count = 0;      % final step iteration count value
lastD = -1;     % initial value of previous radius D (m)


%  --step through the clothoid path

while ((D < Rtarget) && (D > lastD) && (count < maxiter))

    %  --check to see if we are close enough

    if (abs(Rtarget - D) < tol), break; end


    %  --adjust the increment if we are close to the target

    if (D > (Rtarget-dS))

        %  --(Rtarget-dS-tol) < Dtarget < (Rtarget+tol)
        dS = dS * (Rtarget - D) / (Rtarget - lastD);

        %  --increment final step counter
        count = count + 1;
    end


    %  --save last data before updating new values

    lastL = Lpath;
    lastphi = phi;
    lastN = north;
    lastE = east;
    lastD = D;


    %  --trapezoidal clothoid integration over one path step

    Lpath = lastL + dS;

    phi = lastphi + dS * (kappa0 + kp * (lastL + (dS/2)) );

    north = lastN + (dS/2) * (cos(phi) + cos(lastphi));
    east = lastE + (dS/2) * (sin(phi) + sin(lastphi));

    D = sqrt( (north-north0)^2 + (east-east0)^2 );
end

return



