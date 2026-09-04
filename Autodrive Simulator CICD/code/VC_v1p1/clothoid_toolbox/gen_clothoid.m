function [north,east] = gen_clothoid(north0,east0,psi0,kappa0,kp,R)
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
%  This function generates a clothoid curve using a trapezoidal integral
%  approximation to the clothoid integrals.
%
%  north0 = initial northing-position (m)
%  east0 = initial easting-position (m)
%  psi0 = initial heading angle (rad) [positive from north to east]
%  kappa0 = initial curvature (rad/m)
%  kp = constant curvature rate (dkappa/ds) for this clothoid
%  R = desired length of the clothoid (m)
%
%  north = output array of northing-position coordinates (m)
%  east = output array of easting-position coordinates (m)
%
%  This function does not call any external functions.

%  Last Updated:  1/11/2023

%  --define the local persistent variables

persistent NGRID S PHI COSPHI SINPHI NORTH EAST DS

if isempty(NGRID)
    %  --choose an odd number of grid points
    %  Note:  A value of NGRID = 21 insures that a max lookahead of 
    %         20 meters results in a max value of DS = 1 meter
    NGRID = 21;

    %  --initialize the constant normalized path array
    S = zeros(1,NGRID);
    DS = 1 / (NGRID - 1);  % delta-s (normalized)
    for n = 1:NGRID
        S(n) = (n-1) * DS;  % scale by R to get actual path
    end

    %  --initialize the local storage arrays
    PHI = zeros(1,NGRID);
    COSPHI = zeros(1,NGRID);
    SINPHI = zeros(1,NGRID);
    NORTH = zeros(1,NGRID);
    EAST = zeros(1,NGRID);
end


%  --compute the tangent angle (rad) at each grid point along the path

for n = 1:NGRID
    %  --use Horners Rule for efficient polynomial evaluation
    R_S = R * S(n);
    PHI(n) = psi0 + (R_S) * (kappa0 + (R_S) * (kp/2));

    %  --compute the clothoid integrands
    COSPHI(n) = cos(PHI(n));
    SINPHI(n) = sin(PHI(n));   
end


%  --evaluate the Trapezoidal rule integration

R_DS = R * (DS/2);  % pre-compute scaled delta-s for integration

for n = 1:NGRID
    if (n == 1)
        %  --initial values are provided as inputs
        NORTH(n) = north0;
        EAST(n) = east0;
    else
        %  --accumulate the result at each grid point
        NORTH(n) = NORTH(n-1) + R_DS * (COSPHI(n) + COSPHI(n-1));
        EAST(n) = EAST(n-1) + R_DS * (SINPHI(n) + SINPHI(n-1));
    end
end


%  --transfer output arrays

north = NORTH;
east = EAST;

return



