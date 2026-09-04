function [KWP,HWP,SWP] = curvature_and_heading(EWP,NWP)
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
%  This function calculates the curvature at each waypoint by first fitting
%  parametric polynomials to the northing and easting waypoints.  The
%  analytic curvature is then computed from the coefficients.  This
%  function also calculates the heading at each waypoint using the fitted 
%  polynomial coefficients.
%
%  EWP = vector of easting waypoint coordinates (m)
%  NWP = vector of northing waypoint coordinates (m)
%  
%  KWP = vector of curvature values at each waypoint (rad/m)
%  HWP = vector of heading values at each waypoint (rad)
%  SWP = vector of path distances at each waypoint (m)

%  S. C. Southward    3/1/2023

%  --define arrays for the outputs

NPTS = length(NWP);

KWP = zeros(size(NWP));
HWP = zeros(size(NWP));
SWP = zeros(size(NWP));


%  --determine the path position at each waypoint

for n = 2:NPTS
    dN = NWP(n) - NWP(n-1);
    dE = EWP(n) - EWP(n-1);
    SWP(n) = SWP(n-1) + sqrt(dN^2 + dE^2);
end


%  --loop through each waypoint and compute curvature

for n = 1:NPTS
    %  --get the indices for the next three consecutive waypoints

    if (n == 1)
        %  --use a forward difference
        index = [1:3];
    elseif (n == NPTS)
        %  --use a backward difference
        index = [NPTS-2:NPTS];
    else
        %  --use a central difference
        index = [n-1:n+1];
    end

    %  --the following analysis requires two 2nd order polynomial fits
    %
    %  nwp_fit(u) = PN(1)*u^2 + PN(2)*u + PN(3)
    %  ewp_fit(u) = PE(1)*u^2 + PE(2)*u + PE(3)
    %
    %  nwp_fit(u1) = NWP(index(1))
    %  nwp_fit(u2) = NWP(index(2))
    %  nwp_fit(u3) = NWP(index(3))
    %
    %  ewp_fit(u1) = EWP(index(1))
    %  ewp_fit(u2) = EWP(index(2))
    %  ewp_fit(u3) = EWP(index(3))
    %
    %  u1 = (SWP(index(1)) - SWP(index(2)) < 0
    %  u2 = (SWP(index(2)) - SWP(index(2)) = 0
    %  u3 = (SWP(index(3)) - SWP(index(2)) > 0

    %  --define the parametric array using the distance between waypoints

    u1 = SWP(index(1)) - SWP(index(2));    % should be negative
    u2 = 0;  % parametric value of u @ waypoint to compute curvature
    u3 = SWP(index(3)) - SWP(index(2));    % should be positive


    %  --from the symbolic toolbox we know that:
    %
    %     A =
    %       [u1^2, u1, 1]
    %       [u2^2, u2, 1]
    %       [u3^2, u3, 1]
    % 
    %     detA = (u1 - u2)*(u1 - u3)*(u2 - u3)
    % 
    %     invA =
    %       [        u2 - u3,        - u1 + u3,         u1 - u2]
    %       [  - u2^2 + u3^2,      u1^2 - u3^2,   - u1^2 + u2^2]
    %       [u2*u3*(u2 - u3), -u1*u3*(u1 - u3), u1*u2*(u1 - u2)] / detA


    %  --compute the numerical determinant and inverse of A

    detA = (u1 - u2) * (u1 - u3) * (u2 - u3);  % determinant of A

    invA = [        u2 - u3,        - u1 + u3,         u1 - u2
              - u2^2 + u3^2,      u1^2 - u3^2,   - u1^2 + u2^2
            u2*u3*(u2 - u3), -u1*u3*(u1 - u3), u1*u2*(u1 - u2)] / detA;


    %  --compute the polynomial coefficients:  PN = invA * NWP

    PE = zeros(1,3);
    PN = zeros(1,3);
    for row = 1:3
        for col = 1:3
            PN(row) = PN(row) + invA(row,col) * NWP(index(col));
            PE(row) = PE(row) + invA(row,col) * EWP(index(col));
        end
    end
           
    
    %  --compute the analytic curvature using the polynomial coefficients
    %    and evaluate the result at the middle waypoint (u=0)
    %    https://en.wikipedia.org/wiki/Curvature
    %
    %    Note:  x = northing, y = easting, psi=0 means north
    %           and psi>0 means a rotation from north to east
    %
    %                   x'(0)*y''(0) - y'(0)*x''(0)
    %    kappa(u=0) = -------------------------------
    %                 ((x'(0))^2 + (y'(0))^2) ^ (3/2)
    %
    %    where:
    %       x(u) = nwp_fit(u) = PN(1)*u^2 + PN(2)*u + PN(3)
    %       x'(u) = dx(u)/du = 2*PN(1)*u + PN(2)
    %       x''(u) = d^2(x(u))/du^2 = 2*PN(1)
    %
    %       y(u) = ewp_fit(u) = PE(1)*u^2 + PE(2)*u + PE(3)
    %       y'(u) = dy(u)/du = 2*PE(1)*u + PE(2)
    %       y''(u) = d^2(y(u))/du^2 = 2*PE(1)

    num = 2 * ( PN(2)*PE(1) - PE(2)*PN(1) );
    den = ( PE(2)^2 + PN(2)^2 ) ^ (3/2);

    KWP(n) = num/den;

    %  Note:
    %       Curvature is also defined as:  kappa(u) = dpsi(u)/du
    %       In a right-hand coordinate system, a positive heading angle
    %       is when the rotation is from northing to easting, therefore a
    %       positive value of curvature occurs when turning from northing
    %       to easting.

    %  --compute the analytic heading angle using polynomial coefficients
    %    psi = atan2(y'(0),x'(0))

    HWP(n) = atan2(PE(2),PN(2));
end

return




