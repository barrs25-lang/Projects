function [kpmax,kpmin] = kplimit_search(R,k0,kprange,tol)
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
%  This function finds the upper and lower bounds for curvature rate
%  kp such that the resulting extremal clothoids just reach the target
%  radius R (m).
%
%  This function calls the following functions:
%       clothoid_endpoint.m

%  --define initial values for search

n0 = 0;
e0 = 0;
phi0 = 0;


%  --create a linear search array for kp values

N = 50000;
KP = linspace(kprange(1),kprange(2),N);

D = zeros(1,N);

maxiter = 500;


%  --evaluate the clothoid at each kp grid point

for n = 1:N
    %  --get the next clothoid endpoint
    [nf,ef,~] = clothoid_endpoint(n0,e0,phi0,k0,KP(n),R,maxiter,tol);

    %  --compute the final radial distance
    D(n) = sqrt( (nf-n0)^2 + (ef-e0)^2 );
end

% figure(2)
% clf
% plot(KP,D)
% grid on
% hold on
% yline(R)

%  --get the min and max values of kp

k = find(abs(R-D) < tol);

kpmin = KP(k(1));
kpmax = KP(k(end)); 

return



