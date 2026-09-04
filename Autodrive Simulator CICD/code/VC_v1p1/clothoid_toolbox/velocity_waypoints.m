function VWP = velocity_waypoints(KWP,SWP, ...
    vstart,vmax,vfinal,max_alat,max_along)
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
%  This function defines a vector of velocity waypoints that attempt to use
%  the maximum specified velocity, but derates the velocities based on
%  acceleration limits.
%
%  KWP = vector of curvature waypoints (rad/m)
%  SWP = vector of path position waypoints (m) 
%  vstart = velocity at first waypoint (m/s)
%  vmax = maximum desired velocity (m/s)
%  vfinal = velocity at last waypoint (m/s)
%  max_alat = maximum lateral acceleration (m/s/s)
%  max_along = maximum longitudinal acceleration (m/s/s)
%
%  VWP = vector of resulting velocity waypoints (m/s)

%  This function does not call any external user functions.

N = length(KWP);  % total number of waypoints


%  --define initial velocity waypoints only considering lateral acceleration

VWP = zeros(size(KWP));

for n = 1:N
    %  --calculate the maximum velocity based on curvature at this waypoint
    %    max_alat = kwp * vwp_max^2
    %    vwp_max = sqrt(abs(max_alat / kwp))

    maxv_from_max_alat = sqrt(abs(max_alat / KWP(n)));

    %  --limit the velocity
    if (maxv_from_max_alat > vmax)
        if (n == 1)
            VWP(n) = min(vstart,vmax);
        elseif (n == N)
            VWP(n) = min(vfinal,vmax);
        else
            VWP(n) = vmax;
        end
    else
        VWP(n) = maxv_from_max_alat;
    end
end


%  --rate limit the velocity waypoints based on longitudinal acceleration
%    along = (delta_v/delta_t) = (delta_v/delta_s) * (delta_s/delta_t)
%          = (delta_v/delta_s) * velocity

%  --first rate limit for positive accelerations (dv/dt > 0)
for n = 2:N
    %  --get the change in velocity and the change in position
    delta_v = VWP(n) - VWP(n-1);
    delta_s = SWP(n) - SWP(n-1);
%     velocity = (VWP(n) + VWP(n-1)) / 2;
    velocity = VWP(n);

    %  --compute the maximum allowable positive change in velocity (rate limit)
    max_pos_delta_v = max_along * abs(delta_s) / velocity;

    %  --limit the positive change in velocity if too large
    if (delta_v > max_pos_delta_v)  % positive accelerations only!
        VWP(n) = VWP(n-1) + max_pos_delta_v;
        if (VWP(n) > vmax), VWP(n) = vmax; end
    end
end

%  --finally rate limit for negative accelerations (deceleration)
for n = N:-1:2
    %  --get the change in velocity and the change in position
    delta_v = VWP(n) - VWP(n-1);
    delta_s = SWP(n) - SWP(n-1);
%     velocity = (VWP(n) + VWP(n-1)) / 2;
    velocity = VWP(n-1);

    %  --compute the maximum allowable negative change in velocity (rate limit)
    max_neg_delta_v = - max_along * abs(delta_s) / velocity;

    %  --limit the negative change in velocity if too large
    if (delta_v < max_neg_delta_v)  % negative accelerations only!
        VWP(n-1) = VWP(n) - max_neg_delta_v;
        if (VWP(n-1) < 0), VWP(n-1) = 0; end
    end
end

return
