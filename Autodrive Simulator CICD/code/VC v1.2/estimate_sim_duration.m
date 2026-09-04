function tfinal = estimate_sim_duration(VWP,SWP)
%  This function estimates the duration of an autonomous vehicle control
%  simulation based on a velocity waypoint vs. position profile.
%
%  Between any two waypoints (n-1, n) the velocity V(s) is assumed to be a
%  straight line segment such that:
%
%  SWP(n-1) <= s <= SWP(n)
%  V(s) = VWP(n-1) + a * (s - SWP(n-1))
%  a = (VWP(n) - VWP(n-1)) / (SWP(n) - SWP(n-1)) = deltaV/deltaS
%
%  A simpler form is:  V(s) = b + a * s
%  where:  b = VWP(n-1) - a * SWP(n-1)
%
%  The velocity is also defined by the derivative:  V = ds/dt
%  Rearranging this definition:  V * dT = ds  ->  dT = (1/V(s)) * ds
%
%  Integrating both sides over the waypoint interval:
%  int(dT,t(n-1),t(n)) = int((1/V(s)),SWP(n-1),SWP(n))
%
%  int(dT,t(n-1),t(n)) = (t(n) - t(n-1)) = duration of interval
%
%  int((1/V(s)),SWP(n-1),SWP(n)) = (log(a*SWP(n)+b)-log(a*SWP(n-1)+b)) / a
%
%  Define:  num = a*SWP(n) + b
%               = a*SWP(n) + VWP(n-1) - a*SWP(n-1)
%               = a*(SWP(n) - SWP(n-1)) + VWP(n-1)
%               = a*deltaS + VWP(n-1)
%
%           den = a*SWP(n-1) + b
%               = a*SWP(n-1) + VWP(n-1) - a*SWP(n-1)
%               = VWP(n-1)
%
%  int((1/V(s)),SWP(n-1),SWP(n)) = (1/a) * log(num/den)
%               = (1/a) * log(1 + a*deltaS/VWP(n-1))
%
%  The final time duration over one waypoint segment is:
%       tfinal = (1/a) * log(1 + a*deltaS/VWP(n-1))

%  --initialize variables

tfinal = 0;
vmin = 0.1;

NUMWP = length(VWP);


%  --accumulate the durations of each waypoint segment

for n = 2:NUMWP
    %  --prevent zero velocity (requires very long sim times)
    Vn = max(VWP(n), vmin);      % VWP(n)
    Vnm1 = max(VWP(n-1), vmin);  % VWP(n-1)


    %  --compute the changes in velocity and position
    deltaV = Vn - Vnm1;
    deltaS = SWP(n) - SWP(n-1);
    
    a = deltaV / deltaS;

    %  --compute the time duration of this waypoint segment
    if (a == 0)
        %  --velocity is constant
        deltaT = deltaS / Vnm1;
    else
        %  --velocity is linear, use integration result
        deltaT = (1/a) * log(1 + a*deltaS/Vnm1);
    end

    %  --accumulate the result
    tfinal = tfinal + deltaT;
end

return

