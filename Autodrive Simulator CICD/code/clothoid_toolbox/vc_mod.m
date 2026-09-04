function z = vc_mod(x,y)
%
%  Alternative to the Matlab mod(x,y) function that does not require a
%  divide and can be converted to C or C++ properly.
%
%  WARNING:  This function will NOT handle vector inputs!  It was designed
%  for real-time implementation where it would ONLY see scalar inputs.
%
%  Input x can be positive or negative
%  Output is always bounded by:  0 <= z <= y

%  Last Updated:  1/8/2023

%  --determine the sign of the input data
x_is_negative = false;
if ( x < 0 )
    %  --input x is negative
    x_is_negative = true;
    x = - x;
end


%  --find the interval n
n = 1;
while ( x >= n*y )
    n = n + 1;
end
n = n - 1;


%  --get the positive remainder
z = x - n*y;


%  --adjust for negative values
if ( x_is_negative )
    if ( x == n*y )
        z = - z;
    else
        z = y - z;
    end
end

return


