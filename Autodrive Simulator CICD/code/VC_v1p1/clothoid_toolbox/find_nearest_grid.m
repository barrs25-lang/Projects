function index = find_nearest_grid( value, grid, N )
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
%  This function determines the index of the 1D grid point closest to the
%  input value.
%
%  This function does not call any external functions.

%  --start search at the first grid point
index = 1;

for i = 1:N
    %  --check to see if the input value is less than this grid point
    if (value <= grid(index))
        %  --found the interval so exit the for loop
        break
    else
        %  --interval not found so check the next grid point
        index = index + 1;
    end
end

%  --how to interpret the returned index value

%  (index == 1)     ->             value <= grid(1)
%  (index == 2)     ->   grid(1) < value <= grid(2) -> first interval
%  (index == 3)     ->   grid(2) < value <= grid(3) -> second interval
%       ...
%  (index == N)     -> grid(N-1) < value <= grid(N) -> (N-1)st interval
%  (index == (N+1)) ->   grid(N) < value

return

