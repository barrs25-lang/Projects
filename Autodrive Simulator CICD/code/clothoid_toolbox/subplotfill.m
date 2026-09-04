function h = subplotfill(nrows,ncols,index)
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

dx = 1/ncols;
dy = 1/nrows;

A = 1:nrows*ncols;
A = reshape(A,ncols,nrows)';

[row,col] = find(A == index);

x = (col - 1) * dx;
y = 1 - row * dy;

h = axes('outerposition',[x, y, dx, dy]);

return