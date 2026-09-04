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
%  This script auto-generates a Matlab m-file script that defines the upper
%  and lower limits for curvature rate as a function of target distance and
%  initial curvature.
%
%  This script does not call any external functions.
%
%  This script requires the kplimdata.mat file to exist on the path.
%
%  This script generates the following output:
%       kplim_include.m
%
%  where this include file is a text file containining lookup table data 
%  that is pre-formatted for insertion directly into get_kp_limit.m

clear, clc

%  --load the kp limit data

load kplimdata

NR = length(Rgrid);
NK = length(Kgrid);


%  --create the include file for writing

fid = fopen('kplim_include.m','w');


%  --write the grid dimensions
fprintf(fid,'\nNR = %d;\n',NR);
fprintf(fid,'\nNK = %d;\n',NK);

%  --write the target distance grid vector
fprintf(fid,'\nRgrid = [');
for n = 1:NR
    fprintf(fid,'%11.8e',Rgrid(n));
    if (n < NR), fprintf(fid,','); end
end
fprintf(fid,'];\n');

%  --write the initial curvature grid vector
fprintf(fid,'\nKgrid = [');
for n = 1:NK
    fprintf(fid,'%11.8e',Kgrid(n));
    if (n < NK), fprintf(fid,','); end
end
fprintf(fid,'];\n');

%  --write the positive curvature rate limit matrix
fprintf(fid,'\nKPpos = [\n    ');
for n = 1:NK
    for m = 1:NR
        fprintf(fid,'%11.8e,',KPpos(n,m));
    end
    fprintf(fid,'\n    ');
end
fprintf(fid,'    ];\n');

%  --write the hegative curvature rate limit matrix
fprintf(fid,'\nKPneg = [\n    ');
for n = 1:NK
    for m = 1:NR
        fprintf(fid,'%11.8e,',KPneg(n,m));
    end
    fprintf(fid,'\n    ');
end
fprintf(fid,'    ];\n');

%  --close the file
fclose(fid);

