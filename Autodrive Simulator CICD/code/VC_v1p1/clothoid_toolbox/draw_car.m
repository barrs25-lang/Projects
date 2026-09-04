function draw_car(xcar,ycar,head,lookahead,lw)
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
%  This function draws a wireframe car on the current figure at the
%  desired pose specified by xcar (meters), ycar (meters), and head
%  (radians).  The lookahead (meters) input specifies the length of a
%  heading indicator starting at the center of the rear axle and pointing
%  forward.

%  Last updated:  1/5/2023

%  --define the local persistent variables and constants
persistent V0 C NC NV hcar

if (nargin < 5), lw = 0.75; end

if isempty(V0)
    %  --define the car vertices (arbitrary units)
    V0 = [-20	-8	% 1
          -16	-20	% 2
          -4	-24	% 3
          79	-25	% 4
          85	-22	% 5
          90	-12	% 6
          92	0	% 7
          90	12	% 8
          85	22	% 9
          79	25	% 10
          -4	24	% 11
          -16	20	% 12
          -20	8	% 13
          -7    -16 % 14
          46    -16 % 15
          70    -19 % 16
          73    -8  % 17
          73    8   % 18
          70    19  % 19
          46    16  % 20
          -7    16  % 21
          -10   8   % 22
          -11   0   % 23
          -10   -8  % 24
          -2    0   % 25
          2     0   % 26
          0     -2  % 27
          0     2   % 28
          0     0   % 29
          0     0   % 30
        ];
    
    NV = size(V0,1);
    
    %  --define the actual car dimensions (approximate)
    L = 4.306;   % length (m)
    W = 1.770;   % width (m)
    
    %  --scale the vertices with units of meters
    dL = max(V0(:,1)) - min(V0(:,1));  % total length
    dW = max(V0(:,2)) - min(V0(:,2));  % total width
    V0(:,1) = (L / dL) * V0(:,1);  % x-direction coordinates (meters)
    V0(:,2) = (W / dW) * V0(:,2);  % y-direction coordinates (meters)
    
    %  --insert the lookahead value
    V0(NV,1) = lookahead;
    
    %  --define the connector lines between vertices
    C = [1 2
        2 3
        3 4
        4 5
        5 6
        6 7
        7 8
        8 9
        9 10
        10 11
        11 12
        12 13
        13 1
        14 15
        15 16
        16 17
        17 18
        18 19
        19 20
        20 21
        21 22
        22 23
        23 24
        24 14
        15 20
        25 26
        27 28
        29 30   % the last connector is the lookahead vector
        ];
    
    %  --draw the initial car object
    carcolor = [138, 30, 65]/255;  % maroon
    
    NC = size(C,1);
    hcar = zeros(NC,1);
    for m = 1:NC
        %  --draw each connector of the car wireframe
        hcar(m) = line([V0(C(m,1),1),V0(C(m,2),1)], ...
                       [V0(C(m,1),2),V0(C(m,2),2)]);
        if (m < NC)
            %  --car frame
            set(hcar(m),'linewidth',lw,'color',carcolor);
        else
            %  --lookahead vector
            set(hcar(m),'linewidth',2*lw,'color',0.8*[1 0 0]);
        end
    end   
end


%  --insert the lookahead value
V0(NV,1) = lookahead;

%  --compute rotation matrix & transform the data
Rk = [cos(head) -sin(head); sin(head) cos(head)];

Vk = Rk*V0';
Xck = Vk(1,:);  % x-coord of car vertices at the desired pose
Yck = Vk(2,:);  % y-coord of car vertices at the desired pose

%  --translate car origin to current location
Xck = Xck + xcar;
Yck = Yck + ycar;

Vck = [Xck(:),Yck(:)];

%  --update the pose of the car on the figure
for m = 1:NC
    set(hcar(m),'xdata',[Vck(C(m,1),1),Vck(C(m,2),1)], ...
                'ydata',[Vck(C(m,1),2),Vck(C(m,2),2)]);
end

return
