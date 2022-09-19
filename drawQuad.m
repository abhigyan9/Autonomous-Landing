
function drawQuad(uu,V,F,patchcolors)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

    % define persistent variables 
    % persistent variables used within functions, but retained in memory
    % outside of it as well
    persistent vehicle_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        [Vertices,Faces,facecolors] = defineVehicleBody;
        vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis(1*[-1,1,-1,1,-1,1]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawVehicleBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           vehicle_handle);
    end
end

  
%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%

function handle = drawVehicleBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V, phi, theta, psi);  % rotate vehicle
  V = translate(V, pn, pe, pd);  % translate vehicle
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = R*V;
  
  if isempty(handle),
  handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody

% Define the vertices (physical location of vertices

    a = 0.25*0.035;   % core dimension
    da = 0.035;  % arm diameter
    at = 0.1*0.035;  % arm thickness
    rt = 0.2*0.035;   % rotor thickness
    dri = 0.05*0.035;     % rotor dia (circrumscribing circle)
    dro = 0.25*0.035;
    V_core = 0.5*a*[1,1,-1;-1,1,-1;-1,-1,-1;1,-1,-1;1,1,1;-1,1,1;-1,-1,1;1,-1,1];
    V_arm1 = 0.5*[da,at,at;da,-at,at;da,-at,-at;da,at,-at;-da,at,at;-da,-at,at;-da,-at,-at;-da,at,-at];
    V_arm2 = 0.5*[-at,da,at;at,da,at;at,da,-at;-at,da,-at;-at,-da,at;at,-da,at;at,-da,-at;-at,-da,-at];
    V_r1 = 0.5*[da+0.5*sqrt(3)*(dro+dri),dri*sin(pi/6),rt;da+0.5*sqrt(3)*(dro),dri,rt;da+0.5*sqrt(3)*(dro-dri),dri*sin(pi/6),rt;...
        da+0.5*sqrt(3)*(dro-dri),-dri*sin(pi/6),rt;da+0.5*sqrt(3)*(dro),-dri,rt;da+0.5*sqrt(3)*(dro+dri),-dri*sin(pi/6),rt;...
        da+sqrt(3)*dro,dro*sin(pi/6),rt;da+0.5*sqrt(3)*dro,dro,rt;da,dro*sin(pi/6),rt;...
        da,-dro*sin(pi/6),rt;da+0.5*sqrt(3)*dro,-dro,rt;da+sqrt(3)*dro,-dro*sin(pi/6),rt;...
        da+0.5*sqrt(3)*(dro+dri),dri*sin(pi/6),-rt;da+0.5*sqrt(3)*(dro),dri,-rt;da+0.5*sqrt(3)*(dro-dri),dri*sin(pi/6),-rt;...
        da+0.5*sqrt(3)*(dro-dri),-dri*sin(pi/6),-rt;da+0.5*sqrt(3)*(dro),-dri,-rt;da+0.5*sqrt(3)*(dro+dri),-dri*sin(pi/6),-rt;...
        da+sqrt(3)*dro,dro*sin(pi/6),-rt;da+0.5*sqrt(3)*dro,dro,-rt;da,dro*sin(pi/6),-rt;...
        da,-dro*sin(pi/6),-rt;da+0.5*sqrt(3)*dro,-dro,-rt;da+sqrt(3)*dro,-dro*sin(pi/6),-rt];
    V_r2 = ([0,-1,0;1,0,0;0,0,1]*V_r1')';
    V_r3 = ([0,-1,0;1,0,0;0,0,1]*V_r2')';
    V_r4 = ([0,-1,0;1,0,0;0,0,1]*V_r3')';
    % rotor1 rotated thrice to make 4 rotors in total
    V = [V_core;V_arm1;V_arm2;V_r1;V_r2;V_r3;V_r4]';

% define faces as a list of vertices numbered above

  F_core = [1,2,3,4;1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8;5,6,7,8];
  F_arm1 = F_core + 8;  % faces list for arm 1
  F_arm2 = F_core + 16; % faces list for arm 2
  F_r1 = [1,2,14,13;2,3,15,14;3,4,16,15;4,5,17,16;5,6,18,17;6,1,13,18;7,8,20,19;8,9,21,20;9,10,22,21;10,11,23,22;11,12,24,23;12,7,19,24;...
      1,14,20,7;2,15,21,8;3,16,22,9;4,17,23,10;5,18,24,11;6,13,19,12] + 24;
  F_r2 = F_r1 + 24;
  F_r3 = F_r2 + 24;
  F_r4 = F_r3 + 24;
  F = [F_core;F_arm1;F_arm2;F_r1;F_r2;F_r3;F_r4];

% define colors for each face    
  red = [1, 0, 0];
  green = [0, 1, 0];
  blue = [0, 0, 1];
  yellow = [1, 1, 0];
  cyan = [0, 1, 1];
%{
  facecolors = [...
    green;...    % left wing
    green;...    % right wing
    red;
    blue;...     % tail
    yellow;
    cyan;
    ];
  %}
  red_rotors = repmat(cyan,18*4,1);
  facecolors = [green;green;green;green;green;green;red;red;red;red;red;red;red;red;red;red;red;red;red_rotors];

end
  