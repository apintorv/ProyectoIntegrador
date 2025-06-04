%drive.m - does the dead rekoning kinematics based on wheel speeds
%  written by: Shawn Lankton
%  for: ECE8843 (sort-of) fun (mostly)
%
%  This predicts how the robot's position will change based on velocities
%  for the left and right wheel along with a 'time driving.'  This is based
%  on the simplest formulation for 2-wheeled kinematics that I could find.
%  http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
%
%  Inputs:
%    posn - [yposn, xposn, theta]
%    wdia - diameter of the robot's wheelbase
%    vL - left wheel velocity
%    vR - right wheel velocity
%    t - time driving at these velocities
%
%  Outputs:
%    newposn - [yposn, xposn, theta]
%

function [odometry,newposn] = Compute_Drive(posn, wdia, vL, vR, t)

% -----------------------------------------
% Get speed differences & sums
vdiff = vR-vL;
vsum = vR+vL;

% -----------------------------------------
% Calculate new angle (pretty simple)
newposn(3) = posn(3) + vdiff*t/wdia;

% -----------------------------------------
% 
if(vdiff == 0)
    %calculate new [y x] if wheels moving together.
    newposn(1) = vL*t*sin(posn(3))+posn(1);
    newposn(2) = -vR*t*cos(posn(3))+posn(2);  
else
    %calculate new [y x] if wheels moving at unequal speeds.
    newposn(1) = posn(1) - wdia*vsum/(2*vdiff)*(cos(vdiff*t/wdia+posn(3))-cos(posn(3)));
    newposn(2) = posn(2) + wdia*vsum/(2*vdiff)*(sin(vdiff*t/wdia+posn(3))-sin(posn(3)));
end

% -----------------------------------------
% Get odometry
odometry = newposn - posn;