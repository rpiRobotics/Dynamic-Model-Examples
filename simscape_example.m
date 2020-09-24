% Simscape Model 

clear variables; close all; clc

% Define model parameters

% Arm kinematics
x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1];
H = [[0;0;1],[0;1;0],[0;0;1]];
d1 = 0.35; d2 = 0.1; d3 = 0.25;
P = [d1*z0, zeros(3,1), d2*y0, d3*z0];

% Link mass-inertia
m = [3; 1.1; 2.4];

Ic = zeros(3,3,3); % inertia tensors at link COM in PoE frame
Ic(:,:,1) = [3.75, 0.9, 0.6; 0.9, 4.1, 0.11; 0.6, 0.11, 1];
Ic(:,:,2) = [1.86, 0.11, 0.05; 0.11, 2.89, 0.08; 0.05, 0.08, 2.91];
Ic(:,:,3) = [1.07, 0.5, 0.34; 0.5, 2.19, 0.03; 0.34, 0.03, 4.16];
% NOTE: assumes that N^2 Im term has already been added
% Modified to match Simscape frames:
Isim = Ic;
Isim(:,:,2) = [1,0,0;0,0,1;0,-1,0]'*Isim(:,:,2)*[1,0,0;0,0,1;0,-1,0];

% COM measured from respective PoE frame
c = [[0.01;-0.05;-0.14],[0.03;0.045;-0.01],[-0.02;-0.01;0.13]]; 
% Modified to match Simscape frames:
csim = c;
csim(:,2) = [1,0,0;0,0,1;0,-1,0]'*csim(:,2);

% link friction (linear)
bl = [0.2;0.2;0.2];

% Simulation parameters
g = [0;0;-9.81]; % gravity vector
q0 = [0;0;0]; % initial joint position
qdot0 = [0;0;0]; % initial joint speed
T = 30; % simulation time
taustep = [0;0;0];

% Visualization parameters
r = 0.05;

% Run simulation
sim('simscape_3link_sim.slx')

