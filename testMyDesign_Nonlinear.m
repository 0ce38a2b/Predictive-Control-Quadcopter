%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This file will run a complete simulation of the nonlinear UAV model
% with the designed controller in the file `myMPController.m`.
%
% Authors: Lucian Nita & Ian McInerney
% Revision: 2024.01
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear variables
close all
currentFolder=pwd;
addpath(strcat(currentFolder,'/Model'),strcat(currentFolder,'/HelperFunctions'))
 
partNum = 2; % The part of the core coursework that is being worked on
useSecondEllipse = 1; % Introduce a second ellipse to the course for part 2

%% Create the shape to test on
testCourse = defaultCourse( 0, partNum );

% Only for part 2 - add a second ellipse
if( partNum == 2 && useSecondEllipse == 1 )
    % Define the new ellipse
    ellipse.h  = -0.1;
    ellipse.a  = 0.3;
    ellipse.b  = 0.15; % 0.2 用于测试 2d 无解情况
    ellipse.xc = 0.15;
    ellipse.yc = 0.25;
    testCourse.shape.constraints.ellipses{2} = ellipse;
end

%% Load the parameters for the Simulation model & Perturb the parameters
% The following parameters could all be perturbed:
% IxxVal first diagonal element in the intertia matrix
% IyyVal second diagonal element in the intertia matrix
% IzzVal third diagonal element in the intertia matrix
% kVal rotor lift constant
% lVal moment arm between rotor and centre of mass
% mVal total UAV mass
% bVal rotor drag constant
% omegaMax2Val square of the maximum rotational speed for one propellor  
load('UAV_NominalParameters.mat');

UAVParams.IxxVal       = testCourse.perturb.IxxVal       * IxxVal;
UAVParams.IyyVal       = testCourse.perturb.IyyVal       * IyyVal;
UAVParams.IzzVal       = testCourse.perturb.IzzVal       * IzzVal;
UAVParams.kVal         = testCourse.perturb.kVal         * kVal;
UAVParams.lVal         = testCourse.perturb.lVal         * lVal;
UAVParams.mVal         = testCourse.perturb.mVal         * mVal;
UAVParams.bVal         = testCourse.perturb.bVal         * bVal;
UAVParams.omegaMax2Val = testCourse.perturb.omegaMax2Val * omegaMax2Val;

%% Extract the student functions
extractFunctions( 'FunctionTemplate_mpc_core26.m', 1 );

%% Call the setup function for the student
tic;
param = mySetup( testCourse.shape );
setupTime = toc;

%% Declare other simulation parameters & Setup the simulation
T = 6;           % Set the simulation final time
tstep = 0.001;   % Step size for simulation. Has to be less than minimum allowed Ts

if( isfield( param, 'Ts' ) )
    Ts=param.Ts;
else
    error('Sampling time not set: Field Ts nonexistent in param. Please set param.Ts')
end
if ~isa(Ts,"double")
    error( 'Ts must be of numeric data type: double' )
else
    if ~all(size(Ts)==[1,1])
        error( 'Ts must be a scalar' )
    else
        if( ( Ts < 0.01 ) || ( Ts > 1 ) )
            error( 'Ts must be in the interval [0.01, 1]' )
        end
    end
end

hw = waitbar( 0,'Please wait...' ); % create waiting bar
warning( 'on' );

odeOpts = odeset( 'RelTol', 1e-3, 'MaxStep', 0.001 ); % ODE Options

x = [ testCourse.shape.start(1,1), testCourse.shape.start(1,2), testCourse.shape.start(1,3), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; % Initial conditions [x,y,z,phi,theta,psi,u,v,w,p,q,r,Internal variable];
y    = x(1:12)';              %Initial measurement signal

% Setup variables to hold the results
time    = 0;
inputs  = [];
states  = x;
allContTime = []; % Variable to hold optimization time

%% Iterate for the simulation time
for t=0:Ts:T 
    waitbar( t/T, hw, 'Please wait...' );
    
    tic;
    % Call the controller function
    u = myMPController(param,y);
    contTime = toc;
    
    % Saturate the inputs to [0, 1] for the simulation
    usat = min( max( u, 0 ), 1 );
    
    % Setup the simulation model & Simulate
    mod = @(t, state) UAV_nl_model( usat, state, UAVParams );
    [tt, x] = ode23t( mod, t:tstep:t + Ts, x(end,:), odeOpts );
    
    % The output is simply all the states
    y = x(end,1:12)';

    % Keep variables for analysis
    % We ignore the first row to avoid data overlap
    time    = [time;    tt(2:end, end)];
    states  = [states;  x(2:end, :)];
    inputs  = [inputs;  u'.*ones( size( tt, 1 ) - 1, 1 ) ];
    
    % Save the computation time for analysis
    allContTime = [allContTime; contTime];
end
close(hw);

compTime.setup=setupTime;
compTime.MPC  =allContTime;
%% Visualize the Course
[~, ~, ~, text] = analyzeCourse( [], time, states, inputs, compTime, Ts, testCourse, 1 );
fprintf(text)
