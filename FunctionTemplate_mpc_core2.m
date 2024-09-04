function param = mySetup(shape)
Ixx = load("UAV_NominalParameters.mat","IxxVal").IxxVal;
Iyy = load("UAV_NominalParameters.mat","IyyVal").IyyVal;
Izz = load("UAV_NominalParameters.mat","IzzVal").IzzVal;
b = load("UAV_NominalParameters.mat","bVal").bVal;
g = 9.81;
k = load("UAV_NominalParameters.mat","kVal").kVal;
l = load("UAV_NominalParameters.mat","lVal").lVal;
m = load("UAV_NominalParameters.mat","mVal").mVal;
omegaMax2 = load("UAV_NominalParameters.mat","omegaMax2Val").omegaMax2Val;

%%%%%%% Set to true to plot the generated path by 3D A* %%%%%%%%%%
param.plot = false;

param.Ixx = Ixx; param.Iyy = Iyy;
param.Izz = Izz;param.b = b;param.g = g;
param.k = k;param.l = l;param.m = m;
param.omegaMax2 = omegaMax2;
param.initialParams = [param.Ixx, param.Iyy, param.Izz, param.b, param.g, param.k, param.l, param.m,param.omegaMax2];


param.start = shape.start;  

param.target = shape.target; 
param.xTar = shape.target(1);
param.yTar = shape.target(2);
param.zTar = shape.target(3);

% Make the UAV go to (xTar, yTar,zTar)
r = zeros(12,1); 
r(1,1) = param.xTar; r(2,1) = param.yTar; r(3,1) = param.zTar;

param.targetMod = r;

param.constraints = shape.constraints;
param.hc = shape.constraints.rect.h; % height constraint 
param.boxc = shape.constraints.rect.bot; % box constraint

shape.constraints.rect; % The rectangle and ellipses for the current course
param.eps_t = shape.eps_t; % The target point tolerance

param.ThetaMax = shape.ThetaMax; 
param.Tt = shape.Tt; 
param.Te = shape.Te; 

param.nx = 12;
param.ny = 12;
param.nu = 4;

param.Ts = 0.14; % 降低 Ts 是有用的
param.N = ceil((param.Tt + param.Te)/param.Ts);


param.pathGuess = getPathGuess(shape,100, 1.1, param.N,param); 

% 黑魔法
param.magicfalg = 0; %  0 is off
%%%%%%%%%%%%%%%%%%%%%%%% nonlinear mpc %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

param.nlobjtv = nlmpc(param.nx,param.ny,param.nu);

param.nlobjtv.Model.StateFcn  = @(x,u) QuadrotorStateFcnBase(x,u,Ixx,Iyy,Izz,b,g,k,l,m,omegaMax2);
param.nlobjtv.Jacobian.StateFcn = @ (x,u) QuadrotorStateJacobianFcnBase(x,u,Ixx,Iyy,Izz,b,g,k,l,m,omegaMax2);

param.nlobjtv.Ts = param.Ts;
param.nlobjtv.PredictionHorizon = param.N; 
param.nlobjtv.ControlHorizon = param.N ;

%Limit all four control inputs to be in the range [0,1]
param.nlobjtv.MV = struct( Min={0;0;0;0}, Max={1;1;1;1}, RateMin={-1;-1;-1;-1}, RateMax={1;1;1;1} ); 

param.nlobjtv.OV = struct( ...
    Min = {-inf, -inf ,param.hc,-inf, -inf ,-inf, -inf ,-inf, -inf ,-inf, -inf ,-inf }, ...
    Max = {inf, inf , -0.01 ,inf, inf ,inf, inf ,inf, inf ,inf, inf ,inf });


param.nlobjtv.Weights.OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0]; % weights on states
param.nlobjtv.Weights.ManipulatedVariables = [0.1 0.1 0.1 0.1];    % weights on control moves
% param.nlobjtv.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];

param.nlobjtv.Optimization.SolverOptions.Algorithm = 'interior-point';
param.nlobjtv.Optimization.UseSuboptimalSolution = true;
param.nlobjtv.Optimization.SolverOptions.FunctionTolerance = 0.001;
param.nlobjtv.Optimization.SolverOptions.StepTolerance = 0.001;
param.nlobjtv.Optimization.SolverOptions.MaxIter = 550;
param.nlobjtv.Optimization.SolverOptions.ConstraintTolerance = 0.001;




 % validateFcns(param.nlobjtv,rand(param.nx,1),rand(param.nu,1));


end % End of mySetup

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function u = myMPController(param, y)

persistent it lastMv options MaxIter 

param.nlobjtv.Optimization.CustomIneqConFcn = @(X,U,e,data) myIneqConFunction(X,U,e,data,...
    param.constraints,param,y);

if isempty(it)
    param.magicfalg = 0;
    param.nlobjtv.Optimization.SolverOptions.Algorithm = 'interior-point';

    MaxIter = 25;
    options = nlmpcmoveopt; % warm start 
    it = 0;  lastMv = zeros(param.nu,1);
 
    firstStateGuess=zeros(param.N,12);
    for i = 1:param.N
        firstStateGuess(i,1)=param.pathGuess(i,1);
        firstStateGuess(i,2)=param.pathGuess(i,2);
        firstStateGuess(i,3)=param.pathGuess(i,3);
    end
    options.X0 = firstStateGuess;
    param.nlobjtv.Optimization.SolverOptions.MaxIter = MaxIter;

    [u,options,~] = nlmpcmove(param.nlobjtv,y,lastMv,param.targetMod.',[],options);
else

    if (it < param.N) 
        if param.N <= ceil(param.Te/param.Ts) + 2
            param.magicfalg = 0;
        else
            param.nlobjtv.Optimization.SolverOptions.Algorithm = 'sqp';

            param.magicfalg  = 1;
        end

        param.nlobjtv.Optimization.SolverOptions.MaxIter = max(5,MaxIter);
        param.nlobjtv.PredictionHorizon = param.N-it + 10;  param.nlobjtv.ControlHorizon = param.N-it + 10;
        [u,options,info] = nlmpcmove(param.nlobjtv,y,lastMv,param.targetMod.',[],options);
        conv=info.Iterations;
        it = it+1; lastMv = u; MaxIter = MaxIter -1;


    end
end
end

function cineq = myIneqConFunction(X,U,e,data,constraints,param,y)

p = data.PredictionHorizon;
posX = X(2:p+1,1);
posY = X(2:p+1,2);
posZ = X(2:p+1,3);

stepsLeft = size(X,1);

[DRect,clRect,chRect] = getRectConstraints(constraints.rect.bot);

% position Constraints on UAV
cineq = [
     posX*DRect(1,1)+posY*DRect(1,2)-chRect(1)-e;
    -posX*DRect(1,1)-posY*DRect(1,2)+clRect(1)-e;
     posX*DRect(2,1)+posY*DRect(2,2)-chRect(2)-e;
    -posX*DRect(2,1)-posY*DRect(2,2)+clRect(2)-e;
    -posZ + param.hc;
     posZ;
     ];

% elliptical cylinder constraint
for i = 1:length(constraints.ellipses)
         el = constraints.ellipses{i};
         elCon = min(-((((posX)-el.xc)/el.a).^2 + (((posY)-el.yc)/el.b).^2) + 1.2 + e, ...
                     posZ - el.h+ 0.01  + e);
         cineq = [cineq; elCon;]; 
end

%  tv 就是毒药，我们再用毒药解渴
% 注释掉这段2023b 能运行，跑出来啥样，最后在小盒子里面吗？
%  这段是需要的，但是启用 tv 的时候会不稳定有报错，那就
if param.magicfalg==0
    if param.N < ceil((param.Tt + param.Te)/param.Ts) - 1% 这里完全是魔法(用)
        % final box constraint
        for i= 1:min(ceil(param.Te/param.Ts + 2), stepsLeft-1)
            xFin = X(p+2-i,1:3);
            cineq=[ cineq;
                xFin' - param.targetMod(1:3) - param.eps_t*ones(3,1);
                -xFin' + param.targetMod(1:3)- param.eps_t*ones(3,1);
                ];
        end
    end
end
end











function [A,B,C] = getLineParamsStd(pointA,pointB)
% Get line parameters in standard form Ax + By-C = 0
A = pointA(2)-pointB(2);
B = pointB(1)-pointA(1);
C = A*pointA(1)+B*pointA(2);
end

function [DRect,clRect,chRect] = getRectConstraints(rect)

%Compute constraint on the cart based on rectangular shape
A = rect(1,:);
B = rect(2,:);
C = rect(3,:);
D = rect(4,:);

% First two parallel edges - computed in same direction
[a1,b1,c1] = getLineParamsStd(A,B);
[a2,b2,c2] = getLineParamsStd(D,C);
% Second set of parallel edges - computed in same direction
[a3,b3,c3] = getLineParamsStd(B,C);
[a4,b4,c4] = getLineParamsStd(A,D);

% Compute D matrix and upper/lower bounds
DRect=zeros(2,2);
DRect(1,1)=a1;
DRect(1,2)=b1;
DRect(2,1)=a3;
DRect(2,2)=b3;

clRect=[min(c2,c1);min(c4,c3)];
chRect=[max(c1,c2);max(c3,c4)];
end



function out1 = QuadrotorStateFcnBase(in1,in2,Ixx,Iyy,Izz,b,g,k,l,m,omegaMax2)
%QuadrotorStateFcnBase
%    OUT1 = QuadrotorStateFcnBase(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2024-01-31 18:10:05


input_1 = in2(1,:);
input_2 = in2(2,:);
input_3 = in2(3,:);
input_4 = in2(4,:);

p = in1(10,:);
phi = in1(4,:);
psi = in1(6,:);
q = in1(11,:);
r = in1(12,:);
theta = in1(5,:);
u = in1(7,:);
v = in1(8,:);
w = in1(9,:);
t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t9 = -Izz;
t11 = -input_2;
t12 = -input_3;
t13 = -input_4;
t14 = 1.0./m;
t15 = input_1+input_2+input_3+input_4;
t8 = r.*t2;
t10 = q.*t5;
t16 = t8+t10;
out1 = [u;v;w;p+t16.*tan(theta);q.*t2-r.*t5;t16./t4;-k.*omegaMax2.*t14.*t15.*(t5.*t6+t2.*t3.*t7);k.*omegaMax2.*t14.*t15.*(t3.*t5-t2.*t6.*t7);g-k.*omegaMax2.*t2.*t4.*t14.*t15;(q.*r.*(Iyy+t9)+k.*l.*omegaMax2.*(input_1+input_4+t11+t12))./Ixx;-(p.*r.*(Ixx+t9)-k.*l.*omegaMax2.*(input_1+input_2+t12+t13))./Iyy;-(b.*omegaMax2.*(input_1+input_3+t11+t13)-p.*q.*(Ixx-Iyy))./Izz];
end



function [A,B] = QuadrotorStateJacobianFcnBase(in1,in2,Ixx,Iyy,Izz,b,g,k,l,m,omegaMax2)
%QuadrotorStateJacobianFcnBase
%    [A,B] = QuadrotorStateJacobianFcnBase(IN1,IN2,IN3)
%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2024-02-02 17:47:52
input_1 = in2(1,:);
input_2 = in2(2,:);
input_3 = in2(3,:);
input_4 = in2(4,:);
p = in1(10,:);
phi = in1(4,:);
psi = in1(6,:);
q = in1(11,:);
r = in1(12,:);
theta = in1(5,:);
t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t8 = tan(theta);
t11 = -Iyy;
t12 = -Izz;
t13 = 1.0./Ixx;
t14 = 1.0./Iyy;
t15 = 1.0./Izz;
t18 = 1.0./m;
t19 = input_1+input_2+input_3+input_4;
t9 = q.*t2;
t10 = r.*t2;
t16 = q.*t5;
t17 = r.*t5;
t20 = t3.*t5;
t21 = t5.*t6;
t22 = 1.0./t4;
t23 = Ixx+t11;
t24 = Ixx+t12;
t25 = Iyy+t12;
t27 = b.*omegaMax2.*t15;
t28 = t2.*t3.*t7;
t29 = t2.*t6.*t7;
t30 = k.*l.*omegaMax2.*t13;
t31 = k.*l.*omegaMax2.*t14;
t37 = k.*omegaMax2.*t2.*t4.*t18;
t26 = -t17;
t32 = t10+t16;
t33 = -t27;
t34 = -t29;
t35 = -t30;
t36 = -t31;
t39 = t21+t28;
t40 = -t37;
t38 = t9+t26;
t41 = t20+t34;
t42 = k.*omegaMax2.*t18.*t39;
t43 = k.*omegaMax2.*t18.*t41;
t44 = -t42;
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t8.*t38,-t32,t22.*t38,-k.*omegaMax2.*t18.*t19.*(t2.*t6-t7.*t20),k.*omegaMax2.*t18.*t19.*(t2.*t3+t7.*t21),k.*omegaMax2.*t4.*t5.*t18.*t19,0.0,0.0,0.0,0.0,0.0,0.0,t32.*(t8.^2+1.0),0.0,t7.*t22.^2.*t32,t3.*t19.*t40,t6.*t19.*t40,k.*omegaMax2.*t2.*t7.*t18.*t19,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t19.*t43,t19.*t44,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,-r.*t14.*t24,q.*t15.*t23,0.0,0.0,0.0];
mt2 = [t5.*t8,t2,t5.*t22,0.0,0.0,0.0,r.*t13.*t25,0.0,p.*t15.*t23,0.0,0.0,0.0,t2.*t8,-t5,t2.*t22,0.0,0.0,0.0,q.*t13.*t25,-p.*t14.*t24,0.0];
A = reshape([mt1,mt2],12,12);
if nargout > 1
    B = reshape([0.0,0.0,0.0,0.0,0.0,0.0,t44,t43,t40,t30,t31,t33,0.0,0.0,0.0,0.0,0.0,0.0,t44,t43,t40,t35,t31,t27,0.0,0.0,0.0,0.0,0.0,0.0,t44,t43,t40,t35,t36,t33,0.0,0.0,0.0,0.0,0.0,0.0,t44,t43,t40,t30,t36,t27],[12,4]);
end
end


function map = getMap3D(constraints,res,e)
    map = false(res,res,res); % false = 0, true = 1: is obstacle
    Obstacle = @(x,y,z) isObstacle(x,y,z,constraints,e);
    for x = 1:size(map,1)
        for y = 1:size(map,2)
            for z = 1:size(map,3)
               xt =  1 - (2/res)*x;
               yt =  1 -(2/res)*y;
               zt =  1 -(2/res)*z;
               map(x,y,z) = Obstacle(xt,yt,zt);
        end
    end
    end
end

function c = isObstacle(x,y,z,constraints,e)
 % 传进来一个点(x,y,z)，判断是不是障碍点
 % 在大盒子外面，也被视为障碍点
 [DRect,clRect,chRect] = getRectConstraints(constraints.rect.bot); 

 c1 = (x*DRect(1,1)+y*DRect(1,2)-chRect(1))>0;
 c2 = (-x*DRect(1,1)-y*DRect(1,2)+clRect(1))>0;
 c3 = (x*DRect(2,1)+y*DRect(2,2)-chRect(2))>0;
 c4 = (-x*DRect(2,1)-y*DRect(2,2)+clRect(2))>0;

 hc = constraints.rect.h;
 c5 = -z +  hc > 0;

 c= c1||c2||c3|| c4 ||c5; % 有一个约束不满足，则在大盒子外面

     % 投影椭圆内，且高度也在椭圆内部, 则被视为障碍点
     for i = 1:length(constraints.ellipses)
           el = constraints.ellipses{i};
           ec = ((-(((x-el.xc)/el.a).^2 + ((y-el.yc)/el.b).^2) + 1*e )>0 && (z - el.h*e >0));
           c = c || ec;
     end 
end



% ---------get the 3D map for clear plot purpose --------------%

function map = getMap3D_plot(constraints,res,e)
    map = false(res,res,res); % false = 0, true = 1: is obstacle
    Obstacle = @(x,y,z) isObstacle_plot(x,y,z,constraints,e);
    for x = 1:size(map,1)
        for y = 1:size(map,2)
            for z = 1:size(map,3)
               xt =  1 - (2/res)*x;
               yt =  1 -(2/res)*y;
               zt =  1 -(2/res)*z;
               map(x,y,z) = Obstacle(xt,yt,zt);
        end
    end
    end
end

function c = isObstacle_plot(x,y,z,constraints,e)

 c = false;


     for i = 1:length(constraints.ellipses)
           el = constraints.ellipses{i};
           ec = ((-(((x-el.xc)/el.a).^2 + ((y-el.yc)/el.b).^2) +1)>0 && (z - el.h>0));
           c = c || ec;
     end 
end
% ----end-----get the 3D map for clear plot purpose --------------%


function pt = resamplePath3D(t, px, py, pz)
% Ensure inputs are column vectors
t = t(:);
px = px(:);
py = py(:);
pz = pz(:);

if numel(t) == 1 && t > 1 && rem(t,1) == 0
    t = linspace(0,1,t)';
elseif any(t < 0) || any(t > 1)
    error('All elements of t must be 0 <= t <= 1');
end

nt = numel(t);
n = numel(px);

% Combine coordinates into a single matrix for processing
pxyz = [px, py, pz];
ndim = 3; % Now working in 3 dimensions
pt = NaN(nt, ndim);

% Calculate chord lengths and cumulative arc lengths for interpolation
chordlen = sqrt(sum(diff(pxyz, [], 1).^2, 2));
chordlen = chordlen / sum(chordlen);
cumarc = [0; cumsum(chordlen)];

% Determine bins for interpolation
[junk, tbins] = histc(t, cumarc);
tbins((tbins <= 0) | (t <= 0)) = 1;
tbins((tbins >= n) | (t >= 1)) = n - 1;

% Perform interpolation
s = (t - cumarc(tbins)) ./ chordlen(tbins);
pt = pxyz(tbins, :) + (pxyz(tbins + 1, :) - pxyz(tbins, :)) .* repmat(s, 1, ndim);
end



function pathGuess = getPathGuess(shape,res, e, predictionHorizon,param)
    %  Constraints is the constraint datastructure in shape 
    %  res is the resolution for the discretization of the constraints (higher is better but longer processing time)
    %  e is by how much the constraint are enlarged [1 2]
    %  prediction horizon is how many steps the guess has
    
    constraints = shape.constraints;
    input_map = getMap3D(constraints,res,e); % input map 就是障碍物地图

    %Start Positions
    StartX=floor((1-shape.start(1))*(res/2));
    StartY=floor((1-shape.start(2))*(res/2));
    StartZ=floor((1-shape.start(3))*(res/2));
    start_coords = [StartX,StartY,StartZ];

    EndX=floor((1-shape.target(1))*(res/2));
    EndY=floor((1-shape.target(2))*(res/2));
    EndZ=floor((1-shape.target(3))*(res/2));
    dest_coords = [EndX,EndY,EndZ];

    [route, numExpanded] = AStarGrid3D(input_map, start_coords, dest_coords);

    if route ~= false % 找到路径了
    [x, y, z] = ind2sub(size(input_map), route);
    OptimalPath = [x', y', z'];
    % Convert discrete map to continous values
    pathGuess = zeros(size(OptimalPath));
    for i=1:size(pathGuess,1)
        pathGuess(i,1) = 1-OptimalPath(i,1)*(2/res);
        pathGuess(i,2) = 1-OptimalPath(i,2)*(2/res);
        pathGuess(i,3) = 1-OptimalPath(i,3)*(2/res);
    end

    % Resample map to prediction horizon
    pathGuess = resamplePath3D(predictionHorizon,pathGuess(:,1),pathGuess(:,2),pathGuess(:,3));
    % pathGuess

    else
        pathGuess = zeros(predictionHorizon,12);
    end

    plot = param.plot;
    if plot
    %--- For clear plot purpose, z-axis is upside done by multiplying -1 ---%
        figure
        % axes1 = axes('Parent',figure);
        input_map_plot = getMap3D_plot(constraints,res,e);
        [obstacleX, obstacleY, obstacleZ] = ind2sub(size(input_map), find(input_map_plot));
        % Convert grid indices to Cartesian coordinates
        xt = 1 - (2/res) * obstacleX;
        yt = 1 - (2/res) * obstacleY;
        zt = 1 - (2/res) * obstacleZ;
        scatter3(xt, yt, -zt, 'filled', 'MarkerFaceColor', 'k');

        hold on
        % plot the palnned path
        path = scatter3(pathGuess(:,1),pathGuess(:,2),-pathGuess(:,3),40, 'filled', 'MarkerFaceColor', 'r');
        hold on

        % Plot start and end points
        start = scatter3(shape.start(1), shape.start(2), -shape.start(3), 50, 's', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');
        hold on
        endPoint = scatter3(shape.target(1), shape.target(2), -shape.target(3), 50, 'o', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
        legend([start, endPoint,path], {'Start Point', 'End Point','Path Generated by 3D A*'},'Location', 'northeast');
        zlim([0,-param.hc]);

        % view(axes1,[-152.300000425828 36.0058365660954]);
        % legend1 = legend(axes1,'show');
        % set(legend1,...
        %     'Position',[0.484054214615229 0.657539682539683 0.3984375 0.121428571428571],...
        %     'LineWidth',1);
        % exportgraphics(gcf,'./figures/astar_path.png','Resolution',800);



    end

       
end

function [route, numExpanded] = AStarGrid3D(input_map, start_coords, dest_coords)
    % Initialize variables
    [nrows, ncols, ndepths] = size(input_map);
    map = zeros(nrows, ncols, ndepths);
    map(~input_map) = 1;   % Mark free cells
    map(input_map) = 2;    % Mark obstacle cells
    start_node = sub2ind(size(map), start_coords(1), start_coords(2), start_coords(3));
    dest_node = sub2ind(size(map), dest_coords(1), dest_coords(2), dest_coords(3));
    parent = zeros(nrows, ncols, ndepths);

    % Initialize open list and closed list
    openList = containers.Map('KeyType','double','ValueType','any');
    closedList = containers.Map('KeyType','double','ValueType','any');
    openList(start_node) = true; % The value is arbitrary here, we just need to keep track of the keys

    % Heuristic function: Euclidean distance for 3D
    [X, Y, Z] = ndgrid(1:nrows, 1:ncols, 1:ndepths);
    H = sqrt((X - dest_coords(1)).^2 + (Y - dest_coords(2)).^2 + (Z - dest_coords(3)).^2);

    % Initialize cost arrays
    f = Inf(nrows, ncols, ndepths);
    g = Inf(nrows, ncols, ndepths);
    g(start_node) = 0;
    f(start_node) = H(start_node);

    % Main Loop
    numExpanded = 0;
    while ~isempty(openList)
        % Find the node with the minimum f value
        [~, current] = min(f(:));
        
        if (current == dest_node) || isinf(f(current))
            break;
        end;
        
        % Remove current node from open list and add it to the closed list
        openList.remove(current);
        closedList(current) = true; % Again, value is arbitrary
        f(current) = Inf; % Remove this node from further consideration
        numExpanded = numExpanded + 1;

        % Compute 3D coordinates of current node
        [i, j, k] = ind2sub(size(f), current);

        % Visit all of the neighbors around the current node and update the
        % entries in the f, g, and parent arrays
        neighbors = AStarNeighbors3D(map, i, j, k, closedList);
        
        for neighbor = neighbors
            if closedList.isKey(neighbor)
                continue;
            end
            
            % Compute tentative g score for neighbor
            tentative_gScore = g(current) + 1;

            if (~isKey(openList, neighbor)) || (tentative_gScore < g(neighbor))
                % This path to neighbor is better than any previous one. Record it!
                g(neighbor) = tentative_gScore;
                f(neighbor) = g(neighbor) + H(neighbor);
                parent(neighbor) = current;

                % If neighbor is not in open list, add it
                if (~openList.isKey(neighbor))
                    openList(neighbor) = true;
                end
            end
        end
    end

    % Construct route from start to dest by following the parent links
    if isinf(f(dest_node))
        route = false;
        disp('No path found.');
    else
        route = dest_node;
        
        while parent(route(1)) ~= 0
            route = [parent(route(1)), route];
        end
        
        disp(['Path found.']);
    end

end


function neighbors = AStarNeighbors3D(map, i, j, k, closedList)
% Find the neighbors of cell (i, j, k) on a 3D grid
[nrows, ncols, ndepths] = size(map);
neighbors = [];
directions = [1, 0, 0; -1, 0, 0; 0, 1, 0; 0, -1, 0; 0, 0, 1; 0, 0, -1];

for d = 1:size(directions, 1)
    ni = i + directions(d, 1);
    nj = j + directions(d, 2);
    nk = k + directions(d, 3);
    if ni > 0 && ni <= nrows && nj > 0 && nj <= ncols && nk > 0 && nk <= ndepths
        neighborIndex = sub2ind(size(map), ni, nj, nk);
        if map(ni, nj, nk) ~= 2 && ~isKey(closedList, neighborIndex) % If not an obstacle and not in closedList
            neighbors(end+1) = neighborIndex;
        end
    end
end

end







function [A,B] = QuadrotorStateJacobianFcnBase_extended(in1,in2)
%QuadrotorStateJacobianFcnBase_extended
%    [A,B] = QuadrotorStateJacobianFcnBase_extended(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2024-02-26 20:57:10

Ixx = in1(13,:);
Iyy = in1(14,:);
Izz = in1(15,:);
b = in1(19,:);
input_1 = in2(1,:);
input_2 = in2(2,:);
input_3 = in2(3,:);
input_4 = in2(4,:);
k = in1(16,:);
l = in1(17,:);
m = in1(18,:);
omegaMax2 = in1(21,:);
p = in1(10,:);
phi = in1(4,:);
psi = in1(6,:);
q = in1(11,:);
r = in1(12,:);
theta = in1(5,:);
t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t8 = tan(theta);
t11 = -Iyy;
t12 = -Izz;
t13 = 1.0./Ixx;
t14 = 1.0./Iyy;
t15 = 1.0./Izz;
t18 = -input_2;
t19 = -input_3;
t20 = -input_4;
t21 = 1.0./m;
t23 = input_1+input_2+input_3+input_4;
t9 = q.*t2;
t10 = r.*t2;
t16 = q.*t5;
t17 = r.*t5;
t22 = t21.^2;
t24 = t3.*t5;
t25 = t5.*t6;
t26 = 1.0./t4;
t27 = Ixx+t11;
t28 = Ixx+t12;
t29 = Iyy+t12;
t31 = b.*omegaMax2.*t15;
t32 = p.*q.*t15;
t33 = p.*r.*t14;
t34 = q.*r.*t13;
t35 = t2.*t3.*t7;
t36 = t2.*t6.*t7;
t37 = k.*l.*omegaMax2.*t13;
t38 = k.*l.*omegaMax2.*t14;
t44 = k.*omegaMax2.*t2.*t4.*t21;
t46 = input_1+input_2+t19+t20;
t47 = input_1+input_3+t18+t20;
t48 = input_1+input_4+t18+t19;
t30 = -t17;
t39 = t10+t16;
t40 = -t31;
t41 = -t36;
t42 = -t37;
t43 = -t38;
t49 = t25+t35;
t50 = -t44;
t45 = t9+t30;
t51 = t24+t41;
t52 = k.*omegaMax2.*t21.*t49;
t53 = k.*omegaMax2.*t21.*t51;
t54 = -t52;
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t8.*t45,-t39,t26.*t45,-k.*omegaMax2.*t21.*t23.*(t2.*t6-t7.*t24),k.*omegaMax2.*t21.*t23.*(t2.*t3+t7.*t25),k.*omegaMax2.*t4.*t5.*t21.*t23,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t39.*(t8.^2+1.0),0.0,t7.*t26.^2.*t39,t3.*t23.*t50,t6.*t23.*t50,k.*omegaMax2.*t2.*t7.*t21.*t23,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t23.*t53,t23.*t54,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0];
mt2 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,-r.*t14.*t28,q.*t15.*t27,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t5.*t8,t2,t5.*t26,0.0,0.0,0.0,r.*t13.*t29,0.0,p.*t15.*t27,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t2.*t8,-t5,t2.*t26,0.0,0.0,0.0,q.*t13.*t29,-p.*t14.*t28,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t13.^2.*(q.*r.*t29+k.*l.*omegaMax2.*t48),-t33,t32,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt3 = [0.0,t34,t14.^2.*(p.*r.*t28-k.*l.*omegaMax2.*t46),-t32,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t34,t33,t15.^2.*(b.*omegaMax2.*t47-p.*q.*t27),0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-omegaMax2.*t21.*t23.*t49,omegaMax2.*t21.*t23.*t51,-omegaMax2.*t2.*t4.*t21.*t23,l.*omegaMax2.*t13.*t48,l.*omegaMax2.*t14.*t46,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,k.*omegaMax2.*t13.*t48,k.*omegaMax2.*t14.*t46,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,k.*omegaMax2.*t22.*t23.*t49,-k.*omegaMax2.*t22.*t23.*t51,k.*omegaMax2.*t2.*t4.*t22.*t23,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-omegaMax2.*t15.*t47];
mt4 = [0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-k.*t21.*t23.*t49,k.*t21.*t23.*t51,-k.*t2.*t4.*t21.*t23,k.*l.*t13.*t48,k.*l.*t14.*t46,-b.*t15.*t47,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0];
A = reshape([mt1,mt2,mt3,mt4],21,21);
if nargout > 1
    B = reshape([0.0,0.0,0.0,0.0,0.0,0.0,t54,t53,t50,t37,t38,t40,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t54,t53,t50,t42,t38,t31,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t54,t53,t50,t42,t43,t40,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t54,t53,t50,t37,t43,t31,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[21,4]);
end
end


function [xhat, Pnext] = ExtendedKalmanFilter(y, u, xhat, P, dt)
    % y: k
    % u, xhat: k-1

    % process noise, covariance matrix
    Q = diag([0* ones(1,12),10*ones(1,9)]);
    measNoiseVar = 1;
    R = measNoiseVar * eye(12);

    % Compute the Jacobian matrices A and B using your function
    [A, B] = QuadrotorStateJacobianFcnBase_extended(xhat, u);

    % Prediction Step
    xhatPred = xhat + dt * (A * xhat + B * u);

    F = eye(size(A)) + A*dt; 
    PPred = F * P * F' + Q;  

    % Measurement Update 
    ymeas = y;
    H = [eye(12), zeros(12, 9)];  % Measurement matrix
   
    hWithoutNoise = H * xhatPred; 
    Kgain = PPred * H' / (H * PPred * H'); 
    xhat = xhatPred + Kgain * (ymeas - hWithoutNoise); 
    Pnext = (eye(21) - Kgain * H) * PPred;

end





