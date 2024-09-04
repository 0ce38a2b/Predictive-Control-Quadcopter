function param = mySetup(shape)
param.Ixx = load("UAV_NominalParameters.mat","IxxVal").IxxVal;
param.Iyy = load("UAV_NominalParameters.mat","IyyVal").IyyVal;
param.Izz = load("UAV_NominalParameters.mat","IzzVal").IzzVal;
param.b = load("UAV_NominalParameters.mat","bVal").bVal;
param.g = 9.81;
param.k = load("UAV_NominalParameters.mat","kVal").kVal;
param.l = load("UAV_NominalParameters.mat","lVal").lVal;
param.m = load("UAV_NominalParameters.mat","mVal").mVal;
param.omegaMax2 = load("UAV_NominalParameters.mat","omegaMax2Val").omegaMax2Val;

%%%%%%% Set to true to plot the generated path by 3D A* %%%%%%%%%%
param.plot = false;

u_e = param.m* param.g/(4* param.k* param.omegaMax2); % equlibrium input
param.u_eq = u_e;

param.shape = shape;
param.Ts = 0.14;  
param.start = shape.start; % starting point
param.target=shape.target; % target point

param.xTar = shape.target(1);
param.yTar = shape.target(2);
param.zTar = shape.target(3);

% Make the UAV go to (xTar, yTar,zTar)
r = zeros(12,1);
r(1,1) = param.xTar; r(2,1) = param.yTar; r(3,1) = param.zTar;
param.targetMod = r;
param.shape = shape;
param.constraints = shape.constraints;

param.hc = shape.constraints.rect.h; % height constraint
param.bot = shape.constraints.rect.bot; % box constraint
param.eps_t = shape.eps_t; % The target point tolerance
param.ThetaMax = shape.ThetaMax;
param.Tt = shape.Tt; 
param.Te = shape.Te; 
param.nx = 12; 
param.ny = 12;
param.nu = 4; 

param.N = ceil((param.Tt + param.Te)/param.Ts); % prediction horizon

param.pathGuess = getPathGuess(shape,100, 1.1, param.N,param); 

end % End of mySetup

function u = myMPController(param, y)
  persistent mpciter lastMv lastU lastX MaxIter


  function [cineq, ceq] = multipleShootingConstraints(OPT_variables, N, T, n_states, n_controls, ...
                                                            P, Ixx, Iyy, Izz, b, g, k, l, m, ...
                                                            omegaMax2,constraints,hc,current_state)
        
        X = reshape(OPT_variables(1:n_states*(N+1)), n_states, N+1);
        U = reshape(OPT_variables(n_states*(N+1)+1:end), n_controls, N);
        
        % Initial state constraints
        ceq = [X(:,1) - current_state]; 
    
        h = T; % step size for each interval
        
        % RK4 for integration
        for i = 1:N
            x0 = X(:,i); u0 = U(:,i);
            k1 = f(x0,                             u0, Ixx, Iyy, Izz, b, g, k, l, m, omegaMax2);
            k2 = f(x0 + 0.5 * h * k1,              u0, Ixx, Iyy, Izz, b, g, k, l, m, omegaMax2);
            k3 = f(x0 + 0.5 * h * k2,              u0, Ixx, Iyy, Izz, b, g, k, l, m, omegaMax2);
            k4 = f(x0 +       h * k3,              u0, Ixx, Iyy, Izz, b, g, k, l, m, omegaMax2);
            x_next = x0 + h/6 * (k1 + 2*k2 + 2*k3 + k4);

            ceq = [ceq; X(:,i+1) - x_next]; % system dynamic constraint for each step
        end

        % % Additional constraints for final state positions
        % x_final = P(end-n_states+1:end)';
        % ceq = [ceq; X(:,end) - x_final;]; % 最终状态约束 和 最后几步用于稳定的约束 (是不需要的)
    
        % Inequality constraints 
        p = N; 
        posX = X(1,2:p+1); 
        posY = X(2,2:p+1);
        posZ = X(3,2:p+1);

        stepsLeft = size(X,2);

        % constraints : box constraints
        [DRect,clRect,chRect] = getRectConstraints(constraints.rect.bot);
        
        cineq = [
            posX*DRect(1,1)+posY*DRect(1,2)-chRect(1);
            -posX*DRect(1,1)-posY*DRect(1,2)+clRect(1);
            posX*DRect(2,1)+posY*DRect(2,2)-chRect(2);
            -posX*DRect(2,1)-posY*DRect(2,2)+clRect(2);
            -posZ + (hc+0.01); % height constraint 这里已经有总高度限制了
            posZ + 0.01; % 和 MPC toolbox 不一样，这里的地面限制加在了这里
            ]; % 地面和天花板约束也要放大一点
        
        l = 1.05; % enlarge factor
        % elliptical cylinder constraint
        for i = 1:length(constraints.ellipses)
            el = constraints.ellipses{i};
            elCon = min(-((((posX)-el.xc)/el.a).^2 + (((posY)-el.yc)/el.b).^2) + 1*l, ...
                posZ - el.h*l);
            cineq = [cineq; elCon;];
        end

        cineq = reshape(cineq,[numel(cineq),1]); % 拉成一列

        %-----------------------------------------------------------------%
        % % put constraint on the time interval [Tt, Tt + Te]
        % if mpciter > 1 % 这里完全是魔法
        for i= 1:min(ceil(param.Te/param.Ts) + 2 , stepsLeft-1 ) % ceil(param.Te/param.Ts)
            xFin = X(1:3,p+2-i);
            cineq=[ cineq;
                xFin - param.targetMod(1:3) - param.eps_t * ones(3,1);
                -xFin + param.targetMod(1:3) - param.eps_t * ones(3,1);
                ];
        end
        % end
  end
  
    
    if isempty(mpciter)
        mpciter = 0;
        MaxIter = ceil((param.Tt + param.Te)/param.Ts) + 50; % 第一次的时候多算几次
        lastMv = zeros(param.nu,1);
        N = param.N; 
        U = repmat(lastMv, 1, N);    
        
        firstStateGuess = repmat(y, 1, N + 1);
       
        % A* is NECESSARY
        for i = 2:param.N
            firstStateGuess(1,i)=param.pathGuess(i,1);
            firstStateGuess(2,i)=param.pathGuess(i,2);
            firstStateGuess(3,i)=param.pathGuess(i,3);
        end
        X = firstStateGuess;


        initialParams = [param.Ixx, param.Iyy, param.Izz, param.b, param.g, param.k, param.l, param.m,param.omegaMax2];
        
        
    end

    % reuse last X and U as initial guess
    if (mpciter>=1)
        MaxIter = MaxIter - 1; % 递减
        N = param.N - mpciter;
        X  = lastX(:,2:end); U = lastU(:,2:end); 
        
    end 

    Q = diag([1,1,1,1,1,1,0,0,0,0,0,0]); % weights on states
    R = diag([0.1 0.1 0.1 0.1]);         % weights on Controls

    P = [y', param.targetMod']; % Starting and target points
     

    OPT_variables = [reshape(X, numel(X), 1); reshape(U, numel(U), 1)];

    objective = @(OPT_variables) objectiveFunction(OPT_variables, N, param.Ts, ...
        P, Q, R, param.nx, param.nu,param.u_eq);

    nonlcon = @(OPT_variables) multipleShootingConstraints(OPT_variables, N, param.Ts, ...
        param.nx, param.nu, P,...
        param.Ixx,param.Iyy,param.Izz,param.b,param.g, ...
        param.k,param.l,param.m, param.omegaMax2, ...
        param.constraints,param.hc,y);



    options = optimoptions('fmincon', ...
        'Algorithm', 'interior-point', ... 
        'FunctionTolerance', 0.001, ...
        'StepTolerance', 0.001, ...
        'MaxIterations', MaxIter, ...
        'ConstraintTolerance', 0.001, ...
        'Display', 'off');



    % control input constraint on u [0,1]
    u_min = 0; u_max = 1;
    U_lb = u_min * ones(param.nu, N); U_ub = u_max * ones(param.nu, N);

    u_lb = [repmat(-Inf, numel(X), 1); reshape(U_lb, numel(U_lb), 1)];
    u_ub = [repmat(Inf, numel(X), 1); reshape(U_ub, numel(U_ub), 1)];


    [sol,~, ~, ~] = fmincon(objective, OPT_variables, [], [], [], [],u_lb, u_ub, ...
        nonlcon, options);

   
    u = sol(12*(N+1)+1 : 12*(N+1) + 4);  % size(u)
   
    numX = param.nx * (N + 1); % Assuming X includes N+1 time steps

    U_vectorized = sol(numX+1:end); % Extract U portion from the solution vector
    numX = param.nx * (N + 1);  X_vectorized = sol(1:numX);

    % Reshape X back into its matrix form [param.nx x (N+1)]
    lastX = reshape(X_vectorized, param.nx, N + 1); lastU = reshape(U_vectorized, param.nu, N);
    lastMv = u;  mpciter =  mpciter + 1; 
end % End of myMPController





function [obj] = objectiveFunction(OPT_variables, N, T, P, Q, R, n_states, n_controls, u_eq)
    X = reshape(OPT_variables(1:n_states*(N+1)), n_states, N+1);
    U = reshape(OPT_variables(n_states*(N+1)+1:end), n_controls, N);
    
    P_const = repmat(P(13:24)', 1, N);
    u_eq_rep = repmat(u_eq, 1, N); 
    
    dX = X(:,1:N) - P_const; % 
    dU = U - u_eq_rep;

    dX_Q = dX' * Q; 
    obj_X = sum(diag(dX_Q * dX)); 
 
    dU_R = dU' * R; 
    obj_U = sum(diag(dU_R * dU)); 
    
    % Total objective
    obj = obj_X + obj_U;
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
           ec = ((-(((x-el.xc)/el.a).^2 + ((y-el.yc)/el.b).^2) + 1 )>0 && (z - el.h >0));
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



he closed list
        openList.remove(current);
        closedList(current) = true; % Again, value is arbitraryfunction pathGuess = getPathGuess(shape,res, e, predictionHorizon,param)
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
        
        % Remove current node from open list and add it to t
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




function rhs = f(in1,in2,Ixx,Iyy,Izz,b,g,k,l,m,omegaMax2)
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
rhs = [u;v;w;p+t16.*tan(theta);q.*t2-r.*t5;t16./t4;-k.*omegaMax2.*t14.*t15.*(t5.*t6+t2.*t3.*t7);k.*omegaMax2.*t14.*t15.*(t3.*t5-t2.*t6.*t7);g-k.*omegaMax2.*t2.*t4.*t14.*t15;(q.*r.*(Iyy+t9)+k.*l.*omegaMax2.*(input_1+input_4+t11+t12))./Ixx;-(p.*r.*(Ixx+t9)-k.*l.*omegaMax2.*(input_1+input_2+t12+t13))./Iyy;-(b.*omegaMax2.*(input_1+input_3+t11+t13)-p.*q.*(Ixx-Iyy))./Izz];
end




