function [ course, fig ] = defaultCourse( dis, coursenum )
%DEFAULTCOURSE Generate the default course for the core coursework

if coursenum == 1
    % The course for the first part
    constraints.rect.bot = [0.00, 0.05;
                            0.45, 0.50;
                            0.50, 0.45;
                            0.05, 0.00];
    constraints.rect.h=-0.6;
    constraints.ellipses = {};
    
    start  = [0.05, 0.05, -0.05];
    target = [0.4, 0.4, -0.4];
    
    
    Tt   = 5;
    Te   = 1;
    ThetaMax = 0.1;
    course.perturbSize = 0.1;

elseif coursenum == 2
    % The course for the second part
    constraints.rect.bot = [0.00, 0.05;
                            0.25, 0.30;
                            0.50, 0.05;
                            0.25, -0.20];
       
    constraints.rect.h=-0.4;
    
    ellipse.a  = 0.3;
    ellipse.b  = 0.3;
    ellipse.h  = -0.32;
    ellipse.xc = 0.25;
    ellipse.yc = -0.23;
    constraints.ellipses{1} = ellipse;
    
    Tt   = 3.3;
    Te   = 2.7;
    ThetaMax = 0.2;
    course.perturbSize = 0.1;
    
    target = [0.05, 0.05, -0.05];
    start = [0.45, 0.05, -0.30]; % 用于 凸测试
    % start = [0.25, 0.25, -0.30]; % 用于测试椭圆高度约束
   % start = [0.25, -0.1, -0.35]; % 用于测试椭圆高度约束

    % test 1
    % start = [0.25, 0.25, -0.30];
    % target = [0.05, 0.05, -0.05];
   
    % 起点终点都在椭圆上，中间没有椭圆
    % start = [0.25, 0.25, -0.30];
    % target = [0.25, -0.1, -0.35];

    % 终点在起点下方
    % start = [0.25, 0.25, -0.30];
    % target = [0.05, 0.05, -0.15];

    Tt   = 2.3;
    Te   = 3.7;
    ThetaMax = 0.2;
    course.perturbSize = 0.1;



elseif coursenum == 4
   constraints.rect.bot = [0.00, 0.05;
                            0.25, 0.30;
                            0.50, 0.05;
                            0.25, -0.20];
    constraints.rect.h=-0.4;

    ellipse0.a  = 0.08;
    ellipse0.b  = 0.4;
    ellipse0.h  = -0.25;
    ellipse0.xc = 0.18;
    ellipse0.yc = -0.23;

    ellipse1.a  = 0.06;
    ellipse1.b  = 0.05;
    ellipse1.h  = -0.4;
    ellipse1.xc = 0.38;
    ellipse1.yc = 0.03;
    
    constraints.ellipses={ellipse0,ellipse1};

    target  = [0.05, 0.05, -0.05];
    start = [0.45, 0.05, -0.30];
    
    
    Tt   = 5; % 5
    Te   = 1; % 1
    ThetaMax = 0.2;
    course.perturbSize = 0.1;

elseif coursenum == 10
    % The course for the second part
    constraints.rect.bot = [0.00, 0.20;
                            0.40, 0.20;
                            0.40, -0.20;
                            0.0, -0.20];
    constraints.rect.h = -0.4;
    
    ellipse0.a  = 0.095;
    ellipse0.b  = 0.06;
    ellipse0.h  = -0.4;
    ellipse0.xc = 0.21;
    ellipse0.yc = 0.0;

    ellipse1.a  = 0.06;
    ellipse1.b  = 0.05;
    ellipse1.h  = -0.4;
    ellipse1.xc = 0.38;
    ellipse1.yc = 0.03;

    ellipse2.a  = 0.05;
    ellipse2.b  = 0.08;
    ellipse2.h  = -0.4;
    ellipse2.xc = 0.35;
    ellipse2.yc = -0.1;

    ellipse3.a  = 0.12;
    ellipse3.b  = 0.05;
    ellipse3.h  = -0.4;
    ellipse3.xc = 0.17;
    ellipse3.yc = -0.15;

    ellipse4.a  = 0.05;
    ellipse4.b  = 0.17;
    ellipse4.h  = -0.4;
    ellipse4.xc = 0.04;
    ellipse4.yc = 0.0;

    ellipse5.a  = 0.08;
    ellipse5.b  = 0.06;
    ellipse5.h  = -0.12;
    ellipse5.xc = 0.22;
    ellipse5.yc = 0.16;

    ellipse6.a  = 0.03;
    ellipse6.b  = 0.05;
    ellipse6.h  = -0.4;
    ellipse6.xc = 0.11;
    ellipse6.yc = -0.10;

    ellipse7.a  = 0.06;
    ellipse7.b  = 0.04;
    ellipse7.h  = -0.4;
    ellipse7.xc = 0.23;
    ellipse7.yc = 0.1;

    ellipse8.a  = 0.02;
    ellipse8.b  = 0.05;
    ellipse8.h  = -0.4;
    ellipse8.xc = 0.11;
    ellipse8.yc = 0.10;

    ellipse9.a  = 0.08;
    ellipse9.b  = 0.03;
    ellipse9.h  = -0.2;
    ellipse9.xc = 0.28;
    ellipse9.yc = 0.17;
    constraints.ellipses={ellipse5,ellipse0,ellipse3,ellipse1,ellipse9,...
        ellipse5,ellipse7};

    start  = [0.31, -0.18, -0.05];
    target = [0.22, 0.17, -0.30];
    
    
    Tt   = 5.8;
    Te   = 0.2;
    ThetaMax = 0.2;
    course.perturbSize = 0.1;
end

shape.constraints = constraints;

shape.eps_t       = 0.02;
shape.start       = start;
shape.target      = target;
shape.Tt          = Tt;
shape.Te          = Te;
shape.ThetaMax    = ThetaMax;

% The default shape is always considered problem 1
course.prob      = 1;
course.shape     = shape;
course.completed = 0;

% Create the perturbations
rng(15012);
randPerturb = @() ( ( rand()-0.5 ) * course.perturbSize ) + 1;

course.perturb.IxxVal        = randPerturb();
course.perturb.IyyVal        = randPerturb();
course.perturb.IzzVal        = randPerturb();
course.perturb.kVal          = randPerturb();
course.perturb.lVal          = randPerturb();
course.perturb.mVal          = randPerturb();
course.perturb.bVal          = randPerturb();
course.perturb.omegaMax2Val  = randPerturb();

fig = plotCourse( course, 'Default course ', dis );
end

