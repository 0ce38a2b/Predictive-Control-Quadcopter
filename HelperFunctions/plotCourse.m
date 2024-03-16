function [ fig ] = plotCourse( course, titleString, dis )
%PLOTCOURSE Summary of this function goes here
%   Detailed explanation goes here

%% Extract the shape in the course
shape = course.shape;


%% Create the figure and add stuff to it
if( dis )
    fig = figure( 'Name', titleString );
else
    fig = figure( 'Name', titleString, 'Visible', 'off' );
end
hold on;

title( titleString );


%% Add the constraint boundaries
h=shape.constraints.rect.h;
v = [shape.constraints.rect.bot, zeros(4,1);
    shape.constraints.rect.bot, h*ones(4,1)];
f = [1 2 3 4; 
     5 6 7 8;
     1 2 6 5;
     2 3 7 6;
     3 4 8 7;
     4 1 5 8];
p=patch('Faces',f,'Vertices',v,'FaceColor','red');
p.FaceAlpha = 0.3;
view(3);

% Save the limits for the rectangle so we can restore them after adding the ellipses
xl = xlim();
yl = ylim();
z1 = zlim();

% Add the ellipses if there are any
if( isfield( shape.constraints, 'ellipses' ) && ~isempty( shape.constraints.ellipses ) )
    for i=1:1:length( shape.constraints.ellipses ) 
        ellipse = shape.constraints.ellipses{i};

        t = -pi:0.01:pi;
        x = ellipse.xc + ellipse.a * cos( t );
        y = ellipse.yc + ellipse.b * sin( t );
        z = ellipse.h * ones(size(t));
        l=length(t);
        ve = [x' y' 0*z';
             x' y' z'];
        fe=zeros(l,4);
        fe2=zeros(2,l);
        fe2(1,1) = 1;
        fe2(2,1) = l+1;
        fe(1,1:4) = [l 1 l+1 l+l];
        for k=2:l
            fe(k,1:4) = [k-1 k l+k l+k-1];
            fe2(1,k) = k;
            fe2(2,k) = l+k;
        end
  
        pe=patch('Faces',fe,'Vertices',ve,'FaceColor','red','EdgeColor','none');
        pe.FaceAlpha = 0.3;
        pe2=patch('Faces',fe2,'Vertices',ve,'FaceColor','red','EdgeColor','none');
        pe2.FaceAlpha = 0.3;
    end
end
    
% Restore the previous plot limits
xlim( xl );
ylim( yl );
zlim( z1 );


%% Add the starting point
plot3( shape.start(:,1), shape.start(:,2), shape.start(:,3), 'bo' );


%% Add the target point (with epsilon size)
vt=[shape.target(:,1)-shape.eps_t, shape.target(:,2)-shape.eps_t, shape.target(:,3)-shape.eps_t;
    shape.target(:,1)-shape.eps_t, shape.target(:,2)+shape.eps_t, shape.target(:,3)-shape.eps_t;
    shape.target(:,1)+shape.eps_t, shape.target(:,2)+shape.eps_t, shape.target(:,3)-shape.eps_t;
    shape.target(:,1)+shape.eps_t, shape.target(:,2)-shape.eps_t, shape.target(:,3)-shape.eps_t;
    shape.target(:,1)-shape.eps_t, shape.target(:,2)-shape.eps_t, shape.target(:,3)+shape.eps_t;
    shape.target(:,1)-shape.eps_t, shape.target(:,2)+shape.eps_t, shape.target(:,3)+shape.eps_t;
    shape.target(:,1)+shape.eps_t, shape.target(:,2)+shape.eps_t, shape.target(:,3)+shape.eps_t;
    shape.target(:,1)+shape.eps_t, shape.target(:,2)-shape.eps_t, shape.target(:,3)+shape.eps_t];
pt=patch('Faces',f,'Vertices',vt,'FaceColor','blue');
pt.FaceAlpha = 0.3;
plot3( shape.target(:,1), shape.target(:,2),  shape.target(:,3), 'b*' );


%% Set the right limits to the visualization

zl = zlim();
yl = ylim();
xl = xlim();

zdiff = diff( z1 );
ydiff = diff( yl );
xdiff = diff( xl );

if( ydiff == xdiff )
    % The axis are already equal
    newyl = yl;
    newxl = xl;
elseif( ydiff > xdiff )
    % The y axis is larger than x - expand x
    expand = ( ydiff - xdiff ) / 2;
    
    newxl = xl + [-expand, expand];
    newyl = yl;
else
    % The x axis is larger than y - expand y
    expand = ( xdiff - ydiff ) / 2;
    
    newxl = xl;
    newyl = yl + [-expand, expand];
end

xydiff = diff( newxl );
if( zdiff == xydiff )
    % The axis are already equal
    newzl = zl;
elseif( zdiff > xydiff )
    % The y axis is larger than x - expand x
    expand = ( zdiff - xydiff ) / 2;
    
    newxl = newxl + [-expand, expand];
    newyl = newyl + [-expand, expand];
    newzl = zl;
else
    % The x axis is larger than y - expand y
    expand = ( xydiff - zdiff ) / 2;
    
    newzl = zl + [-expand, expand];
end

% Add a slight border around the graph
zlim( newzl + [-0.01, 0.01] );
ylim( newyl + [-0.01, 0.01] );
xlim( newxl + [-0.01, 0.01] );

axis square;
set(gca,'ZDir','reverse')

end
