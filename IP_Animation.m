function animation =  IP_Animation(x,th)

persistent j
persistent pjx
persistent pjy
    if isempty(j)
        j = 0;
    end
    j = j+1
W  = 0.2; % width of cart
H  = 0.08; % hight of Cart
L  = 0.35; % length of pendulum  
wr = 0.04; % right wheel 

% Position coordination
y = H/2+wr/2;
w1x = x -0.9*W/2;
w1y = 0;
w2x = x+0.9*W/2-wr;
w2y = 0;

% position of pendulum 
px = x + L*sin(th);
py = y - L*cos(th);

pjx(j) = px;
pjy(j) = py;

base = plot([-1 1],[0 0],'k','LineWidth',2); % base line
hold on;
cart = rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',0.1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1]);
left_wheel  = rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1]);
right_wheel = rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1]);
    
pendulum = plot([x px],[y py],'b','LineWidth',2.5); % Pendulum rod 
p_cir = viscircles([px py],0.02,'Color',[1 0.1 0.1],'LineWidth',2.5); % Pendulum Trajectory(Circle)
p_cir1 = viscircles([x y],0.02,'Color','w','LineWidth',0.2); % center of Cart
  
%line_traj = plot(pjx(1:j),pjy(1:j), 'm--','LineWidth',1);  % Pendulum Trajectory (Line)
    xlabel('X (m)');
    ylabel('Y (m)');
    title('(JITENDRA) Inverted Pendulum: SwingUp control')
    axis(gca,'equal');
    xlim([-1 1]);
    ylim([-0.4 0.5]);
    %set(gcf,'Position',[10 900 800 400])
    grid on;
    %drawnow
    %pause(0.01);
   
   
   