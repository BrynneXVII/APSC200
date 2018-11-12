clear all; 
% Linear Approximated Control of a 1-wheeled vehicle for a circular
% trajectory

T=20; % Simulation Time
delta=0.01; % Step Size
N=T/delta; % Number of Data points
t=linspace(0,T-delta,N); % Time Vector

% System Parameters

r = 10; % Circle Radius [m]
alpha = 0.5; % angular rate [rad/s]

% Initial Conditions

X0=2; % [m]
Y0=5; % [m]
theta0=0; % [rad]

q=[X0;Y0;theta0]; % Initial coordinate'

% Reference Trajectories

Xd = r*sin(alpha*t); % [m]
Yd = r-r*cos(alpha*t);% [m]
thetad = alpha*t;% [rad]

% Xddot = -r*alpha*sin(alpha*t); % [m/s]
% Yddot = r*alpha*cos(alpha*t); % [m/s]

vd = r*alpha; % [m/s]
omegad = alpha; % [rad/s]

% Controller Gains . Have to Find in Loop for each time step

% K1=100;
% K = [0,0,0;
%     0,0,-K1];
Nz=0; % Inlcude Noise? 1 if yes, 0 if no.
jj=0;
for i=1:N
    
    A = [0,0,-vd*sin(thetad(i));
        0,0,vd*cos(thetad(i));
        0,0,0];
    
    B = [cos(thetad(i)),0;
        sin(thetad(i)),0;
        0,1];
    
    p = [-1 -2 -5]; % closed loop poles
    
    K = place(A,B,p); % Place poles so that K is Hurwitz for A, B at each time step
    
    qd=[Xd(i);Yd(i);thetad(i)]; % Desired Trajectory Vector
    ud=[vd;omegad]; % Desired control input (based on planned trajectory)
    
    u = K*(qd-q)+[vd;omegad]; % Control input for each time
    
    U(i,:)=u; % Save the control inputs
    
    qdot=[u(1)*cos(q(3));u(1)*sin(q(3));u(2)]; % Change in vehicle state based on current state and control inputs
    
    

    
    q=q+qdot*delta;
    
    Q(i,:)=q;
    Qd(i,:)=qd;
    Qdot(i,:)=qdot;
        
end

%% Simulate the Results
clf;
% Plotting Preamble
win(1)=subplot(2,2,[1 3]);        % add first plot in 2 x 2 grid
win(2)=subplot(2,2,2);        % add second plot in 2 x 2 grid
win(3)=subplot(2,2,4);    % add third plot to span positions 3 and 4
AX=[1.1*min(min(Q(:,1),min(Xd'))) 1.1*max(max(Q(:,1)),max(Xd')) 1.1*min(min(Q(:,2)),min(Yd')) 1.1*max(max(Q(:,2)),max(Yd))];
i=1;

for i=1:20:N % Plot every 20 data points for speed's sake.
    
    figure(1)
    
    set(win(1), 'ColorOrder', [238/255 189/255 49/255; 17/255 51/255 93/225],'NextPlot', 'replacechildren');
    plot(win(1),Q(1:i,1),Q(1:i,2),'--',Xd(1:i),Yd(1:i),'--');
    hold(win(1),'on')
    axis(win(1),AX);
    xlabel(win(1),'x [m]')
    ylabel(win(1),'y [m]')
    [X,Y] = plot_unicycle(Q(i,:),axis(win(1)));
    fill(win(1),X,Y,[238/255 189/255 49/255]);
    hold(win(1),'off');
    
    set(win(2), 'ColorOrder', [157/255 25/255 57/225; 17/255 51/255 93/225; 238/255 189/255 49/255],'NextPlot', 'replacechildren');
    plot(win(2),t(1:i),Qd(1:i,:)-Q(1:i,:),'-')
    axis(win(2),[0 T -25 25])
    xlabel(win(2),'Time [s]')
    ylabel(win(2),'Error [m]')
    legend(win(2),'x error','y Error','\theta Error');
    
    set(win(3), 'ColorOrder', [157/255 25/255 57/225; 17/255 51/255 93/225; 238/255 189/255 49/255],'NextPlot', 'replacechildren');
    plot(win(3),t(1:i),Qdot(1:i,:),'-')
    axis(win(3),[0 T -25 25])
    xlabel(win(3),'Time [s]')
    ylabel(win(3),'Q_{dot}')
    legend(win(3),'x_{dot}','y_{dot}','\theta_{dot}');
    
    pause(10^-9)
end

%% Static Plot

close all;

figure(2)

subplot(2,2,[1 3])        % add first plot in 2 x 2 grid
set(gca, 'ColorOrder', [238/255 189/255 49/255; 17/255 51/255 93/225],'NextPlot', 'replacechildren');
axis([1.1*min(Q(:,1)) 1.1*max(Q(:,1)) 1.1*min(Q(:,2)) 1.1*max(Q(:,2))])
xlabel('x')
ylabel('y')
legend('Vehicle Trajectory','Reference Trajectory')

subplot(2,2,2)        % add second plot in 2 x 2 grid
set(gca, 'ColorOrder', [157/255 25/255 57/225; 17/255 51/255 93/225; 238/255 189/255 49/255],'NextPlot', 'replacechildren');
plot(t,Q-Qd,'.')
axis([0 T -25 25])
xlabel('Time [s]')
ylabel('Error')
legend('x error','y Error','\theta Error');
hold on

subplot(2,2,4)    % add third plot to span positions 3 and 4
set(gca, 'ColorOrder', [157/255 25/255 57/225; 17/255 51/255 93/225; 238/255 189/255 49/255],'NextPlot', 'replacechildren');
plot(t,Qdot,'.')
axis([0 T -25 25])
xlabel('Time [s]')
ylabel('Q_{dot}')
legend('x_{dot}','y_{dot}','\theta_{dot}');
hold on



