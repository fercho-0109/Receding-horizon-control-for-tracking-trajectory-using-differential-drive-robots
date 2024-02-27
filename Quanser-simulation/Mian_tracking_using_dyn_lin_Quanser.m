% Replay the experiment of paper TCST22
% RHC for contrained differential drive Robot (USING Dynamic feedback linearization)
%
%----- USING QUANCER QBot2 ENVIROMENT--------------------------
%
% #################################################################################
% Edited by Alexis Marino (mrnlsf96p01z605k@studenti.unical.it) September, 15th 2023
% #################################################################################
%
% Took form the article "A Receding Horizon Trajectory Tracking Strategy
% for Input-Constrained Differential-Drive Robots via Feedback-Linearization"
% Autors Cristian Tiriolo, Giuseppe Franz`e, Walter Lucia

clear
close all;
clc;
load('Ref_poly.mat')
%% parameters
R=0.035;        % radius od wheels [m]
D= 0.235;      % distance between the two wheels [m]
omega=10;   % max angular velocity [rad/s] 
Ts= 0.15;       % sampling time [s]
db=0.5;       % vision radius [m]
vm=0.01;        % min linear velocity [m/s]
parameters=[R,D,omega,Ts,db,vm];
% symetric saturations constrains
Hd=[-1/omega 0 1/omega 0;
    0 -1/omega 0 1/omega]';

T=[R/2 R/2;     % diff drive to unicycle
    R/D -R/D];

Tinv=inv(T);

Qbd=[db*db 0  % vision radius 
    0 db*db];

Hu=Hd*Tinv;

HTs=[Ts 0;
     0 1];

% matrices for the model uncertanty
A=[1 0 Ts 0;
    0 1 0 Ts;
    0 0 1 0;
    0 0 0 1];

B=[(Ts*Ts)/2 0;
    0 (Ts*Ts)/2;
    Ts 0;
    0 Ts];

Bp=[0 0;
    0 0;
    -1 0;
    0 -1;];

Cq=[0 0 1 0;
    0 0 0 1];

Dqu=zeros(2);

%% Robot position "odometric calculation"
% X_robot_k1 =@(X_k,v_k,theta_k)(X_k+Ts*v_k*cos(theta_k));
% Y_robot_k1 =@(Y_k,v_k,theta_k)(Y_k+Ts*v_k*sin(theta_k));
% theta_k1 = @(theta_k,w_k)(theta_k+Ts*w_k);

%% reference trajectory
% enviroment-shape
Tsim=39;
xr=@(k)(polyval(p1,k));
yr=@(k)(polyval(p2,k));
xrr=polyder(p1);
yrr=polyder(p2);
xrstar=@(k)(polyval(xrr,k));
yrstar=@(k)(polyval(yrr,k));
thetar=@(k)(polyval(p3,k));
t1=0:Ts:Tsim;
xp=xr(t1);
yp=yr(t1);

% eight-shape
% Tsim=44; 
% xr=@(k)(0.6*sin(k/3.5));
% yr=@(k)(0.6*sin(k/7));
% xrstar=@(k)((0.6/3.5)*cos(k/3.5));
% yrstar=@(k)((0.6/7)*cos(k/7));
% t=0:0.5:44;
% xp=xr(t);
% yp=yr(t);
%% control 
%_______off line__________%
% positions
X0=[-0.25 -0.75 0]'; % initial conditions 
%X0=[-0.50 -2.75 0]';
Qe=[100 0 0 0;
    0 100 0 0;
    0 0 0 0;
    0 0 0 0];

rho=0.001;
Qu=rho*eye(2);

% velocities to inicialate the dinamic compensator
v=[0.01,0.01]; % first element v(k-1) second element v(k)

% current states of the robot 
X_k=X0(1);     
Y_k=X0(2);
theta_k=X0(3);

% compute the radius of the worse case scenario
ru=(2*omega*R*vm)/(sqrt(4*vm*vm*Ts*Ts+D*D));

% Error computing of the system with respect to the reference
Err0= [X_k;Y_k;v(2)*cos(theta_k);v(2)*sin(theta_k)]-[xr(0);yr(0);0;0];

% control computing of the worse case scenario
[Q0,Y0]=worse_case_NB(Err0,A,B,ru,Bp,Qe,Qu,Cq,Dqu,Qbd)

% %% on-line part
% n=length(t1);
% % variables to plots the results 
% x=zeros(1,n);
% y=zeros(1,n);
% theta=zeros(1,n);
% wr=zeros(1,n);
% wl=zeros(1,n);
% trackerr=zeros(1,n);
% 
% % on-line part
% figure(1)  % plot the robot movement in the enviroment  
% load mapa1.mat
% show(map1)
% hold on
% 
% j=1;
% qseq=X0;
% sum=0;
% err=0;
% for i=0:Ts:Tsim
% 
%     Xr=xr(i);
%     xrr(j)=Xr;  % save the reference
%     Yr=yr(i);
%     yrr(j)=Yr;
%     Xrd=xrstar(i);
%     Yrd=xrstar(i);
% 
%     Err = [X_k;Y_k;v(2)*cos(theta_k);v(2)*sin(theta_k)]-[Xr;Yr;0;0]; %[[xr;yr]-[X_k;Y_k];0;0] 
%     sum=sum+Err;
% 
%     Tfl = [cos(theta_k) -v(2)*sin(theta_k);
%               sin(theta_k) v(2)*cos(theta_k)];
% 
% 
%     H_theta_v = Hu*HTs*inv(Tfl);
%     nk=[v(1)/Ts;0];
% 
%     %[Q_str,Y_str] = RHC_OA(Err,A,B,H_theta_v,Q0,Qe,Qu);
%     %[Q_str,Y_str] = RHC_OA_new(Err,A,B,H_theta_v,Q0,Qe,Qu);
%     [Q_str,Y_str] = RHC_Dy_lin_mincx(Err,A,B,H_theta_v,Qe,Qu,Q0);
% 
% 
%     uk=Y_str*inv(Q_str)*Err-Tfl*nk;
% 
%     % it works but I am using the matrix----------------------------------
%     % w = Tinv*HTs*inv(Tfl)*uk+Tinv*HTs*nk; % angular velocities for right and left
%     % wr(j)=w(1);
%     % wl(j)=w(2);
%     % --------------------------------------------------------------------
%     %u = HTs*inv(Tfl)*uk+HTs*nk; % linear velocity - angular velocity
% 
%     u=inv(Tfl)*uk;         % aceleration and angular velocity
%     acel=u(1);        % take the aceleration
%     vel=acel*Ts+v(1); % back derivate aproximation to take the velocity
%     v=[v(2),vel];     % v(k-1) equal to previus v(k) equal to current
% 
%     vw=[v(2);u(2)];   % angular right and left velocities      
% 
%     wrwl=Tinv*vw;     % Unicycle/Diffdrive input transformation
%     wr(j)=wrwl(1); 
%     wl(j)=wrwl(2);
% 
%     % Apply to the real plant in discrete time ---------------------------
%     % % first idea
%     % x_real = (R/2)*(w(1)+w(2))*cos(theta_k);
%     % y_real = (R/2)*(w(1)+w(2))*sin(theta_k);
%     % theta_real = (R/D)*(w(1)-w(2));
%     %---------------------------------------------------------------------
% 
%     % Cristian idea in continuos time-------------------------------------
%     %Applico la legge di controllo al sitema non lineare
%     v1=vw(1); w1=vw(2);
%     t=0:0.00001:Ts;
%     [t,q]= ode45(@(t,q,v,w)DiffDrive(t,q,v1,w1),t,qseq(:,end));
%     %aggiorno la sequenza di stato e ingresso
%     qseq=[qseq q(end,:)'];
%     x(j)=X_k;
%     X_k=qseq(1,end);
%     y(j)=Y_k;
%     Y_k=qseq(2,end);
%     theta(j)=theta_k;
%     theta_k=qseq(3,end);
%     % the positioning proccess is't needed because I am using the positions of the evolucion in ode but it works anyway
%     % --------------------------------------------------------------------
% 
%     % save tracking error
%     trackerr(j)=norm([X_k,Y_k]-[Xr,Yr]);
%     err=err+trackerr(j);
%     pause(0.001)
%     p_traj=plot(X_k,Y_k,'bo');
%     p_traj_ref=plot(Xr,Yr,'r.');
%     j=j+1
% end
% hold off
% avr=sum/i
% Track_err=err/i
% %%
% figure(2)
% grid on
% plot(x,y,'b',xp,yp,'--')
% grid on 
% title('Robot vs Reference')
% legend('Robot evo','Robot ref')
% 
% figure(3)
% cero=zeros(1,n);
% plot(t1,x-xrr,'r',t1,y-yrr,'b',t1,cero,'k')
% grid on
% title('error')
% 
% figure(4)
% p=plot(t1,wr,'r',t1,wl,'b');
% p(1).LineWidth=1.25;
% p(2).LineWidth=1.25;
% legend('wr','wl')
% grid on
% title('wr and wl inputs')
% 
% figure(5)
% thetarr=thetar(t1);
% plot(t1,theta,t1,thetarr,'--')
% grid on
% title('Theta')
% 
% figure(6)
% p=plot(t1,trackerr,'b',t1,cero,'k--');
% p(1).LineWidth=1.25;
% grid on
% title('Tracking error')