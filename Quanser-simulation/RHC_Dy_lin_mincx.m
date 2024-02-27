function [Q_str,Y_str]=RHC_Dy_lin_mincx(Err,A,B,H_theta,Qx,Qu,Q0)
n=size(A,1);
m=size(B,2);
%Qdb=(d^2*eye(2));
setlmis([])

Q = lmivar(1,[n 1]); % create the unknow variables type 1 symetric
Qb = lmivar(1,[n 1]); % create the unknow variables type 1 symetric
Y = lmivar(2,[m n]); % type 2 m x n matrix
gamma=lmivar(1,[1 1]); % variable scalar

% LMI constrains
lmiterm([-1 1 1 Q],1,1);    %constraint 1   F1=([Q]>=0);

lmiterm([-9 1 1 Qb],1,1);    %constraint 1   F1=([Qb]>=0);

lmiterm([-2 1 1 0],1);   %constraint 2 block (1,1)      F2=([1 Err0';
lmiterm([-2 2 1 0],Err);  %constraint 2 block (2,1)        Err0 Q ]>=0);
lmiterm([-2 2 2 Q],1,1);   %constraint 2 block (2,2)

lmiterm([-3 1 1 Q],1,1);        %constraint 3 block (1,1) 
lmiterm([-3 1 1 -Q],1,1);        %constraint 3 block (1,1) 
lmiterm([-3 1 1 Qb],-1,1);        %constraint 3 block (1,1)
lmiterm([-3 2 1 Q],A,1);        %constraint 3 block (2,1)   F3=([Q+Q'-Qb     Q*A'+Y'*B' Q*sqrt(Qx)    Y'*sqrt(Qu)
lmiterm([-3 2 1 Y],B,1);        %constraint 3 block (2,1)        A*Q+B*Y     Q          zeros(2)      zeros(2)
lmiterm([-3 3 1 Q],sqrt(Qx),1); %constraint 3 block (3,1)        sqrt(Qx)*Q  zeros(2)   gamma*eye(2)  zeros(2)
lmiterm([-3 4 1 Y],sqrt(Qu),1); %constraint 3 block (4,1)        sqrt(Qu)*Y  zeros(2)   zeros(2)      gamma*eye(2)]>=0);
lmiterm([-3 2 2 Q],1,1);        %constraint 3 block (2,2)
lmiterm([-3 3 2 0],zeros(n,n)); %constraint 3 block (3,2)
lmiterm([-3 4 2 0],zeros(m,n)); %constraint 3 block (4,2)
lmiterm([-3 4 3 0],zeros(m,n)); %constraint 3 block (4,3)
lmiterm([-3 3 3 gamma],1,eye(n)); %constraint 3 block (4,3)
lmiterm([-3 4 4 gamma],1,eye(m)); %constraint 3 block (4,3)
 
lmiterm([-4 1 1 Q],1,1)             %constraint 4 block (1,1) F4=([Q Y'*H_theta(1,:)';
lmiterm([-4 2 1 Y],H_theta(1,:),1)  %constraint 4 block (2,1)      H_theta(1,:)*Y 1]>=0);
lmiterm([-4 2 2 0],1)               %constraint 4 block (2,2)

lmiterm([-5 1 1 Q],1,1)             %constraint 5 block (1,1)  F5=([Q Y'*H_theta(2,:)';
lmiterm([-5 2 1 Y],H_theta(2,:),1)  %constraint 5 block (2,1)       H_theta(2,:)*Y 1]>=0);
lmiterm([-5 2 2 0],1)               %constraint 5 block (2,2)

lmiterm([-6 1 1 Q],1,1)             %constraint 6 block (1,1)  F6=([Q Y'*H_theta(3,:)';
lmiterm([-6 2 1 Y],H_theta(3,:),1)  %constraint 6 block (2,1)       H_theta(3,:)*Y 1]>=0);
lmiterm([-6 2 2 0],1)               %constraint 6 block (2,2)

lmiterm([-7 1 1 Q],1,1)             %constraint 7 block (1,1)  F7=([Q Y'*H_theta(4,:)';
lmiterm([-7 2 1 Y],H_theta(4,:),1)  %constraint 7 block (2,1)       H_theta(4,:)*Y 1]>=0);
lmiterm([-7 2 2 0],1)               %constraint 7 block (2,2)

c=[1 0 0 0; 0 1 0 0 ];
lmiterm([8 1 1 Q],c,c');
lmiterm([-8 1 1 0],(c*Q0*c'));


% lmiterm([8 1 1 Q],1,1)              %constraint 8 block (1,1)  F8=(Q<=Q0);
% lmiterm([-8 1 1 0],(Q0))              %constraint 8 block (1,1)

lmis = getlmis;

% solution

options = [1e-4,0,0,0,1];
c=zeros(29,1);
c(29)=1;
[time,sol]=mincx(lmis,c,options);


Q_str = dec2mat(lmis,sol,Q);
Y_str = dec2mat(lmis,sol,Y);
end