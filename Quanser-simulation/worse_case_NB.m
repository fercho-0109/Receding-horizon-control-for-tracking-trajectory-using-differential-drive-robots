function [Q0,Y0]=worse_case_NB(Err0,A,B,ru,Bp,Qe,Qu,Cq,Dqu,Qbd)

n=size(A,1);
m=size(B,2);

Q=sdpvar(n,n); % create the unknow variables
gamma=sdpvar(1,1); 
Y = sdpvar(m,n);
lambda = diag(sdpvar(2,1));

% LMI constrains
F1=([1 Err0';
    Err0 Q ]>=0);

F2=([   Q       Y'*sqrt(Qu)   Q*sqrt(Qe)    Q*Cq'+Y'*Dqu'  Q*A'+Y'*B'
    sqrt(Qu)*Y  gamma*eye(m)  zeros(m,n)    zeros(m)       zeros(m,n)
    sqrt(Qe)*Q  zeros(n,m)    gamma*eye(n)  zeros(n,2)     zeros(n)
    Cq*Q+Dqu*Y  zeros(m)      zeros(m,n)    lambda         zeros(m,n)
    A*Q+B*Y     zeros(n,m)    zeros(n,n)    zeros(n,m)     Q-Bp*lambda*Bp']>=0);

F3=([ru*ru*eye(m) Y;
    Y' Q]>=0);

h=[1 0 0 0;0 1 0 0];

F4=([h*Q*h']>=Qbd);

F5=(lambda>=0);

F=F1+F2+F3+F4+F5;

% solution
opts=sdpsettings('solver','mosek','verbose',0);
solvesdp(F,gamma,opts);

Q0=double(Q);
Y0=double(Y);
end