function [qpunto] = DiffDrive(t,q,v,w)

x=q(1); 
y=q(2); 
theta=q(3);

q1punto=cos(theta)*v;
q2punto=sin(theta)*v;
q3punto = w;


qpunto= [q1punto;q2punto;q3punto];