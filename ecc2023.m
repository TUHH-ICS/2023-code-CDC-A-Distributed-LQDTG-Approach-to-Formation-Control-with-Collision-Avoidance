%------
% Project: 2023-code-ECC-A-Distributed-Approach-of-Linear-Quadratic-Discrete-Time-Game-to-Multi-agent-Formation-with-Collision-Avoidance
% Copyright: Institute of Control Systems, Hamburg University of Technology
% License: 
% References:
% Authors: Prima Aditya and Herbert Werner
%------

%---------------------------------------------------------------------------------------------
% For Paper, 
% "A Distributed Approach of Linear Quadratic Discrete-Time Game to Multi-agent Formation with Collision Avoidance"
% by Prima Aditya and Herbert Werner
% Copyright (c) Institute of Control Systems, Hamburg University of Technology. All rights reserved.
% Licensed under the GPLv3. See LICENSE in the project root for license information.
%--------------------------------------------------------------------------------------------
clc;clear all;close all;
n  = 2;
N  = 3;%number of agents
M  = 2;%number of edges
h  = n*M;
m  = n*N;
dt = 0.1;
A  = [eye(h), dt*eye(h);
     zeros(h), eye(h)];
B  = [dt^2/2*eye(h); dt*eye(h)];
F  = [eye(m), zeros(m,1), dt*eye(m);
     zeros(1,m), 1, zeros(1,m);
     zeros(m), zeros(m,1), eye(m)];
G  = [dt^2/2*eye(m); zeros(1,m);dt*eye(m)];
d12 = -[-1.5;0];
d13 = -[1.5;0]; 
% d12 = [2.5;1];%agent 1 is 2.5 x and 2.5 y from agent 2
% d13 = [-2.5;1]; %agent 1 is -2.5 x and 2.5 y from agent 3
r   = 0.5;

x_real(:,1) = [10;1;1;1;5;1;1;0;1;0;0;0;0];
xpinv(:,1)  = x_real(:,1);
x(:,1)      = [10;1;1;1;5;1;0;1;0;0;0;0];
D           = [-1 -1;1 0;0 1];
Phi         = kron(kron(eye(2),-D'),eye(n)); 
z(:,1)      = Phi*x(:,1) - [d12;d13;0;0;0;0];
R           = dt*eye(h);

Qa_tild  = mdiag(eye(h),eye(h));                                                             
ep       = 0.001;
eps      = 1/ep;
tf       = 25;
T        = tf/dt;
tol      = 1e-4;
Hor      = 10;
kmax     = 10;
lmax     = 10;
Phia     = kron(-D',eye(n));
Phiai    = pinv(Phia);
u0       = zeros(m,1);
alpha    = 0.1;
udata    = [];
for t = 1:T
    %iteration A
    k = 1;
    z_predict(:,1) = z(:,t);
    K_tild         = randn(h,2*h,Hor);
    while(k <= kmax && (k == 1 || max(abs(squeeze(K_tild - Kold)),[], 'all') > tol))
        Kold  = K_tild;
        for j = 1:Hor%predict the trajectory
           z_predict(:,j+1)= (A + B*K_tild(:,:,j))*z_predict(:,j); 
        end
        QbT_tild           = mdiag(1/(sig(z_predict(1:2,Hor+1)+d12)^2-r^2), 1/(sig(z_predict(1:2,Hor+1)+d12)^2-r^2), 1/(sig(z_predict(3:4,Hor+1)+d13)^2-r^2), 1/(sig(z_predict(3:4,Hor+1)+d13)^2-r^2), 1/(sig(z_predict(1:2,Hor+1)+d12)^2-r^2), 1/(sig(z_predict(1:2,Hor+1)+d12)^2-r^2), 1/(sig(z_predict(3:4,Hor+1)+d13)^2-r^2), 1/(sig(z_predict(3:4,Hor+1)+d13)^2-r^2));
        QT_tild            = dt*( Qa_tild + QbT_tild); 
        P_tild(:,:,Hor+1)  = QT_tild;
        for j = Hor:-1:1
            Qb_tild(:,:,j) = mdiag(1/(sig(z_predict(1:2,j)+d12)^2-r^2), 1/(sig(z_predict(1:2,j)+d12)^2-r^2), 1/(sig(z_predict(3:4,j)+d13)^2-r^2), 1/(sig(z_predict(3:4,j)+d13)^2-r^2), 1/(sig(z_predict(1:2,j)+d12)^2-r^2), 1/(sig(z_predict(1:2,j)+d12)^2-r^2), 1/(sig(z_predict(3:4,j)+d13)^2-r^2), 1/(sig(z_predict(3:4,j)+d13)^2-r^2));
            Q_tild(:,:,j)  = dt*(Qa_tild + Qb_tild(:,:,j)); 
            %partial derivatives
            dQdq1x(:,:,j)= [ -(2*abs(d12(1,:) + z_predict(1,j))*sign(d12(1,:) + z_predict(1,j))*(eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2)),                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0, -(2*abs(d12(1,:) + z_predict(1,j))*sign(d12(1,:) + z_predict(1,j))*(eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2)), 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0, -(2*abs(d12(1,:) + z_predict(1,j))*sign(d12(1,:) + z_predict(1,j))*(eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2)),                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0, -(2*abs(d12(1,:) + z_predict(1,j))*sign(d12(1,:) + z_predict(1,j))*(eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2)), 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0];
            dQdq1y(:,:,j)=[ -(2*abs(d12(2,:) + z_predict(2,j))*sign(d12(2,:) + z_predict(2,j))*(eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2)),                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0, -(2*abs(d12(2,:) + z_predict(2,j))*sign(d12(2,:) + z_predict(2,j))*(eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2)), 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0, -(2*abs(d12(2,:) + z_predict(2,j))*sign(d12(2,:) + z_predict(2,j))*(eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2)),                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0, -(2*abs(d12(2,:) + z_predict(2,j))*sign(d12(2,:) + z_predict(2,j))*(eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d12(1,:) + z_predict(1,j))^2/eps + abs(d12(2,:) + z_predict(2,j))^2/eps + 1)^(1/2)), 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0;
                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0];
            dQdq2x(:,:,j)=[ 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0, -(2*abs(d13(1,:) + z_predict(3,j))*sign(d13(1,:) + z_predict(3,j))*(eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2)),                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0, -(2*abs(d13(1,:) + z_predict(3,j))*sign(d13(1,:) + z_predict(3,j))*(eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2)), 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0, -(2*abs(d13(1,:) + z_predict(3,j))*sign(d13(1,:) + z_predict(3,j))*(eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2)),                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0, -(2*abs(d13(1,:) + z_predict(3,j))*sign(d13(1,:) + z_predict(3,j))*(eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2))];
            dQdq2y(:,:,j)= [ 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0, -(2*abs(d13(2,:) + z_predict(4,j))*sign(d13(2,:) + z_predict(4,j))*(eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2)),                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0, -(2*abs(d13(2,:) + z_predict(4,j))*sign(d13(2,:) + z_predict(4,j))*(eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2)), 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0, -(2*abs(d13(2,:) + z_predict(4,j))*sign(d13(2,:) + z_predict(4,j))*(eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2)),                                                                                                                                                                                                                                                         0;
                         0, 0,                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                         0, 0, 0,                                                                                                                                                                                                                                                         0, -(2*abs(d13(2,:) + z_predict(4,j))*sign(d13(2,:) + z_predict(4,j))*(eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps))/(((eps*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2) - eps)^2 - r^2)^2*(abs(d13(1,:) + z_predict(3,j))^2/eps + abs(d13(2,:) + z_predict(4,j))^2/eps + 1)^(1/2))];
            %Riccati gain
            P_tild(:,:,j)  = Q_tild(:,:,j) + [z_predict(:,j)'*dQdq1x(:,:,j);z_predict(:,j)'*dQdq1y(:,:,j);z_predict(:,j)'*dQdq2x(:,:,j);z_predict(:,j)'*dQdq2y(:,:,j);zeros(4,2*h)] + A'*P_tild(:,:,j+1)*A - ...
                              A'*P_tild(:,:,j+1)*B*inv(R+B'*P_tild(:,:,j+1)*B)*B'*P_tild(:,:,j+1)*A;
            K_tild(:,:,j)  = -inv(R+B'*P_tild(:,:,j+1)*B)*B'*P_tild(:,:,j+1)*A;
        end
        k    = k+1;
    end 
    a(:,t)   = K_tild(:,:,1)*z(:,t);
    z(:,t+1) = A*z(:,t) + B*K_tild(:,:,1)*z(:,t); 
    % centralised solution
    for v = 1:N
         upinv(:,t) = Phiai * a(:,t);
    end
    xpinv(:,t+1) = F * xpinv(:,t) + G*upinv(:,t); 
    % iteration B
    l = 1;
    if t > 1
        u0 = usol;
    end
    while (l <= lmax)
        u0_temp = (eye(m) - 2*alpha*Phia'*Phia)*u0(:,l) + 2*alpha*Phia'*a(:,t); 
        u0 = [u0 u0_temp];
    l = l+1;
    end
    udata = [udata,u0];
    %toc;
    usol          = u0(:,end);
    uhat(:,t)     = usol;   
    x_real(:,t+1) = F * x_real(:,t) + G * uhat(:,t);
end
% 
% % xall = xlsread('xall.xlsx','B2:IR14');
% xall = xlsread('xall.xlsx','Sheet2','B16:IR28');
% figure(1)
% plot(x_real(1,:),x_real(2,:),x_real(3,:),x_real(4,:),x_real(5,:),x_real(6,:),'linewidth',2)
% hold all
% s1=plot(xall(1,:),xall(2,:),'--','linewidth',2);
% s1.Color = "#0072BD";
% s2=plot(xall(3,:),xall(4,:),'--','linewidth',2);
% s2.Color = "#D95319";
% s3=plot(xall(5,:),xall(6,:),'--','linewidth',2);
% s3.Color = 	"#EDB120";
% p1=plot(x_real(1,end),x_real(2,end),'o','linewidth',3);
% p1.Color = "#0072BD";
% p2=plot(x_real(3,end),x_real(4,end),'o','linewidth',3);
% p2.Color = "#D95319";
% p3=plot(x_real(5,end),x_real(6,end),'o','linewidth',3);
% p3.Color = 	"#EDB120";
% yline(1,'--k')
% circ1 = viscircles([x_real(1,end) x_real(2,end)],r,'Color',"#0072BD",'linestyle',':');
% circ2 = viscircles([x_real(3,end) x_real(4,end)],r,'Color',"#D95319",'linestyle',':');
% circ3 = viscircles([x_real(5,end) x_real(6,end)],r,'Color',"#EDB120",'linestyle',':');
% legend('Agent 1','Agent 2','Agent 3','location','best','fontsize',14)
% xlabel('x-axis','fontsize',12)
% ylabel('y-axis','fontsize',12)
% xlim([0 11]);
% ylim([0 11]);
% grid on

% figure(2)
% subplot(2,1,1)
% plot(0:dt:tf,x_real(8,:),0:dt:tf,x_real(10,:),0:dt:tf,x_real(12,:),'linewidth',2)
% legend('Agent 1','Agent 2','Agent 3','location','best','fontsize',14)
% xlabel('time','fontsize',12)
% ylabel('velocity in x-axis','fontsize',12)
% grid on
% subplot(2,1,2)
% plot(0:dt:tf,x_real(9,:),0:dt:tf,x_real(11,:),0:dt:tf,x_real(13,:),'linewidth',2)
% legend('Agent 1','Agent 2','Agent 3','location','best','fontsize',14)
% xlabel('time','fontsize',12)
% ylabel('velocity in y-axis','fontsize',12)
% grid on

% Agent 1 for iteration Iter = 10;
fig = figure(97);clf;
ax  = axes;
plot(ax,udata(1,:),'*b')
hold on
for i = 1:length(upinv(1,:))
    plot(ax,(lmax+1)*i,upinv(1,i),'xr','linewidth',5);
end
grid on
xlabel('Time/ iterations')
ylabel('Control input of Agent 1')
legend('Distributed approach','Centralized solution','fontsize',12,'location','southeast');
xlim([0 1000])
options.axes.Names = {'Position','XLim'};
options.axes.Values = {[.5 .35 .35 .35],[100,200]};
options.rectangle.Names = {};
options.rectangle.Values = {};
options.arrows.Names = {'HeadLength','HeadWidth'};
options.arrows.Values = {8,8};
[zoom_utils] = zoom_plot(ax,options);

% Agent 3 for iteration Iter = 10;
fig = figure(97);clf;
ax  = axes;
plot(ax,udata(5,:),'*b')
hold on
for i = 1:length(upinv(5,:))
    plot(ax,(lmax+1)*i,upinv(5,i),'xr','linewidth',5);
end
grid on
xlabel('Time/ iterations')
ylabel('Control input of Agent 3')
legend('Distributed approach','Centralized solution','fontsize',12,'location','northeast');
xlim([0 1000])
options.axes.Names = {'Position','XLim'};
options.axes.Values = {[0.53 0.2 .35 .35],[100,200]};
options.rectangle.Names = {};
options.rectangle.Values = {};
options.arrows.Names = {'HeadLength','HeadWidth'};
options.arrows.Values = {8,8};
[zoom_utils] = zoom_plot(ax,options);
 
% figure(3)
% subplot(2,1,1)
% plot(0:dt:tf,z(1,:),0:dt:tf,z(2,:),'linewidth',2)
% legend('Edge 1','Edge 2','location','best','fontsize',14)
% xlabel('time','fontsize',12)
% ylabel('relative positions','fontsize',12)
% grid on
% subplot(2,1,2)
% plot(0:dt:tf,z(3,:),0:dt:tf,z(4,:),'linewidth',2)
% legend('Edge 1','Edge 2','location','best','fontsize',14)
% xlabel('time','fontsize',12)
% ylabel('relative velocities','fontsize',12)
% grid on
 
% figure(99)
% set(gcf,'color','w');
% filename='myfig';
% counter=1;
% for i=1:tf/dt
%     plot(x_real(1,i),x_real(2,i),'d',x_real(3,i),x_real(4,i),'o',x_real(5,i),x_real(6,i),'s','linewidth',3);
%     hold on
%     grid on
%     p1 = plot(x_real(1,1:i),x_real(2,1:i),'linewidth',2);
%     p1.Color = "#0072BD";
%     p2 = plot(x_real(3,1:i),x_real(4,1:i),'linewidth',2);
%     p2.Color = "#D95319";
%     p3 = plot(x_real(5,1:i),x_real(6,1:i),'linewidth',2);
%     p3.Color = 	"#EDB120";
%     legend('Agent 1','Agent 2','Agent 3','fontsize',14)
%     xlabel('x-axis','fontsize',14);
%     ylabel('y-axis','fontsize',14);
%     xlim([-1 12]);
%     ylim([0 12]);
%     frame = getframe(99);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%     imwrite(imind,cm,strcat(filename,num2str(counter),'.png'));
%     counter=counter+1;
%     pause(0.1)   
%     if i~= length(1:(tf/dt))
%     clf
%     end
% end
% print -depsc myfig.png
