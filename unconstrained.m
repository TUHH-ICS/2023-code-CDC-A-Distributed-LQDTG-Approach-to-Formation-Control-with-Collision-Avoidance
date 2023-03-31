%Formation control problem without collision avoidance (unconstrained)
%agents
clc;clear all;close all;
n   = 2;    %number of dimensional plane
N   = 4;    %number of agents
h   = n*N;  %adjusting 
dt  = 0.1;  %sampling time
A   = [eye(h), zeros(h,1), dt*eye(h);
         zeros(1,h), 1, zeros(1,h);
         zeros(h), zeros(h,1), eye(h)];      %state matrix
B   = [dt^2/2*eye(h); zeros(1,h);dt*eye(h)]; %input matrix
d12 = [1.5;1]; d13 = [0;2]; d23 = [-1.5;1]; d24 = [-3;0]; d34 = [-1.5;-1]; %desired displacements
d   = [d12;d13;d23;d24;d34]; r = 0.5;        %safety distance
p4  = [15;3.5];   p3 = [0;5]; p2 = [12;1]; p1 = [3.5;1]; %initial position
v1  = [0.5;1];    v2 = [0;0];   v3 = [0;0]; v4 = [0;0];  %initial velocities
x_real(:,1) = [p1;p2;p3;p4;1;v1;v2;v3;v4];   %initial state
R   = dt*eye(h);   %control weight
D   = kron([1 1 0 0 0;
            -1 0 1 1 0;
            0 -1 -1 0 1;
            0 0 0 -1 -1],eye(n)); %incidence matrix
L    = D*D';     %Laplacian matrix
W    = eye(n);   %weight
Dbar = -D*W*d; 
Dcap = d'*W*d;
Qa   = dt*[L, Dbar, zeros(h);
        Dbar', Dcap, zeros(1,h);
        zeros(h), zeros(h,1),L];  %state weighting matrix                                                          
ep   = 0.01;
eps  = 1/ep;
tf   = 4;     %simulation time in seconds
T    = tf/dt; %finitie time
tol  = 1e-3;
Hor  = 10;    %predicted horizon
QT   = Qa;    %terminal weight
kmax = 12;
for t = 1:T
    k = 1;
    K = 1e-3*zeros(N*n,2*N*n+1,Hor);
    while(k <= kmax && (k == 1 || max(abs(squeeze(K - Kold)),[], 'all') > tol))
        Kold = K;
        P(:,:,Hor+1)  = QT;
        for j = Hor:-1:1
            P(:,:,j)   = Qa + A'*P(:,:,j+1)*A - A'*P(:,:,j+1)*B*inv(R+B'*P(:,:,j+1)*B)*B'*P(:,:,j+1)*A;
            K(:,:,j)   = -inv(R+B'*P(:,:,j+1)*B)*B'*P(:,:,j+1)*A;
        end 
        k     = k+1;
    end 
    x_real(:,t+1) = A*x_real(:,t) + B*K(:,:,1)*x_real(:,t); %should use the warm start
end

xall = xlsread('xall.xlsx','5edgesunconstrained','B2:GT18');
figure(1)
plot(x_real(1,:),x_real(2,:),x_real(3,:),x_real(4,:),x_real(5,:),x_real(6,:),x_real(7,:),x_real(8,:),'linewidth',3)
hold all
s1=plot(xall(1,:),xall(2,:),':','linewidth',2.5);
s1.Color = "#0072BD";
s2=plot(xall(3,:),xall(4,:),':','linewidth',2.5);
s2.Color = "#D95319";
s3=plot(xall(5,:),xall(6,:),':','linewidth',2.5);
s3.Color = 	"#EDB120";
s4=plot(xall(7,:),xall(8,:),':','linewidth',2.5);
s4.Color = 	"#7E2F8E";
p1=plot(x_real(1,end),x_real(2,end),'o','linewidth',2.5);
p1.Color = "#0072BD";
p2=plot(x_real(3,end),x_real(4,end),'o','linewidth',2.5);
p2.Color = "#D95319";
p3=plot(x_real(5,end),x_real(6,end),'o','linewidth',2.5);
p3.Color = 	"#EDB120";
p4=plot(x_real(7,end),x_real(8,end),'o','linewidth',2.5);
p4.Color = 	"#7E2F8E";
circ1 = viscircles([x_real(1,end) x_real(2,end)],r,'Color',"#0072BD",'linestyle',':');
circ2 = viscircles([x_real(3,end) x_real(4,end)],r,'Color',"#D95319",'linestyle',':');
circ3 = viscircles([x_real(5,end) x_real(6,end)],r,'Color',"#EDB120",'linestyle',':');
circ4 = viscircles([x_real(7,end) x_real(8,end)],r,'Color',"#7E2F8E",'linestyle',':');
legend('Agent 1','Agent 2','Agent 3','Agent 4','location','northwest','fontsize',14)
xlabel('x-axis','fontsize',12)
ylabel('y-axis','fontsize',12)
xlim([-1 16]);
ylim([0 13]);
grid on

% figure(98)
% set(gcf,'color','w');
% filename='4agents';
% counter=1;
% vid = VideoWriter('4agents.avi','Motion JPEG AVI');
% open(vid)
% for i=1:tf/dt
%     plot(x_real(1,i),x_real(2,i),'o',x_real(3,i),x_real(4,i),'o',x_real(5,i),x_real(6,i),'o',x_real(7,i),x_real(8,i),'o','linewidth',3);
%     circ1 = viscircles([x_real(1,i) x_real(2,i)],r,'Color',"#0072BD",'linestyle',':');
%     circ2 = viscircles([x_real(3,i) x_real(4,i)],r,'Color',"#D95319",'linestyle',':');
%     circ3 = viscircles([x_real(5,i) x_real(6,i)],r,'Color',"#EDB120",'linestyle',':');
%     circ4 = viscircles([x_real(7,i) x_real(8,i)],r,'Color',"#7E2F8E",'linestyle',':');      
%     hold on
%     grid on
%     p1 = plot(x_real(1,1:i),x_real(2,1:i),':','linewidth',2);
%     p1.Color = "#0072BD";
%     p2 = plot(x_real(3,1:i),x_real(4,1:i),':','linewidth',2);
%     p2.Color = "#D95319";
%     p3 = plot(x_real(5,1:i),x_real(6,1:i),':','linewidth',2);
%     p3.Color = 	"#EDB120";
%     p3 = plot(x_real(7,1:i),x_real(8,1:i),':','linewidth',2);
%     p3.Color = 	"#7E2F8E";
%     legend('Agent 1','Agent 2','Agent 3','Agent 4','fontsize',14,'location','northwest')
%     xlabel('x-axis','fontsize',14);
%     ylabel('y-axis','fontsize',14);
%     xlim([-1 16]);
%     ylim([0 13]);
%     frame = getframe(98);
%     writeVideo(vid,frame);
% 
% %     im = frame2im(frame);
% %     [imind,cm] = rgb2ind(im,256);
% %     imwrite(imind,cm,strcat(filename,num2str(counter),'.png'));
% %     counter=counter+1;
%     pause(0.01)   
%     if i~= length(1:(tf/dt))
%     clf
%     end
% end























                                                                
                                                                
                                                                
                                                                
                                                                
                                                                
                                                                
                                                                
                                                                