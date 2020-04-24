
function [ X,U,info ] = QuadProgoptimizer_v3(HorizonIter,NHorizon)
nx = 5; % number of car states
nu = 1; % number of control states


H = zeros( (nx+nu)*NHorizon,(nx+nu)*NHorizon);
f = zeros((nx+nu)*NHorizon,1);
for i = 1:NHorizon
	H_i = blkdiag(HorizonIter(i).Qk,HorizonIter(i).Rk);
    H( (i-1)*(nx+nu)+1:i*(nu+nx),(i-1)*(nx+nu)+1:i*(nu+nx))= H_i;
end

% Sizes require +nx to accomodate the initial condition x0
Aeq = zeros( (nx)*NHorizon+nx,(nx+nu)*NHorizon);
beq = zeros((nx)*NHorizon+nx,1);

for i = 1:NHorizon-1
    Aeq((i-1)*nx+1:i*nx, (i-1)*(nx+nu)+1:(i-1)*(nx+nu)+nx) = -HorizonIter(i).Ak;
    Aeq((i-1)*nx+1:i*nx, (i-1)*(nx+nu)+nx+1:(i-1)*(nx+nu)+nx+nu)= -HorizonIter(i).Bk;
    Aeq((i-1)*nx+1:i*nx, (i-1)*(nx+nu)+nx+nu+1:(i-1)*(nx+nu)+nx+nu+nx)= eye(nx);

    beq( (i-1)*nx+1:i*nx,1) = [HorizonIter(i).gk];
end

% Last rows to stablish the initial conditions I*x0 = x0*(optimised value)
Aeq(nx*NHorizon+1:(nx)*NHorizon+nx,1:nx)=eye(nx);
beq(nx*NHorizon+1:(nx)*NHorizon+nx,:)= HorizonIter(1).x;

% figure(3)
% plot(1:NHorizon,x_test)
% hold on
% plot(1:NHorizon,HorizonIter(i).x)

LB = zeros((nx+nu)*NHorizon,1);
UB = zeros((nx+nu)*NHorizon,1);

for i=1:NHorizon
%     LB( (i-1)*(nx+nu)+1:i*(nu+nx),1) = [HorizonIter(i).x;-0.5]  -[0.01;0.01;0.1;1;0.5;0];
%     UB( (i-1)*(nx+nu)+1:i*(nu+nx),1) = [HorizonIter(i).x;0.5]   +[0.01;0.01;0.1;1;0.5;0];
    LB( (i-1)*(nx+nu)+1:i*(nu+nx),1) = HorizonIter(i).lb;
    UB( (i-1)*(nx+nu)+1:i*(nu+nx),1) = HorizonIter(i).ub;
end

A = [];
b = [];
options = optimoptions(@quadprog,'MaxIterations',1000,'Display','off','ConstraintTolerance',1e-8); %'iter-detailed'
tic

[z,~,exitflag] = quadprog(H,f,A,b,Aeq,beq,LB,UB,[],options);
QPtime = toc;

X = zeros(nx,NHorizon);
U = zeros(nu,NHorizon);

if exitflag == 1
    for i = 1:NHorizon
        X(1:nx,i) = z(   (i-1)*(nx+nu)+1:(i-1)*(nu+nx)+nx );
        U(1:nu,i) = z((i-1)*(nu+nx)+nx+1);
    end
end

xNew(:,1)= X(:,1);
for i=1:NHorizon-1
    xNew(:,i+1) = HorizonIter(i).Ak * X(:,i) + HorizonIter(i).Bk * U(i) + HorizonIter(i).gk;
end

% figure(3)
% subplot(3,2,1)
% plot((1:NHorizon),X(1,:),'b.')
% hold on
% plot((1:NHorizon),xNew(1,1:NHorizon),'r.')
% hold on
% title('v vs time')
% subplot(3,2,2)
% plot((1:NHorizon),X(2,:),'b.')
% hold on
% plot((1:NHorizon),xNew(2,1:NHorizon),'r.')
% hold on
% title('r vs time')
% subplot(3,2,3)
% plot((1:NHorizon),X(3,:),'b.')
% hold on
% plot((1:NHorizon),xNew(3,1:NHorizon),'r.')
% hold on
% title('d_phi vs time')
% subplot(3,2,4)
% plot((1:NHorizon),X(4,:),'b.')
% hold on
% plot((1:NHorizon),xNew(4,1:NHorizon),'r.')
% hold on
% title('lateral error vs time')
% subplot(3,2,5)
% plot((1:NHorizon),X(5,:),'b.')
% hold on
% plot((1:NHorizon),xNew(5,1:NHorizon),'r.')
% hold on
% title('Wheel_angle vs time')
% 


info.QPtime = QPtime;
if exitflag == 1
    info.exitflag = 0;
else
    info.exitflag = 1;
end

end


