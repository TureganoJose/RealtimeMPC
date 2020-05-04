
function [ X,U,info ] = QuadProgoptimizer(HorizonIter,NHorizon)
nx = 5; % number of car states
nu = 1; % number of control states


H = zeros( (nx+nu)*NHorizon,(nx+nu)*NHorizon);
f = zeros((nx+nu)*NHorizon,1);
for i = 1:NHorizon
	H_i = blkdiag(HorizonIter(i).Qk,HorizonIter(i).Rk);
    H( (i-1)*(nx+nu)+1:i*(nu+nx),(i-1)*(nx+nu)+1:i*(nu+nx))= H_i;
end

% Sizes require +1 to accomodate the initial condition u0
Aeq = zeros( nx*NHorizon +1,(nx+nu)*NHorizon);
beq = zeros( nx*NHorizon +1,1);

% First rows to stablish the initial conditions I*x0 = x0(imposed initial
% condition)
% Last row for u0 = u0(imposed initial input)
Aeq(1:nx,1:nx)=eye(nx);
beq(1:nx,:)= HorizonIter(1).x0;

Aeq(nx+1, nx+nu) = 1;
beq(nx+1, 1) = HorizonIter(1).u0;


for i = 2:NHorizon
    Aeq((i-1)*nx+2:i*nx+1, (i-1)+(i-2)*nx:(i-1)+(i-1)*nx-1) = -HorizonIter(i-1).Ak;
    Aeq((i-1)*nx+2:i*nx+1, (i-1)+(i-1)*nx)= -HorizonIter(i-1).Bk;
    Aeq((i-1)*nx+2:i*nx+1, (i-1)+(i-1)*nx+1: (i-1)+(i-1)*nx+nx   )= eye(nx);

    beq((i-1)*nx+2:i*nx+1) = [HorizonIter(i-1).gk];
end


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

[z,cost,exitflag] = quadprog(H,f,A,b,Aeq,beq,LB,UB,[],options);
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


info.QPtime = QPtime;
info.cost = cost;
if exitflag == 1
    info.exitflag = 0;
else
    info.exitflag = 1;
end

end


