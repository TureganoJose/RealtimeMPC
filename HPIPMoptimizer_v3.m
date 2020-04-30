function [ X,U,info ] = HPIPMoptimizer_v3(HorizonIter,NHorizon)
nx = 5; % number of car states
nu = 1; % number of control states


% dims
dims = hpipm_ocp_qp_dim(NHorizon);

% Number of state variables
dims.set('nx', (nx+nu), 0, N);
dims.set('nu', nu, 0, N-1);

% Number of boundary conditions
dims.set('nbx', (nx+nu), 0, N);
dims.set('nbu', nu, 0, N-1);


nbx = nx;
nbu = nu;
dim.set('nx', nx, 0, NHorizon);
dim.set('nu', nu, 0, NHorizon); % controls
dim.set('nbx', nbx, NHorizon); % state bounds
dim.set('nbu', nbu, 0, NHorizon); % control bounds
%dim.set('ng', ng, 0, N); % general linear constraints
%dim.set('ns', ns, 0, N); % slacks


% qp
qp = hpipm_ocp_qp(dims);
%% Equility Constraints
x0 =[HorizonIter(1).x0;stage(1).u0];
for i = 0:NHorizon-1
   qp.set('A', HorizonIter(i+1).Ak, i); 
   qp.set('B', HorizonIter(i+1).Bk, i); 
   qp.set('b', HorizonIter(i+1).gk, i); 
end

%% Cost
for i = 0:NHorizon
    qp.set('Q', HorizonIter(i+1).Qk, i);
    qp.set('q', HorizonIter(i+1).fk, i);
    if i<NHorizon
        qp.set('R', HorizonIter(i+1).Rk, i); 
    end
end

%% Bounds
%qp.print_C_struct();
for i = 0:NHorizon
    qp.set('Jbx', eye(nx+nu), i)
    if i == 0
        qp.set('lbx', x0, 0)
        qp.set('ubx', x0, 0)
    else
        qp.set('lbx', HorizonIter(i+1).lb(1:nx+nu), i)
        qp.set('ubx', HorizonIter(i+1).ub(1:nx+nu), i)
    end
    
    if i<NHorizon
        qp.set('Jbu', eye(nu), i)
        qp.set('lbu', HorizonIter(i+1).lb(nx+nu+1:nx+nu+nu), i)
        qp.set('ubu', HorizonIter(i+1).ub(nx+nu+1:nx+nu+nu), i)
    end
end
    
%qp.print_C_struct();























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


for i = 2:NHorizon-1
    Aeq((i-1)*nx+1:i*nx, (i-1)+(i-2)*nx:(i-1)+(i-1)*nx-1) = -HorizonIter(i-1).Ak;
    Aeq((i-1)*nx+1:i*nx, (i-1)+(i-1)*nx)= -HorizonIter(i-1).Bk;
    Aeq((i-1)*nx+1:i*nx, (i-1)+(i-1)*nx+1: (i-1)+(i-1)*nx+nx   )= eye(nx);

    beq((i-1)*nx+1:i*nx) = [HorizonIter(i-1).gk];
end
Aeq(nx*NHorizon +1, nx+nu) = 1;
beq(nx*NHorizon +1, 1) = HorizonIter(1).u0;

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