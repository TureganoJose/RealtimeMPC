
function [ X,U,info ] = QuadProgoptimizer(HorizonIter,NHorizon)
nx = 5;
nu = 1;
ng = 2;
nz = nx+2*nu;
nxu = nx+nu;


H = zeros( (nx+nu)*NHorizon,(nx+nu)*NHorizon);
f = zeros((nx+nu)*NHorizon,1);
for i = 1:NHorizon
	H_i = blkdiag(HorizonIter(i).Qk,HorizonIter(i).Rk);
    H( (i-1)*(nx+nu)+1:i*(nu+nx),(i-1)*(nx+nu)+1:i*(nu+nx))= H_i;
end

H = 0.5*(H+H');

Aeq = zeros( (nx+nu)*NHorizon,(nx+nu)*NHorizon);
beq = zeros((nx+nu)*NHorizon,1);


for i = 1:NHorizon
	A_i = [-HorizonIter(i).Ak -HorizonIter(i).Bk; zeros(1,nx+nu)];
    Aeq( (i-1)*(nx+nu)+1:i*(nu+nx),(i-1)*(nx+nu)+1:i*(nu+nx))= A_i;
    beq( (i-1)*(nx+nu)+1:i*(nu+nx),1) = [HorizonIter(i).gk;zeros(1,nu)];
end


LB = zeros((nx+nu)*NHorizon,1);
UB = zeros((nx+nu)*NHorizon,1);

for i=1:NHorizon
    LB( (i-1)*(nx+nu)+1:i*(nu+nx),1) = HorizonIter(i).lb;
    UB( (i-1)*(nx+nu)+1:i*(nu+nx),1) = HorizonIter(i).ub;
end

A = [];
b = [];
options = optimoptions(@quadprog,'MaxIterations',1000,'Display','off');
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

info.QPtime = QPtime;
if exitflag == 1
    info.exitflag = 0;
else
    info.exitflag = 1;
end

end