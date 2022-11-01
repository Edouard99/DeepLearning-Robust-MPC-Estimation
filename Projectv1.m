clear
n = 2;
A=[1 1;0 1];
B=[1 ; 1];
Q=[1 0;0 1];
R=0.1;
rQ=sqrtm(Q);
rR=sqrt(R);
Ymax=sdpvar(2,2);
Yr=sdpvar(2,2);
Xr=sdpvar(1,2);
F=[];
M=[Yr Yr*A'+Xr'*B' Yr*rQ Xr'*rR;A*Yr+B*Xr Yr zeros(2,2) [0;0];rQ*Yr zeros(2,2) eye(2) [0;0];rR*Xr [0 0] [0 0] 1];
F=[-M<=0];
F=[F, Yr<=Ymax];
sol =optimize(F,-logdet(Ymax),sdpsettings('solver','sdpt3'));
Y=value(Yr);
X=value(Xr);
P=inv(Y);
K=X*P;
Q=P-(A+B*K)'*P*(A+B*K);
%%
N=10;
P_val=eig(P);
Q_val=eig(Q);
eps_l=min(Q_val);
c_del_u=max(P_val);
c_del_l=min(P_val);
roh=1-(eps_l/(2*c_del_u));
kmax=norm(K);
kmax_bar=norm(inv(sqrtm(P))*K');
delta_loc=1000;
eta=0.005;
H=[1/3 0;-1/3 0;0 1/3;0 -1/3];
L=[1/(2-eta);-1/(2-eta)];
lambda=norm(B);
val1=kmax_bar*norm(L,"inf");
val2=norm(H,"inf")/(sqrt(c_del_l));
val=[val1 ;val2];
epsilon=eta*lambda*sqrt(c_del_u)*max(val);
epsilon_k=[];
for k = 0:N
    epsilon_k=[epsilon_k; epsilon*((1-sqrt(roh)^k)/(1-sqrt(roh)))];
end
%%
% [Kter,Ster,eter]=lqr(A,B,Q,R)
% phic=A-B*Kter
% Qter=-Q+Kter'*R*Kter +diag([1 1])
% Pter=sylvester(phic',-inv(phic),Qter*inv(phic))
%%
model = LTISystem('A', A, 'B', B);
model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);
Pset = model.LQRPenalty;
Tset = model.LQRSet;
model.x.with('terminalPenalty');
model.x.with('terminalSet');
model.x.terminalPenalty = Pset;
model.x.terminalSet = Tset;
mpc = MPCController(model, N);
Y = mpc.toYALMIP();
for k=1:N
    Y.constraints = Y.constraints + [ H*Y.variables.x(:, k)<=(1-epsilon_k(k))*[1;1;1;1] ];
    Y.constraints = Y.constraints + [ L*Y.variables.u(:, k)<=(1-epsilon_k(k))*[1;1]];
end
mpc.fromYALMIP(Y)
%%
empc = mpc.toExplicit();
empc.clicksim()
%%
[u, feasible, openloop] = mpc.evaluate([3;3])
%%
x1_mesh = -3:0.05:3;
x2_mesh= -3:0.05:3;
[X1,X2]=meshgrid(x1_mesh,x2_mesh);
n=size(X1,1);
m=size(X1,2);
point_1=[];
point_2=[];
u_sol=[];
feas=[];
for k = 1:n
    for j =1:m
        point=[X1(k,j); X2(k,j)];
        [u, feasible, openloop] = mpc.evaluate(point);
        point_1=[point_1 ; point(1)];
        point_2=[point_2 ; point(2)];
        u_sol=[u_sol ; u];
        feas=[feas; feasible];
    end
end
T=table(point_1,point_2,u_sol,feas)
writetable(T,'C:/Users/Edouard/Documents/MATLAB/tabledata.txt');