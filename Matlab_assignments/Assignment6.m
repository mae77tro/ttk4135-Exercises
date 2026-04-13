% Local SQP method (Algorithm 18.1 in Nocedal & Wright)

% Define objective function and gradient
f = @(x) x(1) + x(2);  
df = @(x) [1; 1];

% Define constraint function and Jacobian
c = @(x) (x(1)-3).^2 + (x(2)-2).^2 - 1;
A = @(x) [2*(x(1)-3) 2*(x(2)-2)]; % TODO: Implement Jacobian

% Define Hessian of Lagrangian
HL = @(x,lambda) [-2*lambda 0;
                  0     -2*lambda];% TODO: Implement Hessian

% Initial guess
x0 = [0;0]; lambda0 = -1;
x(:,1) = x0; lambda(:,1) = lambda0; % Store values

% Plot constraint
plot(x(1,1),x(2,1),'rx','linewidth',2); hold on; % Plot x0
xm=5; ym = 4;
fimplicit(@(x,y) (x-3).^2 + (y-2).^2 - 1,[-xm xm],'b','linewidth',1.5); % plot constraint c(x) = 0
xlim([0,xm]); ylim([0,ym]);

% Plot contours of objective function
[x1,x2]=meshgrid(0:0.01:xm);
ff = @(x,y) f([x,y]); z = arrayfun(ff,x1,x2);
contour(x1,x2,z,[0:8],'--')

options = optimoptions('quadprog','Algorithm','active-set','Display','none');

for i = 1:10,
    % TODO: Replace the X's according to (18.7)
    [p,fval,exitflag,output,lo] = quadprog(HL(x(:,i),lambda(:,i)),df(x(:,i)),[],[],A(x(:,i)),-c(x(:,i)),[],[],x(:,i),options);  % (18.7)
    
    l = -lo.eqlin; % Lagrangian multiplier
    
    x(:,i+1) = x(:,i)+p; % TODO: Implement update for x
    lambda(:,i+1) = l; % TODO: Implement update for lambda
    
    plot(x(1,i+1),x(2,i+1),'rx','linewidth',2);    
end