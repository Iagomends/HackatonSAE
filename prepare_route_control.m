function plan = prepare_route_control(c)
%PREPARE_ROUTE_CONTROL Full-route forecast and backward degradation value table.
% Hard reachability is analytic; grid interpolation affects cost, not viability.
t=c.time; N=numel(t); p=c.p;
a=[(c.speed(1)-c.v0)/c.dt;diff(c.speed)/c.dt];
force=c.mass*a+0.5*c.rho*c.Cd*c.area*c.speed.*abs(c.speed)+ ...
    c.Crr*c.mass*c.g*cos(c.grade)*sign(c.speed)+c.mass*c.g*sin(c.grade);
plan.torque=force*c.radius; plan.wheelSpeed=c.speed/c.radius;
plan.power=plan.torque.*plan.wheelSpeed;
plan.distance=[0;cumsum(abs(c.speed(1:end-1))*c.dt)];
plan.progress=plan.distance/max(plan.distance(end),eps);
plan.remainingDistance=plan.distance(end)-plan.distance;
plan.traction=zeros(N,1); plan.maxCharge=zeros(N,1);
plan.bestPoint=zeros(24,N); plan.mechanicalFeasible=true(N,1);
p(13)=2; % Minimum positive current dominates traction aging and SOC use.
for k=1:N
    wr=plan.wheelSpeed(k)*c.final;
    if plan.power(k)<0
        tr=plan.torque(k)*c.etaDrive/c.final;
        q=ecvt_max_regen(tr,wr,p);
        plan.maxCharge(k)=max(0,-q(21));
        plan.mechanicalFeasible(k)=abs(wr)<=p(1)/p(2)*p(7)+(1+p(1)/p(2))*p(8);
    else
        tr=plan.torque(k)/(c.final*c.etaDrive);
        q=ecvt_operating_point(tr,wr,p);
        plan.traction(k)=max(0,q(21));
        plan.mechanicalFeasible(k)=logical(q(14));
    end
    plan.bestPoint(:,k)=q;
end
% N samples represent N-1 forward-Euler control intervals.
plan.traction(end)=0; plan.maxCharge(end)=0;
% Terminal electrical energy forecast. This is demand, not SOC converted
% using nominal voltage; battery resistance remains included in the forecast.
tractionPower=(p(14)-p(15)*plan.traction).*plan.traction;
chargingPower=(p(14)+p(15)*plan.maxCharge).*plan.maxCharge;
plan.remainingTractionEnergy_kWh=flipud(cumsum(flipud(tractionPower*c.dt)))/3.6e6;
plan.remainingRecoverableEnergy_kWh=flipud(cumsum(flipud(chargingPower*c.dt)))/3.6e6;
step=c.dt/(3600*p(16));
plan.required=zeros(N,1); plan.noRegenRequired=zeros(N,1);
plan.required(end)=c.terminalSOC; plan.noRegenRequired(end)=c.terminalSOC;
plan.reachable=false(N,1); plan.reachable(end)=c.terminalSOC<=1;
for k=N-1:-1:1
    % Predictive control must remain above the numerical path reserve at
    % every intermediate sample, even when a later braking event can
    % recover enough energy to meet the terminal SOC alone.
    plan.required(k)=max(c.predictiveMin,plan.required(k+1)+ ...
        step*(plan.traction(k)-plan.maxCharge(k)));
    plan.noRegenRequired(k)=max(0,plan.noRegenRequired(k+1)+step*plan.traction(k));
    plan.reachable(k)=plan.reachable(k+1) && plan.required(k)<=1 && ...
        plan.mechanicalFeasible(k);
end
plan.socGrid=zeros(c.socNodes,N); plan.value=zeros(c.socNodes,N);
plan.config=c;
for k=N:-1:1
    plan.socGrid(:,k)=linspace(plan.required(k), ...
        max(plan.required(k),min(1,plan.noRegenRequired(k))),c.socNodes)';
    if k<N
        [~,plan.value(:,k)]=route_regen_policy(plan.socGrid(:,k),k,plan,c);
    end
end
plan.initiallyFeasible=plan.reachable(1) && c.initialSOC>=plan.required(1)-1e-12;
end
