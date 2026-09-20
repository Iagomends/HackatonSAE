function y = route_powertrain(u,c,plan)
%ROUTE_POWERTRAIN Actual route-mode mechanical and electrical dispatch.
% u=[wheel torque; wheel speed; live SOC; time; case]. Case 0 is inactive.
% Case 2 calls only the local operating-point solver; no preview/terminal input.
y=zeros(39,1); caseNumber=round(u(5));
if caseNumber==0, return; end
T=u(1); w=u(2); soc=u(3); power=T*w; p=c.p;
k=min(numel(c.time),max(1,round((u(4)-c.time(1))/c.dt)+1));
if caseNumber==1
    ratio=c.gear*c.final; wm=ratio*w; tm=T/(ratio*c.etaDrive);
    if power<0, tm=T*c.etaDrive/ratio; end
    cap=c.Tmax;
    if abs(wm)>0, cap=min(cap,c.Pmax/abs(wm)); end
    tm=min(max(tm,-cap),cap);
    pm=tm*wm; eta=p(9); pe=pm/eta;
    if pm<0, eta=p(10); pe=pm*eta; end
    if abs(wm)>c.wmax+1e-8
        error('EV:RouteSpeedLimit','Fixed gear cannot follow the imposed route speed.');
    end
    I=batteryCurrent(pe,p); cr=abs(I)/p(16); ddot=route_aging_rate(I,p);
    n=p(17); kd=interp1(p(18:17+n),p(18+n:17+2*n), ...
        min(cr,p(17+n)),'linear');
    q=[tm;wm;pm;wm;0;tm;0;pm;0;pe;0;pe;ddot;1;tm;wm;0;0;eta;1;I;cr;kd;ddot];
elseif caseNumber==2
    wr=c.final*w; tr=T/(c.final*c.etaDrive);
    if power<0, tr=T*c.etaDrive/c.final; end
    p(13)=3;
    q=ecvt_operating_point(tr,wr,p);
    assert(abs(q(2)-wr)<1e-8,'EV:RouteSpeedLimit','e-CVT cannot follow the imposed route speed.');
elseif caseNumber==3
    % Check preview alignment; a changed route must be prepared again.
    assert(abs(T-plan.torque(k))<1e-6 && abs(w-plan.wheelSpeed(k))<1e-8, ...
        'EV:RouteMismatch','Measured route demand differs from the prepared preview.');
    assert(plan.mechanicalFeasible(k),'EV:UnservedTraction','The prescribed route is mechanically infeasible.');
    q=plan.bestPoint(:,k);
    if power<0
        charge=route_regen_policy(soc,k,plan,c);
        targetPower=-(p(14)+p(15)*charge)*charge;
        scale=0;
        if q(12)<0, scale=min(1,max(0,targetPower/q(12))); end
        q=scalePoint(q,scale,p);
    end
else
    error('EV:RouteCase','Route case must be 0, 1, 2 or 3.');
end
% Physical full-charge protection only; there is NO 20% path clamp.
if power<0 && q(21)<0 && soc-q(21)*c.dt/(3600*p(16))>1
    charge=max(0,(1-soc)*3600*p(16)/c.dt);
    desired=-(p(14)+p(15)*charge)*charge;
    if caseNumber==1
        scale=min(1,max(0,desired/q(12)));
        q([1 3 6 8 10 12])=q([1 3 6 8 10 12])*scale;
        q(21)=-charge; q(22)=charge/p(16); q(24)=route_aging_rate(charge,p);
        q(13)=q(24);
        n=p(17); q(23)=interp1(p(18:17+n),p(18+n:17+2*n), ...
            min(q(22),p(17+n)),'linear');
    else
        q=scalePoint(q,min(1,max(0,desired/q(12))),p);
    end
end
wheelMotor=q(3)*c.etaDrive;
if q(3)<0, wheelMotor=q(3)/c.etaDrive; end
friction=0; fraction=0;
if power<0
    friction=max(0,wheelMotor-power);
    fraction=min(1,max(0,wheelMotor/power));
else
    assert(abs(wheelMotor-power)<1e-5, ...
        'EV:UnservedTraction','Installed powertrain cannot deliver the required traction.');
end
gearLoss=q(3)-wheelMotor; motorLoss=q(12)-q(3);
assert(gearLoss>=-1e-6 && motorLoss>=-1e-6);
residual=q(12)-power-friction-gearLoss-motorLoss;
terminalFeasible=plan.reachable(k) && soc>=plan.required(k)-1e-10;
% This flag is intentionally predictive-only. Direct and Local retain their
% original unconstrained SOC behavior; predictive control reports a failure
% if its current or next Euler state cannot meet the pathwise reserve.
nextSOC=soc-q(21)*c.dt/(3600*p(16));
if caseNumber==3
    minAllowed= c.predictiveMin;
    if k>=numel(c.time), minAllowed=c.terminalSOC; end
    routeEnergyInfeasible=(soc<minAllowed-1e-10) || (nextSOC<minAllowed-1e-10);
else
    routeEnergyInfeasible=false;
end
y=[q;fraction;friction;gearLoss;motorLoss;plan.progress(k); ...
    plan.remainingDistance(k);plan.required(k);double(terminalFeasible); ...
    residual;max(-q(12),0);p(15)*q(21)^2;caseNumber;double(abs(residual)<1e-5); ...
    plan.noRegenRequired(k);double(routeEnergyInfeasible)];
end

function q = scalePoint(q,f,p)
tr=q(1)*f; wr=q(2); wc=q(5); a=p(1)/p(2); b=1+a;
d=ecvt_candidate_metrics(tr,wr,wc,p);
q([1 3 6 7])=[tr;tr*wr;-a*tr;b*tr];
q(8:12)=d(4:8); q(13)=d(3); q(19:24)=d(9:14);
q(17)=q(15)-tr;
end

function I = batteryCurrent(P,p)
disc=p(14)^2-4*p(15)*P;
assert(disc>=0,'Battery power exceeds the existing real-current limit.');
I=2*P/(p(14)+sqrt(disc));
end
