function [charge,value,feasible] = route_regen_policy(soc,k,plan,c)
%ROUTE_REGEN_POLICY Bellman feedback from actual SOC and current route index.
% The exact minimum charging current needed to stay inside the backward
% reachable set is always a candidate, independent of both numerical grids.
soc=soc(:); p=c.p; step=c.dt/(3600*p(16)); N=numel(c.time);
if k>=N
    charge=zeros(size(soc)); value=charge;
    feasible=soc>=c.terminalSOC-1e-12; return
end
% The terminal reserve is a hard pathwise constraint for predictive control.
% Use the small numerical reserve before the final sample so forward Euler
% roundoff cannot create a reported SOC below the physical 20% floor.
minPathSOC=c.predictiveMin;
feasible=plan.reachable(k) & soc>=plan.required(k)-1e-12 & ...
    soc>=minPathSOC-1e-12 & soc<=1+1e-12;
if plan.maxCharge(k)<=0
    charge=zeros(size(soc)); next=soc-step*plan.traction(k);
    value=route_aging_rate(plan.traction(k),p)*c.dt+futureValue(next,k+1,plan);
    return
end
lo=max(0,max((plan.required(k+1)-soc)/step,(minPathSOC-soc)/step));
hi=min(plan.maxCharge(k),max(0,(1-soc)/step));
% No reward for excess terminal SOC: truncate actions once no future regen
% is needed. The preferred band is only a tie-break, not a hard path bound.
hi=min(hi,max(lo,max(0,(plan.noRegenRequired(k+1)-soc)/step)));
lo=min(lo,hi);
base=linspace(0,plan.maxCharge(k),c.regenNodes);
n=p(17); knots=p(16)*p(18:17+n);
actions=min(max([base(:)' knots(:)'],lo),hi);
actions=[actions lo hi];
next=soc+step*actions;
cost=route_aging_rate(actions,p)*c.dt+futureValue(next,k+1,plan);
[value,~]=min(cost,[],2);
preference=max(c.preferredMin-next,0)+max(next-c.preferredMax,0);
preference(cost~=value)=inf;
[~,index]=min(preference,[],2);
charge=actions(sub2ind(size(actions),(1:numel(soc))',index));
% If already outside the reachable set, maximize useful recovery and expose
% infeasibility. Never claim the terminal condition can then be guaranteed.
charge(~feasible)=min(plan.maxCharge(k),max(0,(1-soc(~feasible))/step));
end

function value = futureValue(soc,k,plan)
x=plan.socGrid(:,k); v=plan.value(:,k);
if x(end)-x(1)<1e-14
    value=v(end)*ones(size(soc));
else
    value=interp1(x,v,min(max(soc,x(1)),x(end)),'linear');
end
% Guard is separate from interpolated value, avoiding artificial feasibility.
value(soc<plan.required(k)-1e-10 | soc>1+1e-10)=inf;
end
