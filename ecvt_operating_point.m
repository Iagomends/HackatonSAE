function y = ecvt_operating_point(requestedTorque, requestedSpeed, p)
%ECVT_OPERATING_POINT Quasi-static planetary, Sun=M1, Carrier=M2, Ring=output.
% Fixed output vector documented in README_eCVT.md. SI units.
% Parameter vector is defined only in initialize_model.m.
%#codegen
a = p(1)/p(2); b = 1+a;
% Kinematically achievable ring range with both motor speeds bounded.
speedLimit = a*p(7)+b*p(8);
wr = min(max(requestedSpeed,-speedLimit),speedLimit);
speedOK = abs(requestedSpeed)<=speedLimit;
% Maximize delivered torque magnitude if the requested torque is infeasible.
[ok, wc] = selectSpeed(requestedTorque,wr,p);
tr = requestedTorque;
if ~ok
    lo = 0; hi = 1;
    for n = 1:55
        mid = (lo+hi)/2;
        [candidateOK,~] = selectSpeed(mid*requestedTorque,wr,p);
        if candidateOK
            lo = mid;
        else
            hi = mid;
        end
    end
    tr = lo*requestedTorque;
    [~,wc] = selectSpeed(tr,wr,p);
end
ws = (b*wc-wr)/a;
t1 = -a*tr; t2 = b*tr;
pm1 = t1*ws; pm2 = t2*wc;
pe1 = electricPower(pm1,p(9),p(10));
pe2 = electricPower(pm2,p(9),p(10));
J = (ws/p(7))^2+(wc/p(8))^2;
% Equivalent ring mechanical interface. Actual machines are logged separately.
y = [tr;wr;tr*wr;ws;wc;t1;t2;pm1;pm2;pe1;pe2;pe1+pe2;J; ...
    double(ok && speedOK);requestedTorque;requestedSpeed; ...
    requestedTorque-tr;requestedSpeed-wr];
end

function [ok,wc] = selectSpeed(tr,wr,p)
% Solve a strictly convex scalar quadratic over a union of sign intervals.
% Each interval fixes motor power signs, hence regenerative envelope limits.
a = p(1)/p(2); b = 1+a;
t1 = -a*tr; t2 = b*tr;
free = b*wr*p(8)^2/(b^2*p(8)^2+a^2*p(7)^2);
ok = false; wc = 0; best = inf;
for sign1 = [-1 1]
    for sign2 = [-1 1]
        f1 = 1; f2 = 1;
        if t1*sign1<0, f1=p(11); end
        if t2*sign2<0, f2=p(12); end
        if abs(t1)>p(3)*f1 || abs(t2)>p(4)*f2, continue; end
        w1 = p(7); w2 = p(8);
        if abs(t1)>0, w1=min(w1,p(5)*f1/abs(t1)); end
        if abs(t2)>0, w2=min(w2,p(6)*f2/abs(t2)); end
        sLo = -w1; sHi = w1; cLo = -w2; cHi = w2;
        if sign1<0, sHi=0; else, sLo=0; end
        if sign2<0, cHi=0; else, cLo=0; end
        lower = max(cLo,(a*sLo+wr)/b);
        upper = min(cHi,(a*sHi+wr)/b);
        if lower<=upper
            candidate = min(max(free,lower),upper);
            sun = (b*candidate-wr)/a;
            objective = (sun/p(7))^2+(candidate/p(8))^2;
            if objective<best
                best=objective; wc=candidate; ok=true;
            end
        end
    end
end
end

function pe = electricPower(pm,etaMotor,etaRegen)
% Exactly the existing constant efficiency motor/inverter law.
if pm>=0, pe=pm/etaMotor; else, pe=pm*etaRegen; end
end
