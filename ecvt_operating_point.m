function [y, diagnostics] = ecvt_operating_point(requestedTorque, requestedSpeed, p)
%ECVT_OPERATING_POINT Exact quasi-static planetary operating-point selection.
% Sun=M1, carrier=M2, ring=output. Strategy p(13): speed, |I|, or Ddot.
% First 18 outputs preserve the mechanical interface; see README_eCVT.md.
assert(numel(p)>=21 && any(p(13)==[1 2 3]),'Invalid e-CVT parameters.');
a = p(1)/p(2); b = 1+a;
speedLimit = a*p(7)+b*p(8);
wr = min(max(requestedSpeed,-speedLimit),speedLimit);
speedOK = abs(requestedSpeed)<=speedLimit;
[ok, wc] = selectSpeed(requestedTorque,wr,p);
tr = requestedTorque;
if ~ok
    lo = 0; hi = 1;
    for n = 1:55
        mid = (lo+hi)/2;
        [candidateOK,~] = selectSpeed(mid*requestedTorque,wr,p);
        if candidateOK, lo=mid; else, hi=mid; end
    end
    tr = lo*requestedTorque;
    [deliveredOK,wc] = selectSpeed(tr,wr,p);
    assert(deliveredOK,'No feasible delivered operating point.');
end
ws = (b*wc-wr)/a;
t1 = -a*tr; t2 = b*tr;
diagnostics = ecvt_candidate_metrics(tr,wr,wc,p);
J = diagnostics(p(13));
y = [tr;wr;tr*wr;ws;wc;t1;t2;diagnostics(4:8);J; ...
    double(ok && speedOK);requestedTorque;requestedSpeed; ...
    requestedTorque-tr;requestedSpeed-wr;diagnostics(9:14)];
end

function [ok,wc] = selectSpeed(tr,wr,p)
% Power is affine on each power-sign interval with the existing efficiencies.
% |I(P)| and nonnegative nondecreasing k(C)*|I|/3600 minimize at an endpoint
% or P=0. Lookup knots also delimit flat aging regions for the speed tie-break.
a=p(1)/p(2); b=1+a; t1=-a*tr; t2=b*tr;
free=b*wr*p(8)^2/(b^2*p(8)^2+a^2*p(7)^2);
ok=false; wc=0; best=inf; bestSpeed=inf;
for sign1=[-1 1]
    for sign2=[-1 1]
        f1=1; f2=1;
        if t1*sign1<0, f1=p(11); end
        if t2*sign2<0, f2=p(12); end
        if abs(t1)>p(3)*f1 || abs(t2)>p(4)*f2, continue; end
        w1=p(7); w2=p(8);
        if abs(t1)>0, w1=min(w1,p(5)*f1/abs(t1)); end
        if abs(t2)>0, w2=min(w2,p(6)*f2/abs(t2)); end
        sLo=-w1; sHi=w1; cLo=-w2; cHi=w2;
        if sign1<0, sHi=0; else, sLo=0; end
        if sign2<0, cHi=0; else, cLo=0; end
        lower=max(cLo,(a*sLo+wr)/b);
        upper=min(cHi,(a*sHi+wr)/b);
        if lower>upper, continue; end
        % Electrical power = slope*wc + offset on this closed interval.
        g1=1/p(9); g2=1/p(9);
        if t1*sign1<0, g1=p(10); end
        if t2*sign2<0, g2=p(10); end
        slope=g1*t1*b/a+g2*t2; offset=-g1*t1*wr/a;
        if p(15)>0
            maxPower=p(14)^2/(4*p(15));
            if slope>0, upper=min(upper,(maxPower-offset)/slope);
            elseif slope<0, lower=max(lower,(maxPower-offset)/slope);
            elseif offset>maxPower, continue;
            end
        end
        if lower>upper, continue; end
        candidates=[lower upper min(max(free,lower),upper)];
        if slope~=0
            n=p(17); currents=p(16)*p(18:17+n);
            currents=[0 currents(:)' -currents(:)'];
            powers=(p(14)-p(15)*currents).*currents;
            roots=(powers-offset)/slope;
            candidates=[candidates roots(roots>=lower & roots<=upper)]; %#ok<AGROW>
        end
        for candidate=candidates
            d=ecvt_candidate_metrics(tr,wr,candidate,p);
            objective=d(p(13));
            if ~isfinite(objective), continue; end
            tol=64*eps(max(abs(objective),abs(best)));
            if ~ok || objective<best-tol || ...
                    (abs(objective-best)<=tol && d(1)<bestSpeed)
                best=objective; bestSpeed=d(1); wc=candidate; ok=true;
            end
        end
    end
end
end
