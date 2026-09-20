function validate_route_result(d,c,caseNumber)
%VALIDATE_ROUTE_RESULT Check actual Simulink integration and physical balances.
p=c.p; tol=1e-5;
assert(max(abs(d.battery_power-d.route_battery_power))<tol);
assert(max(abs(d.battery_current-d.route_predicted_current))<1e-8);
assert(max(abs(d.capacity_loss_rate-d.route_degradation_rate))<1e-17);
assert(max(abs(d.battery_power-d.wheel_power-d.friction_brake_power- ...
    d.drivetrain_loss_power-d.motor_loss_power))<tol);
assert(max(abs(p(14)*d.battery_current-d.battery_power-d.battery_loss_power))<tol);
assert(max(abs(diff(d.SOC)+diff(d.time_s).*d.battery_current(1:end-1)/(3600*p(16))))<1e-12);
assert(max(abs(diff(d.accumulated_degradation)- ...
    diff(d.time_s).*d.capacity_loss_rate(1:end-1)))<1e-15);
assert(all(d.regen_fraction>=0 & d.regen_fraction<=1));
assert(all(d.friction_brake_power>=-tol));
assert(max(abs(d.route_motor1_mechanical_power-d.route_motor1_torque.*d.route_motor1_speed))<tol);
assert(max(abs(d.route_motor2_mechanical_power-d.route_motor2_torque.*d.route_motor2_speed))<tol);
if caseNumber==1
    assert(max(abs(d.route_motor1_torque))<=c.Tmax+tol);
    assert(max(abs(d.route_motor1_mechanical_power))<=c.Pmax+tol);
    assert(max(abs(d.route_motor1_speed))<=c.wmax+tol);
else
    a=p(1)/p(2); b=1+a;
    assert(max(abs(a*d.route_motor1_speed+d.route_shaft_speed-b*d.route_motor2_speed))<tol);
    assert(max(abs(d.route_motor1_torque+a*d.route_shaft_torque))<tol);
    assert(max(abs(d.route_motor2_torque-b*d.route_shaft_torque))<tol);
    f1=ones(height(d),1); f2=f1;
    f1(d.route_motor1_mechanical_power<-tol)=p(11);
    f2(d.route_motor2_mechanical_power<-tol)=p(12);
    assert(all(abs(d.route_motor1_torque)<=p(3)*f1+tol));
    assert(all(abs(d.route_motor2_torque)<=p(4)*f2+tol));
    assert(all(abs(d.route_motor1_mechanical_power)<=p(5)*f1+tol));
    assert(all(abs(d.route_motor2_mechanical_power)<=p(6)*f2+tol));
    assert(max(abs(d.route_motor1_speed))<=p(7)+tol && max(abs(d.route_motor2_speed))<=p(8)+tol);
end
if caseNumber==3 && d.terminal_SOC_feasible(1)>0.5
    assert(all(d.terminal_SOC_feasible>0.5));
    assert(d.SOC(end)>=c.terminalSOC-1e-10,'Feasible terminal SOC guarantee failed.');
end
fprintf('PASS: case %d, %d samples: conservation, limits, aging and SOC.\n',caseNumber,height(d));
end
