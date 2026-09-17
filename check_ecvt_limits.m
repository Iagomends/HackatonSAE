% CHECK_ECVT_LIMITS Exercise infeasible torque/speed and constrained optimum.
% Shared operating-point solver used by the Simulink variant.
initialize_model;
requests = [1e5 0; 1e5 500; -1e5 500; 0 15000; 100 15000; ...
    -100 -15000; 300 1000; -300 -1000; 0 0];
for k = 1:size(requests,1)
    q = ecvt_operating_point(requests(k,1),requests(k,2),ecvt_parameters);
    a = N_s/N_r; b = 1+a;
    assert(abs(a*q(4)+q(2)-b*q(5))<1e-8);
    assert(abs(q(8)+q(9)-q(3))<1e-6);
    assert(abs(q(6))<=ecvt_motor1_Tmax+1e-7 && abs(q(7))<=ecvt_motor2_Tmax+1e-7);
    assert(abs(q(4))<=motor1_wmax+1e-7 && abs(q(5))<=motor2_wmax+1e-7);
    assert(abs(q(8))<=ecvt_motor1_Pmax+1e-6 && abs(q(9))<=ecvt_motor2_Pmax+1e-6);
    if k<=6, assert(q(14)==0,'Impossible requests must be flagged.'); end
    % Independent dense feasible-speed sweep checks constrained speed objective.
    sweep = linspace(-motor2_wmax,motor2_wmax,40001);
    sun = (b*sweep-q(2))/a;
    valid = abs(sun)<=motor1_wmax & abs(q(6)*sun)<=ecvt_motor1_Pmax & ...
        abs(q(7)*sweep)<=ecvt_motor2_Pmax;
    objectives = (sun(valid)/motor1_wmax).^2+(sweep(valid)/motor2_wmax).^2;
    if ~isempty(objectives), assert(q(13)<=min(objectives)+1e-10); end
    % The electrical adapter must reproduce the selected point, even at limits.
    assert(abs(ecvt_electrical_power([q(1:2);ecvt_parameters(:)])-q(12))<1e-5);
end
fprintf('PASS: %d edge operating points and independent objective sweeps.\n',size(requests,1));
