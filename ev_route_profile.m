function [time,speed] = ev_route_profile(name)
%EV_ROUTE_PROFILE Named, reproducible prescribed scenarios; SI units.
switch string(name)
    case "original"
        time=[0 5 25 55 70 80 100 130 150 160]';
        speed=[0 0 15 15 0 0 25 25 0 0]';
    case "motorway_service_area"
        % Synthetic 23-minute mixed urban/highway route.  Linear segments
        % are deliberately ramp-limited (roughly <= 1 m/s^2) and include
        % repeated stops, urban braking, highway speed reductions, and a
        % final urban section with several additional braking events.
        time=[0 10 28 45 60 72 92 112 128 145 165 190 210 228 248 ...
              270 292 310 330 360 390 520 545 570 600 760 785 810 ...
              840 1020 1045 1070 1100 1190 1210 1230 1250 1270 1290 ...
              1320 1340 1360 1380]';
        speed=[0 0 12 12 0 0 16 16 5 5 22 22 0 0 18 18 0 0 25 25 ...
               38.8 38.8 24 24 38.8 38.8 28 28 38.8 38.8 20 20 38.8 38.8 18 18 0 0 ...
               15 15 8 8 0]';
    otherwise
        error('EV:RouteProfile','Unknown route profile: %s',string(name));
end
end
