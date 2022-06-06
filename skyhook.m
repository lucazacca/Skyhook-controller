clc
close all
clear
% system parameters definition
m1 = 380; % sprung mass [kg]
m2 = 31; % unsprung mass [kg]
k1 = 29e03; % suspension stiffness [N/m]
k2 = 228e03; % wheel stiffness [N/m]
c_h = 6e03; % high damping [N*s/m]
c_l = 1.5e03; % low damping [N*s/m]
% road parameters definition
Gr = 2.56e-6; % type C road defined in ISO 8606 [cycle*s]
v_car = 30/3.6; % car speed [m/s]
% boolean signals and constants used in the simulation
semi_active = 1; % semi-active suspension active [boolean]
passive_damping = c_h; % passive damping high
user_enable = 1; % user button enabled (semi-active suspension active by user request) [boolean]
test_wheel = 0; % test wheel accelerometer error [boolean]
test_vehicle = 0; % test vehicle accelerometer error [boolean]
%% Suspension performance evaluation - testing for semi-active, passive with c_l and passive with c_h
t_stop = 5; %simulation time
v_car = 30/3.6; % change to simulate a different linear car velocity
Gr = 2.56e-6; % change to simulate a different road roughness
figure(1)
f_vehicle_disp = subplot(2,2, 1:2); % Vehicle displacement
f_vehicle_acc = subplot(223); % Vehicle acceleration
f_road = subplot(224); % Road displacement

semi_active = 1;
sim('sh_harness.slx')
% semi-active suspension plot section
rms_acc(1) = rms(Vehicle_acc_w.Data);
rms_vehicle_disp(1) = rms(Vehicle_disp_m.Data);
rms_road_disp(1) = rms(Road_disp_m.Data);
subplot(f_vehicle_disp)
plot(Vehicle_disp_m.Time, Vehicle_disp_m.Data, 'k')
subplot(f_vehicle_acc)
plot(Vehicle_acc_w.Time, Vehicle_acc_w.Data, 'k')
subplot(f_road)
plot(Road_disp_m.Time, Road_disp_m.Data, 'k')

semi_active = 0;
passive_damping = c_l;
sim('sh_harness.slx')
% passive low damping plot section
rms_acc(2) = rms(Vehicle_acc_w.Data);
rms_vehicle_disp(2) = rms(Vehicle_disp_m.Data);
rms_road_disp(2) = rms(Road_disp_m.Data);
subplot(f_vehicle_disp)
hold on
plot(Vehicle_disp_m.Time, Vehicle_disp_m.Data, 'r')
subplot(f_vehicle_acc)
hold on
plot(Vehicle_acc_w.Time, Vehicle_acc_w.Data, 'r')

passive_damping = c_h;
sim('sh_harness.slx')
% passive high damping plot section
rms_acc(3) = rms(Vehicle_acc_w.Data);
rms_vehicle_disp(3) = rms(Vehicle_disp_m.Data);
rms_road_disp(3) = rms(Road_disp_m.Data);
subplot(f_vehicle_disp)
hold on
plot(Vehicle_disp_m.Time, Vehicle_disp_m.Data, 'Color', [0.4660 0.6740 0.1880])
subplot(f_vehicle_acc)
hold on
plot(Vehicle_acc_w.Time, Vehicle_acc_w.Data, 'Color', [0.4660 0.6740 0.1880])

% visual aids section
subplot(f_vehicle_disp)
title("Vehicle displacement")
xlabel("Time [s]")
ylabel("Displacement [m]")
l_semiactive = sprintf("Semi-active (RMS = %.3f)", rms_vehicle_disp(1));
l_passive_l = sprintf("Passive - c=c_{l} (RMS = %.3f)", rms_vehicle_disp(2));
l_passive_h = sprintf("Passive - c=c_{h} (RMS = %.3f)", rms_vehicle_disp(3));
legend(l_semiactive, l_passive_l, l_passive_h, Location="best")

subplot(f_vehicle_acc)
title("Vehicle acceleration")
xlabel("Time [s]")
ylabel("Acceleration [m/s^2]")
l_semiactive = sprintf("Semi-active (RMS = %.3f)", rms_acc(1));
l_passive_l = sprintf("Passive - c=c_{l} (RMS = %.3f)", rms_acc(2));
l_passive_h = sprintf("Passive - c=c_{h} (RMS = %.3f)", rms_acc(3));
legend(l_semiactive, l_passive_l, l_passive_h, Location="best")

subplot(f_road)
title("Road profile")
xlabel("Time [s]")
ylabel("Displacement [m]")

%% Error testing
t_stop = 50e-3;
test_vehicle = 1; % testing vehicle sensor
sim("sh_harness")
figure(2)
subplot(2, 2, 1:2)
plot(Vehicle_acc.Time, Vehicle_acc.Data)
title("Vehicle acceleration")
ylabel("Acceleration [m/s^2]")
xlabel("Time [s]")
axis([0 t_stop 0 1.1])
grid on
hold on
subplot(2,2,3:4)
plot(Error.Time, Error.Data)
title("Error signal")
ylabel("Error [-]")
xlabel("Time [s]")
axis([0 t_stop 0 2.2])
grid on

test_vehicle = 0; 
test_wheel = 1; % testing wheel sensor
sim("sh_harness")
figure(3)
subplot(2, 2, 1:2)
plot(Wheel_acc.Time, Wheel_acc.Data)
title("Wheel acceleration")
ylabel("Acceleration [m/s^2]")
xlabel("Time [s]")
axis([0 t_stop 0 1.1])
grid on
hold on
subplot(2,2,3:4)
plot(Error.Time, Error.Data)
title("Error signal")
ylabel("Error [-]")
xlabel("Time [s]")
axis([0 t_stop 0 1.1])
grid on