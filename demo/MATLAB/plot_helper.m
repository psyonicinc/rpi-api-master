%%
%       INSTRUCTIONS:
%   1. move thumb to desired orientation
%   2. make && ./demo.out for squeeze demo branch
%   3. copy file datalog.csv into local machine & open with matlab
%   4. for the import screen, select 'Numeric Matrix' and rename the
%       imported array to datas'. 
%   5. make sure datas is a Nx38 element array. If it doesn't work, select
%       'Delimited' in the import screen, make sure the column delimiter word
%       is 'Comma', and then ctl+a to select all the elements in the import
%       screen before hitting import selection. 
%   6. run this script
%
pressure_data = datas(:,1:20);
figure(1)
clf;
hold on
for i = 1:20
    plot(pressure_data(:,i)*100/2^16);
end
xlabel('sample');
ylabel('normalized pressure (0-100)');
title('Pressure');
hold off
qd = datas(:,21:26);
tau = datas(:,27:32);
q = datas(:,33:38);


% e_cost = zeros(size(tau));
e_cost = cumtrapz(abs(tau));

figure(3)
title('cost per finger');
clf
hold on
plot(e_cost(:,1))
plot(e_cost(:,2));
plot(e_cost(:,3));
plot(e_cost(:,4));
plot(e_cost(:,5));
plot(e_cost(:,6));
legend('index','middle','ring','pinky','flexor', 'rotator', 'location', 'Northwest');
hold off


figure(2)
clf
index = 1;
middle = 2;
ring = 3; %...
hold on
% for i = 1:4
%     plot(qd(:,i));
%     plot(tau(:,i));
%     plot(q(:,i));
% end
plot(qd(:,index)); % index
plot(tau(:,index)); % index
plot(q(:,index)); % index
plot(e_cost(:,index)/100);
xlabel('sample');
legend('setpoint','torque','position','cost', 'Location', 'Northwest');
title('Motion Params');
hold off

stiffness_guess = mean(max(e_cost(:,1:4)/100)); %% mean of the max of the integral of the torque for the index-pinky fingers
disp('Stiffness Measure:');
disp(stiffness_guess);

%% Pre load some data I already took

% data format:
%   N x 38 array
%   columns 1-20: pressure data (in api order)
%   columns 21-26: finger setpoints while hand squeezing object
%       column 21 -> index
%       column 22 -> middle
%       column 23 -> ring
%       column 24 -> pinky
%       column 25 -> thumb flexor
%       column 26 -> thumb rotator
%   columns 27-32: finger 'torque' (not in standard units)
%       (order is the same as for the finger setpoints)
%   columns 33-38: finger positions (in degrees from the open stall angle)
%       (order is the same as for the finger setpoints)

load('free-air.mat');
free_air_1 = get_stiffness_guess(datas(:,27:32));
disp('Free air:');
disp(free_air_1);

load('free-air-2.mat');
free_air_2 = get_stiffness_guess(datas(:,27:32));
% https://youtu.be/Vx1z5_IgAmA
disp('Free air 2:');
disp(free_air_2);

load('foam.mat');
foam = get_stiffness_guess(datas(:,27:32));
%https://youtu.be/-hdlyywdhCw
disp('foam:');
disp(foam);

load('chalk-bag.mat');
chalk_bag = get_stiffness_guess(datas(:,27:32));
%https://youtu.be/nY8VZZTUmPA
disp('chalk-bag:');
disp(chalk_bag);

load('glass-bottle.mat');
glass_bottle = get_stiffness_guess(datas(:,27:32));
%https://youtu.be/yxnaDpRh9J0
disp('glass-bottle:');
disp(glass_bottle);

