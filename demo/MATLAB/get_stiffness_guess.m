% output: heurisitc stiffness estimate
% input: Nx6 torque data, from the hand
function stiffness_est = get_stiffness_guess(tau)
e_cost = cumtrapz(abs(tau));
stiffness_est = mean(max(e_cost(:,1:4)/100)); %% mean of the max of the integral of the torque for the index-pinky fingers
end