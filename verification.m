% Evaluates the TVKAE attitude estimation error in nine different
% experimental scenarios. The computed errors are in Deibe et al., "A
% Time–Varying Kalman Filter for Low–Acceleration Attitude Estimation",
% submitted to IEEE TIM, 2022.
%
% For a complete explanation of the TVKAE algoritm, see Deibe et al., "A
% Kalman filter for nonlinear attitude estimation using time 
% matrices and quaternions", Sensors, vol. 20, no. 23, Nov. 2020,
% https://doi.org/10.3390/s20236731, and Deibe et al. "A Time–Varying
% Kalman Filter for Low–Acceleration Attitude Estimation", submitted to
% IEEE TIM, 2022.
%
% For a detailed description of the experimental scenarios, see Caruso et
% al., "Analysis of the Accuracy of Ten Algorithms for Orientation
% Estimation Using Inertial and Magnetic Sensing under Optimal Conditions:
% One Size Does Not Fit All", Sensors, vol. 21, no. 7, p. 2543, Apr. 2021,
% https://doi.org/10.3390/s21072543, and Caruso et al., "Orientation
% Estimation Through Magneto-Inertial Sensor Fusion: A Heuristic Approach
% for Suboptimal Parameters Tuning", IEEE Sensors Journal, vol. 21, no. 3,
% pp. 3408-3419, 2021, https://doi.org/10.1109/JSEN.2020.3024806.

% IMU XSens
errEvalTVKAE([5.1952e-03 6.1652e-07],1,1,0,1); % Slow
errEvalTVKAE([1.4296e-02 8.9217e-07],1,2,0,1); % Medium
errEvalTVKAE([1.6097e-03 2.7897e-08],1,3,0,1); % Fast

% IMU APDM
errEvalTVKAE([3.2915e-01 1.0582e-04],2,1,0,1); % Slow
errEvalTVKAE([3.0040e-01 8.5369e-05],2,2,0,1); % Medium
errEvalTVKAE([4.1735e-04 1.1991e-07],2,3,0,1); % Fast

% IMU Shimmer
errEvalTVKAE([8.1105e-02 1.4415e-05],3,1,0,1); % Slow
errEvalTVKAE([7.8705e-11 9.6995e-08],3,2,0,1); % Medium
errEvalTVKAE([6.8006e-02 1.3946e-05],3,3,0,1); % Fast
