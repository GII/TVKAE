# __TVKAE__
Repository containing the MATLAB open source code for the Time-Varying Kalman Attitude Estimator (TVKAE).

Authors: Alvaro Deibe, Jose A. Anton Nacimiento, Jesus Cardenal and Fernando López Peña.
Universidade da Coruña, Spain; 

Last modified: 20/03/2023

## Cite

When using information from this repository, please cite the following works:

1. <a id="referencia1"></a>A. Deibe, J.A. Anton Nacimiento, J. Cardenal, and F. López Peña, "A Kalman Filter for Nonlinear Attitude Estimation Using Time Variable Matrices and Quaternions," Sensors, vol. 20, no. 23, p. 6731, Nov. 2020, https://doi.org/10.3390/s20236731
2. <a id="referencia2"></a>A. Deibe, J.A. Anton Nacimiento, J. Cardenal, and F. López Peña, "A Time–Varying Kalman Filter for Low–Acceleration Attitude Estimation", Measurement, 2023, https://doi.org/10.1016/j.measurement.2023.112729.

## This repository is organized as follows:

- TVKAE.m: MATLAB code for the TVKAE.
- errEvalTVKAE.m: Matlab code for evaluating the attitude estimation error according to Caruso et al. methodology.
- verification.m: Matlab code for verifying the TVKAE error with beta=0 (TV0) as it has been published in table 2 in [[2]](#referencia2).

- /Data: This folder is empty. In order for the algorithm to work, it should contain the data (fast_v5.mat, medium_v5.mat and slow_v5.mat) from Caruso et al. repositories, at:

  - GitHub: "https://github.com/marcocaruso/mimu_optical_dataset_caruso_sassari" or at
  - IEEEDataPort: "https://ieee-dataport.org/documents/mimuopticalsassaridataset".

It is also necessary that the main folder contains the function "correctQuat.m", available at the Caruso et al. sensor_fusion_algorithm_codes repository:

  - GitHub: "https://github.com/marcocaruso/sensor_fusion_algorithm_codes"
