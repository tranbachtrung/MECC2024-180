# MECC 2024-180
Computational example code for the "Stochastic Predictive Control with Time-Joint State Constraint Violation Probability Objective for Drone Delivery of Medical Products" paper. 

Please read the 'main.m' file to get started with the computational example. To quickly run the code (bypassing the model compilation of Section 1, 2, 4, and 5), run 'main_quick_run.m'. 

The code requires using Casadi 3.6.5 in MATLAB, which can be freely obtained from https://web.casadi.org/get/. The required code is included in this folder of self-contained run.
If you use linux, please change the 'casadi_path' variable in 'main.m' and 'main_quick_run.m'. If you use mac, please download the right Casadi version from the website and set the right path to 'casadi_path'.  

The results of the computational example is saved in the folder 'paper_results' for ease of reference. Running this code online will save the results to 'results'. Note that if the model (Section 1, 2, 4, and 5) and the run settings (Secion 6) do not change, the figures in 'paper_results' and 'results' would be identical.

The code is available to run through Code Ocean: https://codeocean.com/capsule/1781107/tree/v1. 
