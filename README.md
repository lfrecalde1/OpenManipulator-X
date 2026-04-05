# OpenManipulator_Matlab

This repository includes:
- Low-level MATLAB control of OpenManipulator-X through the Dynamixel SDK.
- Symbolic rigid-body dynamics generation (`M`, `C`, `G`, kinematics, and friction terms).
- A system identification pipeline from experimental data.
- Validation scripts for the identified dynamics.

## Safety Notes (Read First)
- `system_ID.m` commands multi-harmonic trajectories. Use it carefully to avoid collisions or aggressive motion.
- Always verify robot clearance and joint limits before running hardware scripts.
- Start with conservative gains in current-control experiments.

## Environment Setup
### 1) MATLAB + Dynamixel SDK
1. Install MATLAB on Linux.
2. Download the Dynamixel SDK for MATLAB/Linux:
   - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/#repository
3. Configure SDK paths:
   - Preferred: edit and run [init_libraries.m](init_libraries.m)
   - Or follow: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/library_setup/matlab_linux/#matlab-linux
4. Build the native SDK library:
```bash
cd <your_dynamixel_sdk>/c/build/linux64
make clean
make
```
5. Set Linux serial permissions if needed:
```bash
sudo chmod a+rw /dev/ttyUSB0
```

### 2) Optional Tools for Identification
- CasADi is required by [Identification/estimate_system_parameters_casadi_robust_new.m](Identification/estimate_system_parameters_casadi_robust_new.m).
- Update the local CasADi path inside that script.

## Recommended Workflow for Control
1. Run the position controller:
   - [example_joint_control.m](example_joint_control.m)
2. Run the current controller:
   - [example_current_control.m](example_current_control.m)
   - Use with care; incorrect current commands can damage hardware.

## Recommended Workflow for Identification
1. Generate symbolic/numeric dynamics:
   - run [dynamics_manipulator_complete.m](dynamics_manipulator_complete.m)
2. Collect excitation data from the robot:
   - run [system_ID.m](system_ID.m)
3. Filter signals and estimate parameters:
   - run [Identification/Identification_filtered_v2.m](Identification/Identification_filtered_v2.m)
4. Validate reconstructed torques:
   - run [Identification/validate_identified_model.m](Identification/validate_identified_model.m)

If you find errors in the dynamics, or if you think you can generate a more accurate representation of the robotic manipulator, feel free to formulate your own dynamics and repeat the identification process.

## Dynamics Model Conventions
### State and Dynamics
- Joint state: `q`, `qd`, `qdd` in radians, rad/s, and rad/s².
- Manipulator model:
```text
M(q) qdd + C(q,qd) qd + G(q) + tau_friction = tau
```
- Viscous friction model:
```text
tau_visc = Fv * qd
```

### Parameter Vector `p`
Used by generated functions `M_fun`, `C_fun`, `G_fun`, `FK_fun`:
```text
p = [a0 d1 a1 d2 a2 ell_e m1 m2 m3 m4
     Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 Ixx4 Iyy4 Izz4 g]'
```

### Friction Parameter Vector `pf`
Current scripts/functions use:
```text
pf = [fv1 fv2 fv3 fv4]'
```

## Notes
- `current_to_torque` and `torque_to_current` are approximate mappings. You can improve them if needed.
- Reference: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/

Identified parameters are stored in `identification_result.mat`. You can use them to compute inertia, Coriolis, gravity, and friction terms:

```matlab
M = M_fun(q(:, i), p);
C = C_fun(q(:, i), q_dot(:, i), p);
G = G_fun(q(:, i), p);
```

Define `p` as:

```matlab
R = load('identification_result.mat');
x_opt_vec = R.x_opt_vec(:);

p = [R.p(1:6); ...
     x_opt_vec(1); x_opt_vec(2); x_opt_vec(3); x_opt_vec(4); ...
     x_opt_vec(5); x_opt_vec(6); x_opt_vec(7); ...
     x_opt_vec(8); x_opt_vec(9); x_opt_vec(10); ...
     x_opt_vec(11); x_opt_vec(12); x_opt_vec(13); ...
     x_opt_vec(14); x_opt_vec(15); x_opt_vec(16); ...
     R.id_info.g];

pf = [x_opt_vec(17); x_opt_vec(18); x_opt_vec(19); x_opt_vec(20)];
```

See [Identification/validate_identified_model.m](Identification/validate_identified_model.m) for details.