%% build_tvc_dual_axis_model.m
%
% Programmatically builds a Simulink model that mirrors the PID gains,
% gimbal limits, and actuator behavior from tvc_pid_controller.ino, so
% you can simulate closed-loop pitch/yaw response BEFORE flashing
% anything to the Teensy.
%
% WHAT THIS BUILDS:
%   Two identical, independent control loops (Pitch and Yaw), each:
%       Setpoint(0) -> Sum -> PID -> Saturation -> Servo lag (1st order)
%       -> deg2rad -> Torque gain (F*L) -> 1/I -> Integrator (rate)
%       -> Integrator (angle, WITH an initial-condition tip so you can
%          watch it recover) -> rad2deg -> feedback to Sum & Scope
%
% HOW TO USE:
%   1. Edit the PHYSICAL PARAMETERS section below to match YOUR rocket
%      (thrust, moment arm, moment of inertia). The placeholders will
%      run without erroring but won't represent your actual vehicle.
%   2. Run this script in MATLAB. It creates and opens
%      "TVC_DualAxis_Test.slx" in your current folder.
%   3. Hit Run in Simulink, then open the two Scopes to see pitch/roll
%      angle settle back to 0 and the commanded gimbal deflection.
%   4. Adjust Kp/Ki/Kd at the top, re-run, repeat -- this is much faster
%      and safer than tuning on a bench rig or, worse, in flight.
%
% VERSION NOTE: Built against the standard Simulink PID Controller block
% path ('simulink/Continuous/PID Controller'). If your MATLAB version
% errors on that add_block line, open the Simulink Library Browser,
% search "PID Controller", right-click > "Open the library containing
% this block" and use the path it shows you instead.

clear; clc;

%% ------------------------------------------------------------------
%  CONTROL GAINS -- copied directly from tvc_pid_controller.ino
%  ------------------------------------------------------------------
gains.pitch.Kp = 2.5;   gains.pitch.Ki = 0.15;  gains.pitch.Kd = 0.6;
gains.yaw.Kp   = 2.5;   gains.yaw.Ki   = 0.15;  gains.yaw.Kd   = 0.6;

MAX_GIMBAL_DEG = 12;      % matches MAX_GIMBAL_DEG in the .ino
SERVO_TAU      = 0.08;    % [s] approx SG90 first-order response time
                          % constant -- measure yours if you want accuracy
                          % (rough rule of thumb: SG90 datasheet speed is
                          % ~0.1s/60deg unloaded; loaded under a gimbal
                          % arm it'll be slower -- treat this as a guess)

%% ------------------------------------------------------------------
%  PHYSICAL PARAMETERS -- *** EDIT THESE FOR YOUR ROCKET ***
%  ------------------------------------------------------------------
THRUST_N        = 8;     % [N] average motor thrust during burn -- placeholder
MOMENT_ARM_M    = 0.15;   % [m] distance from gimbal pivot to vehicle CG -- placeholder
INERTIA_KGM2    = 0.05;   % [kg*m^2] rotational moment of inertia about
                          % pitch/yaw axis through CG -- placeholder.
                          % Estimate via CAD mass properties, a bifilar
                          % pendulum test, or a swing-test rig.
INITIAL_TIP_DEG = 10;     % [deg] initial angle offset to simulate a gust
                          % or launch-rail disturbance, so you can watch
                          % the loop recover

%% ------------------------------------------------------------------
%  BUILD MODEL
%  ------------------------------------------------------------------
modelName = 'TVC_DualAxis_Test';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);
set_param(modelName, 'Solver', 'ode45', 'StopTime', '19.8');

axes = {'Pitch', 'Yaw'};
yOffsets = [0, 250]; % vertical spacing so the two loops don't overlap

for i = 1:numel(axes)
    axisName = axes{i};
    yOff = yOffsets(i);
    g = gains.(lower(axisName));
    build_axis(modelName, axisName, yOff, g.Kp, g.Ki, g.Kd, ...
        MAX_GIMBAL_DEG, SERVO_TAU, THRUST_N, MOMENT_ARM_M, ...
        INERTIA_KGM2, INITIAL_TIP_DEG);
end

Simulink.BlockDiagram.arrangeSystem(modelName);
save_system(modelName);

fprintf('Model "%s.slx" built and saved in %s\n', modelName, pwd);
fprintf('Open the Pitch and Yaw scopes and hit Run to simulate.\n');


%% ====================================================================
%  HELPER FUNCTION -- builds one axis's control loop
%  ====================================================================
function build_axis(modelName, axisName, yOff, Kp, Ki, Kd, ...
        maxGimbalDeg, servoTau, thrustN, momentArmM, inertiaKgm2, ...
        initialTipDeg)

    p = @(name) [modelName '/' axisName '_' name];
    xw = 60; xh = 40; xstep = 110; x0 = 30;

    pos = @(col, row) [x0 + col*xstep, yOff + row*80, ...
                        x0 + col*xstep + xw, yOff + row*80 + xh];

    % --- Setpoint (always 0: hold vertical) ---
    add_block('simulink/Sources/Constant', p('Setpoint'), ...
        'Value', '0', 'Position', pos(0, 0));

    % --- Sum: error = setpoint - feedback ---
    add_block('simulink/Math Operations/Sum', p('ErrorSum'), ...
        'Inputs', '+-', 'Position', pos(1, 0));

    % --- PID Controller ---
    add_block('simulink/Continuous/PID Controller', p('PID'), ...
        'P', num2str(Kp), 'I', num2str(Ki), 'D', num2str(Kd), ...
        'LimitOutput', 'on', ...
        'UpperSaturationLimit', num2str(maxGimbalDeg), ...
        'LowerSaturationLimit', num2str(-maxGimbalDeg), ...
        'AntiWindupMode', 'back-calculation', ...
        'Position', pos(2, 0));
    % NOTE: 'back-calculation' anti-windup is not identical to the
    % conditional-integration scheme in the .ino PID class, but it
    % serves the same purpose (stop the integrator from winding up
    % while the output is saturated) and is close enough for tuning
    % purposes. If you want an exact match, replace this block with a
    % custom MATLAB Function block implementing the same logic as the
    % PID class in the .ino file.

    % --- Servo actuator lag (first-order approx of SG90 response) ---
    add_block('simulink/Continuous/Transfer Fcn', p('ServoLag'), ...
        'Numerator', '[1]', ...
        'Denominator', ['[' num2str(servoTau) ' 1]'], ...
        'Position', pos(3, 0));

    % --- deg to rad ---
    add_block('simulink/Math Operations/Gain', p('Deg2Rad'), ...
        'Gain', 'pi/180', 'Position', pos(4, 0));

    % --- Torque = Thrust * MomentArm * sin(angle) ~ small-angle approx ---
    torqueGain = thrustN * momentArmM;
    add_block('simulink/Math Operations/Gain', p('TorqueGain'), ...
        'Gain', num2str(torqueGain), 'Position', pos(5, 0));

    % --- Angular acceleration = Torque / I ---
    add_block('simulink/Math Operations/Gain', p('InvInertia'), ...
        'Gain', num2str(1/inertiaKgm2), 'Position', pos(6, 0));

    % --- Integrator 1: angular acceleration -> angular rate ---
    add_block('simulink/Continuous/Integrator', p('RateIntegrator'), ...
        'Position', pos(7, 0));

    % --- Integrator 2: angular rate -> angle (rad), with initial tip ---
    initialRad = num2str(initialTipDeg * pi / 180);
    add_block('simulink/Continuous/Integrator', p('AngleIntegrator'), ...
        'InitialCondition', initialRad, 'Position', pos(8, 0));

    % --- rad to deg (for feedback + scope, human-readable units) ---
    add_block('simulink/Math Operations/Gain', p('Rad2Deg'), ...
        'Gain', '180/pi', 'Position', pos(9, 0));

    % --- Scope: vehicle angle + commanded gimbal deflection ---
    add_block('simulink/Sinks/Scope', p('Scope'), ...
        'Position', pos(9, 2), 'NumInputPorts', '2');

    % ---------------- WIRING ----------------
    add_line(modelName, [axisName '_Setpoint/1'], [axisName '_ErrorSum/1']);
    add_line(modelName, [axisName '_ErrorSum/1'], [axisName '_PID/1']);
    add_line(modelName, [axisName '_PID/1'], [axisName '_ServoLag/1']);
    add_line(modelName, [axisName '_ServoLag/1'], [axisName '_Deg2Rad/1']);
    add_line(modelName, [axisName '_Deg2Rad/1'], [axisName '_TorqueGain/1']);
    add_line(modelName, [axisName '_TorqueGain/1'], [axisName '_InvInertia/1']);
    add_line(modelName, [axisName '_InvInertia/1'], [axisName '_RateIntegrator/1']);
    add_line(modelName, [axisName '_RateIntegrator/1'], [axisName '_AngleIntegrator/1']);
    add_line(modelName, [axisName '_AngleIntegrator/1'], [axisName '_Rad2Deg/1']);

    % feedback: angle (deg) back into the error sum's second input
    add_line(modelName, [axisName '_Rad2Deg/1'], [axisName '_ErrorSum/2'], ...
        'autorouting', 'on');

    % scope inputs: vehicle angle (deg) and commanded gimbal (deg)
    add_line(modelName, [axisName '_Rad2Deg/1'], [axisName '_Scope/1'], ...
        'autorouting', 'on');
    add_line(modelName, [axisName '_PID/1'], [axisName '_Scope/2'], ...
        'autorouting', 'on');
end
