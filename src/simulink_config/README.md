# V2 research model

Open `src/V2.slx` in MATLAB/Simulink (built with R2024b). V1 remains the
reference model and is never edited by the builder. V2 is an intentional new
controller architecture, not a claim of numerical equivalence to V1.

![V2 top-level architecture](V2_top_level.png)

```matlab
addpath('src/simulink_config');
open_system('src/V2.slx');
out = sim('V2');
out.logsout.get('B_d_hat').Values
% Regenerate the diagram and embedded MATLAB Function code from source:
build_v2;
% Compile, hover, switches, payload, equations, and noise reproducibility:
test_v2;
```

The upper lane is position → velocity → body force. The lower lane is attitude
→ body rate PID + DOB → body torque. A separate CoM estimator feeds the tall
allocator. Separate propeller and servo dynamics drive the rigid-body plant;
the measurement layer supplies one bottom feedback bus. Servo angles also
return explicitly to allocation. Monitoring has no control outputs. Named
signals are logged to `out.logsout`, including internal PID/nominal torque and
raw/filtered CoM. The state buses are virtual buses with explicit named fields:
`TrueStateBus` includes `W_p`, `W_v`, `euler`, `B_omega`, `B_F_actual`, and
`B_tau_actual`; `MeasuredStateBus` includes the four feedback state vectors.

## Frames and V1 migration

V1 has **positive-down gravity** `[0;0;9.80665]` and negative-Z rotor thrust.
V2 explicitly converts V1 FRD/body and down-positive world coordinates using
`S = diag([1 -1 -1])`. V2 uses **world Z-up and body FLU**. Both coordinate
systems are right-handed. Body-to-world rotation is
`R_WB = Rz(yaw)*Ry(pitch)*Rx(roll)`; body force demand is `R_WB'*W_F_cmd`.
Euler commands and measurements are `[roll;pitch;yaw]` in radians. Position
is the total CoM position. Rigid-body integration retains V1 Euler kinematics;
pitch at ±π/2 is singular and explicitly rejected. Attitude *error* uses the
shortest rotation vector of `R_WB'*R_ref`, including a stable π-angle branch.

Motor/servo indices and positive servo angle sense are unchanged:

| Index | FLU rotor XY / (l/√2) | FLU tilt XY / (sin θ/√2) | Reaction sign relative to thrust |
|---|---|---|---|
| 1 | (+1,+1) | (+1,−1) | −1 |
| 2 | (−1,+1) | (+1,+1) | +1 |
| 3 | (−1,−1) | (−1,+1) | −1 |
| 4 | (+1,−1) | (−1,−1) | +1 |

All rotors have FLU Z = −0.015 m; arm length is 0.4 m. These come from V1's
`P1`–`P4` moment arms and force-vector subsystems. In particular the C++ model's
rotor index/spin table is **not** copied over V1's table. C++ allocation *logic*
is used with the converted V1 geometry. V1's extra duplicate moment-arm branch
is diagnostic; it does not contribute to its plant torque output.

V1 physical defaults retained: nominal/true mass 6 kg, nominal inertia
`diag([.2 .2 .2])`, true unloaded inertia `diag([.3 .3 .4])`, unloaded FLU CoM
`[.01 .005 .001]`, motor lag 0.01 s, motor effectiveness 0.8, command thrust
limits [0,200] N, servo lag 1/30 s, and servo limits ±1 rad. Initial actuator
outputs are zero. Neither V1 actuator path has a transport delay; configurable
zero-delay blocks make later delay experiments explicit. Motor saturation is
before lag/effectiveness, as in V1; servo saturation is both before and after
lag. No separate V1 thrust coefficient `b` or `k` is present: only their ratio.

V1's delayed Y sine is available as `EXP.command.position_mode = 1`: amplitude
−10 m and frequency 0.5 rad/s after 10 s, with absolute-time sine phase as in
V1. The default is a constant [0,0,1] m hover command. The three independently
switched attitude sines retain their V1 amplitudes/frequencies after FLU
conversion. The V1 time-dependent estimator switch is replaced by the requested
explicit controller modes; it is not an attitude/position command source.

## Parameters and initialization

| File | Responsibility |
|---|---|
| `CTRL.m` | PID gains, modes, controller timing, limits, nominal physics, allocator, DOB, MOCE |
| `PLANT.m` | True unloaded vehicle, geometry, propellers, servos, gravity |
| `SENSOR.m` | Per-sensor noise enable/std, bias, common measurement delay |
| `EXP.m` | Command trajectories, payload, initial state, random seed, run duration/physics step |
| `init_v2.m` | Fresh parameter construction, overrides, derived total mass/CoM/inertia |

The parameter files are simple functions returning structures. Model open and
simulation initialization reload them into the **model workspace**. Existing
base-workspace variables cannot silently override these settings. Editing
parameter files takes effect on the next update/simulation; editing a structure
in the base workspace does not. `init_v2()` returns all defaults for inspection.
For scripted experiments use an explicit model-workspace `V2_OVERRIDES` struct:

```matlab
load_system('src/V2.slx');
w = get_param('V2','ModelWorkspace');
overrides = struct();
overrides.EXP.payload.mass = 0.5;
overrides.EXP.payload.position = [0.15 0.08 0.05];
overrides.CTRL.moce.enable = true;
assignin(w,'V2_OVERRIDES',overrides);
out = sim('V2');
assignin(w,'V2_OVERRIDES',struct()); % return to file defaults
```

Payload position and unloaded vehicle CoM use the same fixed body geometry
origin. `PLANT.vehicle.inertia` is about the unloaded vehicle CoM. With total
mass M and derived CoM c, initialization adds the parallel-axis contributions
`m_v*(||c_v-c||² I - (c_v-c)*(c_v-c)')` and
`m_p*(||r_p-c||² I - (r_p-c)*(r_p-c)')`. The payload is a point mass. Only the
base vehicle CoM is editable; the combined true CoM is always derived. The
nominal controller model remains independent of payload/true model changes.

## Controllers and observers

All PID stages use the C++ `Pid` conditional integration, integral-contribution
limits, and filtered measurement-rate D term. The position stage's two-port
interface obtains velocity by sampled position differences. Position reference
feed-forward uses a first-order filtered difference and is exactly zero when
disabled. Attitude D uses measured body rate. Controllers run at 400 Hz;
continuous rigid-body and actuator states use RK4 with a 1.25 ms step.

Edit these switches in `CTRL.m` (the function's returned structure is named `c`):

- `position.velocity_ff.enable = false`
- `velocity.gravity_comp.enable = true`
- `rate.gyroscopic_ff.enable = true`
- `dob.enable = true`
- `moce.enable = false`

Velocity control adds nominal gravity when enabled, multiplies by nominal
mass, rotates to body, and applies the C++ force limit and nonnegative world-Z
force constraint. Gyroscopic feed-forward adds `omega × (J_nominal*omega)`.
Rate torque limits apply after disturbance compensation.

The torque-domain observer uses

```
Q(s) = wc² / (s² + sqrt(2)*wc*s + wc²),   wc = 2 rad/s
B_d_hat = J_nominal * sQ(s)[B_omega] - Q(s)[B_tau_effective]
B_tau_cmd = saturate(B_tau_nom - B_d_hat)
```

The second-order filters use the C++ exact ZOH state transition. At each sample,
the DOB predictor bounds `B_tau_nom - previous_d_hat` and subtracts
`omega × (J_nominal*omega)` to obtain effective torque; the estimate is then
updated and used in the final bounded torque command. This explicit predictor
matches C++ ordering and avoids an algebraic loop. The gyroscopic term is
removed from the observer input regardless of whether gyro feed-forward is
enabled, because rigid-body angular acceleration excludes that term. Disabled
DOB publishes exactly zero and passes bounded nominal torque.

MOCE filters **commanded `B_F_cmd`**, never actual thrust or plant force, with
its own 2 rad/s Q filter. It integrates
`gamma .* ((J_nominal \ skew(QF))' * B_d_hat)` with C++ force/excitation gates,
rate limits, and projected offset limits. Z learning is disabled by default.
The allocator gets the **raw** estimate; `B_c_filtered` is monitoring only.
Disabled MOCE holds its configured initial CoM. This free-flight model has no
ground contact or arming state; C++ airborne/height gating is intentionally
omitted rather than adding a hidden altitude input to the specified interfaces.
With DOB disabled, MOCE's zero disturbance input produces no adaptation.

Allocation follows the C++ two-stage non-sequential solve: high-frequency yaw
reaction split, A1 using current servo directions and CoM moment arms, then A2
for lateral force and remaining yaw, solving for `sin(theta)` with angle limits.
Near-singular solves use the configured regularization. `B_c_hat` modifies
`rotor_position - B_c_hat` explicitly. Thrust commands and servo commands are
both four-element columns. Saturation means allocation need not exactly realize
an infeasible commanded wrench.

Both `CTRL.nominal.propeller` and `PLANT.propeller` expose independent
`b_over_k` and `reaction_torque_ratio`. V1 used a single 0.01 m coefficient for
all reaction components. V2's explicit generalization uses `b_over_k` for
lateral reaction components and `reaction_torque_ratio` for the axial component:
`spin * diag([b_over_k,b_over_k,reaction_torque_ratio])*direction*thrust`.
Their equal defaults reproduce V1; changing one does not overwrite the other.
A1 roll/pitch uses the lateral coefficient and its yaw reaction row uses the
axial coefficient. The plant uses the same equation with its own coefficients.
Neither allocator nor controller reads `PLANT` parameters.

## Validation

`test_v2` runs structural checks, independent mathematical checks, a 10 s hover,
all five switches in both states, a payload/nonzero command run, and repeatable
sensor-noise checks. It rejects nonfinite values, wrong command widths, exceeded
torque/CoM bounds, and unsuccessful default hover settling. Algebraic loops are
configured as compilation errors. `validation.txt` records executed results.
The gravity-disabled short case checks numerical validity, not stable hover.
These are smoke/regression checks, not exhaustive stability certification for
arbitrary payloads, gains, or aggressive trajectories.

`kernels/*.m` are the source for embedded MATLAB Function blocks and shared math.
After editing embedded-kernel source, run `build_v2` to regenerate the saved model.
Parameter edits do not require rebuilding. `export_v2_diagram` regenerates the
native diagram preview. Zero transport-delay settings may emit Simulink
feedthrough warnings; the plant/actuator integrators break the feedback loops,
and compilation treats any actual algebraic loop as an error.
Only MATLAB, Simulink, and the
MATLAB Function block facilities are required; V1's Robotics/DSP/VR library
links are replaced by explicit equations.
