# `robot_arm.py` — Arm Low-Level Controller

This module is the **bridge between the teleoperation/IK layer and the robot's
motor firmware**. It owns one job: take a desired dual-arm joint configuration
(and optional feed-forward torques), and stream them to the robot's motors over
DDS at a fixed control rate, while keeping every non-arm joint locked in place.

It supports five Unitree platforms, each with its own controller class and joint
map:

| Class | Robot | Arm DOF (per call) | SDK IDL | Total motors |
|-------|-------|---------51066-----------|---------|--------------|
| `G1_29_ArmController` | G1 (29-DOF) | 14 (7 + 7) | `unitree_hg` | 35 |
| `G1_23_ArmController` | G1 (23-DOF) | 10 (5 + 5) | `unitree_hg` | 35 |
| `H1_2_ArmController`  | H1-2        | 14 (7 + 7) | `unitree_hg` | 35 |
| `H1_ArmController`    | H1          | 8  (4 + 4) | `unitree_go` | 20 |
| `H2_ArmController`    | H2          | 14 (7 + 7) | `unitree_hg` | 35 |

---

## 1. What data is sent to the robot

The controller publishes a `LowCmd_` message on a DDS topic. For **every** motor
on the robot (not just the arms), the message carries a `motor_cmd[i]` struct
with these fields:

| Field | Meaning | What this code puts in it |
|-------|---------|---------------------------|
| `mode` | Motor enable / control mode | `1` (enabled, FOC) for `hg` robots; `0x01`/`0x0A` for H1 (`go`) |
| `q`    | **Target joint position** (rad) | Arm joints: the (velocity-clipped) IK solution; all other joints: their frozen startup position |
| `dq`   | **Target joint velocity** (rad/s) | `0` for arm joints |
| `tau`  | **Feed-forward torque** (N·m) | Arm joints: `tauff_target` from IK (gravity comp via RNEA); `0` elsewhere |
| `kp`   | **Position gain** (stiffness) | Per-joint constant set once at startup (see §4) |
| `kd`   | **Velocity gain** (damping) | Per-joint constant set once at startup (see §4) |

Plus message-level fields: `mode_pr`, `mode_machine` (read back from the robot
on `hg` robots), `crc` (computed every cycle), and for H1 the `head`/`level_flag`/`gpio` bytes.

**Key point:** only the 14 (or 10 / 8) arm joints are actively driven from
teleop. Every other joint (legs, waist, ankles, head) is sent `q = startup
position` with a stiff `kp`/`kd` so it is *locked* — the robot holds its pose and
does not collapse. The arms are the only thing that move.

### Direction of data flow

```
XR / IK layer                       robot_arm.py                         Robot
─────────────                       ────────────                         ─────
solve_ik() ──► (sol_q, sol_tauff) ──► ctrl_dual_arm(q_target, tauff)
                                        │  (stored under ctrl_lock)
                                        ▼
                          _ctrl_motor_state() @ 250 Hz
                                        │  clip velocity, fill motor_cmd, CRC
                                        ▼
                          lowcmd_publisher.Write(msg) ──► rt/lowcmd ──► motors

motors ──► rt/lowstate ──► _subscribe_motor_state() @ ~500 Hz ──► lowstate_buffer
                                        │
                                        ▼
                       get_current_dual_arm_q() / _dq()  (read back to IK)
```

So the data going **to** the robot is the per-motor command struct above
(`q, dq, tau, kp, kd, mode`). The data coming **back** is each motor's measured
`q` (position) and `dq` (velocity), used both to lock the non-arm joints at
startup and to feed the IK solver / velocity clipping.

---

## 2. DDS topics

| Constant | Topic | Used for |
|----------|-------|----------|
| `kTopicLowCommand_Debug`  | `rt/lowcmd`   | Command publish in **debug mode** (`motion_mode=False`) — direct low-level control |
| `kTopicLowCommand_Motion` | `rt/arm_sdk`  | Command publish in **motion mode** (`motion_mode=True`) — runs alongside the on-board locomotion controller |
| `kTopicLowState`          | `rt/lowstate` | State subscribe (all robots) |

In **motion mode**, the controller writes `motor_cmd[kNotUsedJoint0].q = 1.0`.
This index is a *weight flag* the `arm_sdk` service reads to blend arm-SDK
commands with the running motion controller (`1.0` = full arm-SDK authority).
On `go_home`, it ramps that weight `1 → 0` to hand control back smoothly.

---

## 3. Threading model

Each controller spins up two daemon threads plus the main thread:

1. **`_subscribe_motor_state`** — continuously `Read()`s `rt/lowstate`, copies
   each motor's `q`/`dq` into a `G1_..._LowState`, and stores it in a
   thread-safe `DataBuffer`. Loops every `0.002 s` (~500 Hz).
2. **`_ctrl_motor_state`** — the control loop. Every `control_dt` it reads the
   latest `q_target`/`tauff_target`, velocity-clips them, writes them into the
   arm motor commands, computes the CRC, and publishes. Runs at **250 Hz**.
3. **caller thread** — calls `ctrl_dual_arm(q, tau)` which just stores the
   targets under `ctrl_lock` (non-blocking; the 250 Hz loop picks them up).

`DataBuffer` and `ctrl_lock` (a `threading.Lock`) protect the shared state
between these threads.

---

## 4. Default values (per robot)

### Control gains `kp` / `kd`

Gains are assigned **once at startup**, per joint, by category:

- **wrist** joints → `kp_wrist`, `kd_wrist`
- arm joints that are **not** wrists → `kp_low`, `kd_low`
- non-arm **weak** joints (ankle pitch + the shoulder/elbow joints) → `kp_low`, `kd_low`
- all other (strong) joints → `kp_high`, `kd_high`

| Robot | kp_high | kd_high | kp_low | kd_low | kp_wrist | kd_wrist |
|-------|--------:|--------:|-------:|-------:|---------:|---------:|
| G1_29 | 300.0 | 3.0 | 80.0  | 3.0 | 40.0 | 1.5 |
| G1_23 | 300.0 | 3.0 | 80.0  | 3.0 | 40.0 | 1.5 |
| H1_2  | 300.0 | 5.0 | 140.0 | 3.0 | 50.0 | 2.0 |
| H1    | 300.0 | 5.0 | 140.0 | 3.0 | — (no wrist) | — |
| H2    | 300.0 | 5.0 | 140.0 | 3.0 | 50.0 | 2.0 |

### Other defaults (all robots, unless noted)

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `control_dt` | `1/250 s` (= 0.004 s) | Control-loop period → **250 Hz** publish rate |
| `arm_velocity_limit` | `20.0` rad/s | Per-joint speed cap used by `clip_arm_q_target` |
| `speed_instant_max()` | sets limit to `30.0` | Jump straight to max speed |
| `speed_gradual_max(t=5.0)` | ramps `20 → 30` over `t` s | `limit = 20 + 10·min(1, t_elapsed/5)` |
| `motion_mode` | `False` | `True` → publish on `rt/arm_sdk`; `False` → `rt/lowcmd` |
| `simulation_mode` | `False` | `True` → **skip** velocity clipping (send raw IK targets) |
| `tolerance` (go_home) | `0.05` rad | "Close enough to zero" check |
| `max_attempts` (go_home) | `100` | Polling attempts before giving up |
| subscribe loop period | `0.002 s` | State read rate (~500 Hz) |
| `q_target` init | zeros (size 14/10/8) | Neutral target until first command |

---

## 5. The math: velocity clipping and the PD + feed-forward law

### 5.1 Velocity clipping (`clip_arm_q_target`)

To stop the arms from jumping when a new IK target is far from the current pose,
each command is scaled so no joint exceeds `arm_velocity_limit` in one control
tick:

```
delta         = q_target − q_current
motion_scale  = max(|delta|) / (velocity_limit · control_dt)
q_clipped     = q_current + delta / max(motion_scale, 1.0)
```

- `velocity_limit · control_dt` is the **max joint travel allowed in one tick**
  (e.g. `20 · 0.004 = 0.08 rad`).
- If the largest requested step already fits, `motion_scale ≤ 1`, `max(·,1)=1`,
  and the target passes through unchanged.
- If it's too big, `motion_scale > 1` and the whole delta vector is divided down
  by that factor — every joint is scaled by the **same** ratio, so the arm moves
  in a straight line in joint space toward the target, just slower. It reaches
  the target over several ticks instead of in one.

(In `simulation_mode` this clipping is skipped and `q_target` is sent raw.)

### 5.2 The PD + feed-forward torque law (where `kp`/`kd` are used)

This code does **not** compute the torque itself. It ships `q, dq, tau, kp, kd`
to each motor, and the **motor's on-board firmware** closes the loop every
control cycle using the standard Unitree joint-level law:

```
τ_motor = kp · (q_des − q_actual)  +  kd · (dq_des − dq_actual)  +  τ_ff
```

Mapping to the fields this file sets:

| Symbol | Source in code |
|--------|----------------|
| `q_des`  | `motor_cmd[id].q`  = clipped IK target (arms) / frozen pose (others) |
| `q_actual` | measured by the motor encoder (on-board) |
| `dq_des` | `motor_cmd[id].dq` = **0** |
| `dq_actual` | measured by the motor (on-board) |
| `τ_ff`   | `motor_cmd[id].tau` = `tauff_target` (arms) / 0 (others) |
| `kp`, `kd` | `motor_cmd[id].kp`, `.kd` (the constants in §4) |

Interpretation of each term:

- **`kp · (q_des − q_actual)`** — a spring pulling the joint toward the target
  angle. Larger `kp` ⇒ stiffer, tracks faster, but more overshoot/oscillation.
- **`kd · (dq_des − dq_actual)`** — since `dq_des = 0`, this is `−kd · dq_actual`,
  a damper opposing joint velocity. Larger `kd` ⇒ more damping, smoother, but
  sluggish if too high. It suppresses the oscillation `kp` would otherwise cause.
- **`τ_ff`** — feed-forward torque. Here it is the **gravity/inverse-dynamics
  compensation** computed in the IK layer by Pinocchio's RNEA:
  `sol_tauff = pin.rnea(model, data, sol_q, v, 0)` with `v = 0`
  (see `robot_arm_ik.py:282`). This pre-loads the torque needed to hold the arm
  against gravity, so the PD terms only have to correct the *error* — giving
  accurate tracking with comparatively low `kp`. This is why the arm `kp_low`
  (80–140) can be much softer than the body `kp_high` (300) and still hold pose.

So the effective dynamics of each arm joint is a **gravity-compensated
PD (mass-spring-damper) servo**:

```
τ = kp·(q_des − q) − kd·q̇ + τ_gravity
```

Tuning intuition: pick `kp` for the stiffness/tracking you want, then raise `kd`
until oscillation disappears (rough critical-damping target `kd ≈ 2·√(kp·I)`
for effective inertia `I`); the smaller wrist gains reflect the wrist's much
lower inertia and load.

---

## 6. Public API (per controller)

| Method | Purpose |
|--------|---------|
| `ctrl_dual_arm(q_target, tauff_target)` | Set the dual-arm position + feed-forward torque targets (thread-safe). Main entry point from teleop. |
| `get_current_dual_arm_q()` | Measured arm joint positions (rad). |
| `get_current_dual_arm_dq()` | Measured arm joint velocities (rad/s). |
| `get_current_motor_q()` | Measured positions of **all** body motors. |
| `ctrl_dual_arm_go_home()` | Drive both arms to `q = 0`; in motion mode also ramps the arm-SDK weight back to 0. |
| `speed_gradual_max(t=5.0)` | Ramp the velocity limit `20 → 30` rad/s over `t` seconds. |
| `speed_instant_max()` | Set velocity limit to `30` rad/s immediately. |
| `get_mode_machine()` | (`hg` robots) read `mode_machine` from the robot state. |

---

## 7. Typical usage

```python
from robot_arm import G1_29_ArmController
from robot_arm_ik import G1_29_ArmIK

arm    = G1_29_ArmController(motion_mode=False, simulation_mode=False)
arm_ik = G1_29_ArmIK()

arm.speed_gradual_max()           # ease the velocity limit up

while running:
    q  = arm.get_current_dual_arm_q()
    dq = arm.get_current_dual_arm_dq()
    sol_q, sol_tauff = arm_ik.solve_ik(L_wrist_target, R_wrist_target, q, dq)
    arm.ctrl_dual_arm(sol_q, sol_tauff)   # the 250 Hz loop streams it to the motors

arm.ctrl_dual_arm_go_home()       # park the arms
```

`ChannelFactoryInitialize(0)` must be called before constructing a controller
(`0` = real robot, `1` = simulation), as shown in the `__main__` block.
