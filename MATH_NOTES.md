# Math Notes

This document describes the mathematical foundations used in ControlWorkbench.

## Extended Kalman Filter (EKF)

### State Prediction

Given a nonlinear motion model:
```
x_{k+1} = f(x_k, u_k, dt) + w_k
```

The EKF linearizes around the current estimate:

**Jacobians:**
- State transition: `F = ?f/?x` evaluated at `(x?_k, u_k)`
- Process noise: `G = ?f/?w` or input Jacobian `?f/?u`

**Prediction equations:**
```
x??_{k+1} = f(x?_k, u_k, dt)
P?_{k+1} = F · P_k · F? + G · Q · G?
```

### Measurement Update

Given a nonlinear measurement model:
```
z_k = h(x_k) + v_k
```

**Measurement Jacobian:**
```
H = ?h/?x evaluated at x??
```

**Update equations:**
```
y = z - h(x??)           # Innovation
S = H · P? · H? + R      # Innovation covariance
K = P? · H? · S?¹        # Kalman gain
x?? = x?? + K · y         # State update
P? = (I - K·H) · P?      # Covariance update (simple form)
```

**Joseph form** (numerically stable):
```
P? = (I - K·H) · P? · (I - K·H)? + K · R · K?
```

## Motion Models

### Unicycle 2D
State: `x = [px, py, ?]`
Input: `u = [v, ?]`

```
f(x, u, dt) = [px + v·cos(?)·dt]
              [py + v·sin(?)·dt]
              [? + ?·dt        ]

F = [1  0  -v·sin(?)·dt]
    [0  1   v·cos(?)·dt]
    [0  0   1          ]

G = [cos(?)·dt  0 ]
    [sin(?)·dt  0 ]
    [0          dt]
```

### Constant Velocity 2D
State: `x = [px, py, vx, vy]`
Input: `u = [ax, ay]` (acceleration)

```
f(x, u, dt) = [px + vx·dt + ½ax·dt²]
              [py + vy·dt + ½ay·dt²]
              [vx + ax·dt          ]
              [vy + ay·dt          ]

F = [1  0  dt  0]
    [0  1  0  dt]
    [0  0  1   0]
    [0  0  0   1]
```

### Yaw-only Strapdown
State: `x = [?, b_g]` (yaw, gyro bias)
Input: `u = [?_z]` (measured gyro)

```
f(x, u, dt) = [? + (?_z - b_g)·dt]
              [b_g               ]

F = [1  -dt]
    [0   1 ]
```

## Measurement Models

### GPS Position 2D
Measures: `z = [px, py]`

```
h(x) = [x[0]]
       [x[1]]

H = [1  0  0  ...]
    [0  1  0  ...]
```

### Yaw (Magnetometer)
Measures: `z = [?]`

```
h(x) = [x[yaw_index]]

H = [0  0  1  0  ...]  (1 at yaw index)
```

### Range-Bearing to Beacon
Beacon at `(bx, by)`, measures: `z = [r, ?]`

```
dx = px - bx
dy = py - by
r = ?(dx² + dy²)
? = atan2(dy, dx) - ?

h(x) = [r]
       [?]

H = [dx/r      dy/r      0    ]
    [-dy/r²    dx/r²    -1    ]
```

## Complementary Filter

For fusing high-frequency (gyro) and low-frequency (accelerometer/magnetometer) signals:

```
?[k] = ?·(?[k-1] + ?·dt) + (1-?)·?_ref
```

Where:
- `? = ?/(? + dt)` is the filter coefficient
- `? = 1/(2?·fc)` is the time constant
- `fc` is the cutoff frequency

Higher `?` ? more weight on gyro (better high-frequency response)
Lower `?` ? more weight on reference (better low-frequency/drift correction)

## PID Controller

### Continuous-time
```
u(t) = Kp·e(t) + Ki·?e(?)d? + Kd·(de/dt)
```

### Discrete-time (Tustin/Bilinear)

**Proportional:**
```
P[k] = Kp · e[k]
```

**Integral (trapezoidal):**
```
I[k] = I[k-1] + Ki · (e[k] + e[k-1]) · dt/2
```

**Derivative (with filter):**
```
D_raw[k] = (e[k] - e[k-1]) / dt
D[k] = ?·D_raw[k] + (1-?)·D[k-1]
```
where `? = dt/(Tf + dt)` and `Tf` is the filter time constant.

**Anti-windup (back-calculation):**
```
If u_sat ? u:
    I[k] += Kb · (u_sat - u) · dt
```
where `Kb ? 1/Ti = Ki/Kp`.

## LQR Design

### Continuous-time
Minimize: `J = ?(x?Qx + u?Ru)dt`

Solve Continuous Algebraic Riccati Equation (CARE):
```
A?P + PA - PBR?¹B?P + Q = 0
```

Optimal gain: `K = R?¹B?P`

### Discrete-time
Minimize: `J = ?(x?Qx + u?Ru)`

Solve Discrete Algebraic Riccati Equation (DARE):
```
A?PA - P - A?PB(R + B?PB)?¹B?PA + Q = 0
```

Optimal gain: `K = (R + B?PB)?¹B?PA`

## Step Response Metrics

- **Rise Time (10-90%)**: Time to go from 10% to 90% of final value
- **Overshoot**: `(peak - final) / (final - initial) × 100%`
- **Settling Time (2%)**: Time to stay within 2% of final value
- **Steady-State Error**: `|target - final_value|`
