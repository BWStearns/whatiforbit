//! Dormand–Prince 5(4) adaptive Runge–Kutta over the 7-component state.

use crate::dynamics::Y;

const STAGES: usize = 7;

// Butcher tableau (Dormand & Prince 1980).
const A: [[f64; 6]; STAGES] = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [1.0 / 5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [3.0 / 40.0, 9.0 / 40.0, 0.0, 0.0, 0.0, 0.0],
    [44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0, 0.0, 0.0, 0.0],
    [
        19372.0 / 6561.0,
        -25360.0 / 2187.0,
        64448.0 / 6561.0,
        -212.0 / 729.0,
        0.0,
        0.0,
    ],
    [
        9017.0 / 3168.0,
        -355.0 / 33.0,
        46732.0 / 5247.0,
        49.0 / 176.0,
        -5103.0 / 18656.0,
        0.0,
    ],
    [
        35.0 / 384.0,
        0.0,
        500.0 / 1113.0,
        125.0 / 192.0,
        -2187.0 / 6784.0,
        11.0 / 84.0,
    ],
];
// 5th-order solution weights (same as the last A row; FSAL).
const B5: [f64; STAGES] = [
    35.0 / 384.0,
    0.0,
    500.0 / 1113.0,
    125.0 / 192.0,
    -2187.0 / 6784.0,
    11.0 / 84.0,
    0.0,
];
// Embedded 4th-order weights.
const B4: [f64; STAGES] = [
    5179.0 / 57600.0,
    0.0,
    7571.0 / 16695.0,
    393.0 / 640.0,
    -92097.0 / 339200.0,
    187.0 / 2100.0,
    1.0 / 40.0,
];

#[derive(Debug, Clone, Copy)]
pub struct Dp54 {
    pub rtol: f64,
    pub atol: f64,
    pub max_step_s: f64,
}

impl Default for Dp54 {
    fn default() -> Self {
        Self {
            rtol: 1e-10,
            atol: 1e-9,
            max_step_s: 300.0,
        }
    }
}

const T_EPS: f64 = 1e-9;

impl Dp54 {
    /// Integrate y' = f(y) from `t0` to `t1` (either direction).
    ///
    /// `observer(t, y)` is called at every multiple of `output_dt` past `t0`
    /// (exclusive of t0) and at `t1` itself; internal steps are clamped so
    /// these points are hit exactly.
    pub fn integrate<F>(
        &self,
        f: F,
        t0: f64,
        y0: Y,
        t1: f64,
        output_dt: f64,
        mut observer: impl FnMut(f64, &Y),
    ) -> Y
    where
        F: Fn(&Y) -> Y,
    {
        if (t1 - t0).abs() < T_EPS {
            return y0;
        }
        let dir = (t1 - t0).signum();
        let mut t = t0;
        let mut y = y0;
        let mut h_abs = self.max_step_s.min((t1 - t0).abs());
        let mut next_out = t0 + dir * output_dt;

        while (t1 - t) * dir > T_EPS {
            // Clamp the step so we land exactly on the next output point / end.
            let target = if (next_out - t1) * dir < 0.0 { next_out } else { t1 };
            let h = dir * h_abs.min((target - t) * dir);

            // Stage evaluations.
            let mut k = [[0.0; 7]; STAGES];
            k[0] = f(&y);
            for s in 1..STAGES {
                let mut ys = y;
                for i in 0..7 {
                    let mut acc = 0.0;
                    for (j, kj) in k.iter().enumerate().take(s) {
                        acc += A[s][j] * kj[i];
                    }
                    ys[i] += h * acc;
                }
                k[s] = f(&ys);
            }

            let mut y5 = y;
            let mut y4 = y;
            for i in 0..7 {
                let mut acc5 = 0.0;
                let mut acc4 = 0.0;
                for s in 0..STAGES {
                    acc5 += B5[s] * k[s][i];
                    acc4 += B4[s] * k[s][i];
                }
                y5[i] += h * acc5;
                y4[i] += h * acc4;
            }

            // Scaled max-norm error estimate.
            let mut err: f64 = 0.0;
            for i in 0..7 {
                let scale = self.atol + self.rtol * y[i].abs().max(y5[i].abs());
                err = err.max(((y5[i] - y4[i]) / scale).abs());
            }

            if err <= 1.0 {
                t += h;
                y = y5;
                if (t - next_out) * dir >= -T_EPS {
                    observer(t, &y);
                    next_out += dir * output_dt;
                } else if (t1 - t) * dir <= T_EPS {
                    observer(t, &y); // landed on t1 between output points
                }
            }

            let factor = if err > 1e-30 {
                (0.9 * err.powf(-0.2)).clamp(0.2, 5.0)
            } else {
                5.0
            };
            h_abs = (h.abs() * factor).clamp(1e-6, self.max_step_s);
        }
        y
    }
}
