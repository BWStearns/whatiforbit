//! Derivative-free optimizer core: adaptive Nelder–Mead inner minimizer
//! wrapped in an augmented-Lagrangian outer loop for equality/inequality
//! constraints. Deliberately dependency-free and resumable so the app can
//! amortize solves across frames (wasm has no threads in this build).

/// One objective evaluation: cost plus scaled constraint values.
#[derive(Debug, Clone, Default)]
pub struct Eval {
    /// Objective, already normalized to O(1).
    pub cost: f64,
    /// Equality residuals, scaled so |c| <= 1 means "within tolerance".
    pub eq: Vec<f64>,
    /// Inequality values g (feasible when g <= 0), scaled to O(1).
    pub ineq: Vec<f64>,
}

/// Adaptive Nelder–Mead (Gao & Han coefficients) over a scalar merit.
pub struct NelderMead {
    /// (point, merit) pairs; simplex[0] is the current best after `order`.
    simplex: Vec<(Vec<f64>, f64)>,
    dim: usize,
}

impl NelderMead {
    pub fn new(x0: &[f64], steps: &[f64], merit: &mut impl FnMut(&[f64]) -> f64) -> Self {
        let dim = x0.len();
        let mut simplex = Vec::with_capacity(dim + 1);
        simplex.push((x0.to_vec(), merit(x0)));
        for i in 0..dim {
            let mut x = x0.to_vec();
            x[i] += steps[i].max(1e-12);
            let m = merit(&x);
            simplex.push((x, m));
        }
        let mut nm = Self { simplex, dim };
        nm.order();
        nm
    }

    fn order(&mut self) {
        self.simplex
            .sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    }

    pub fn best(&self) -> (&[f64], f64) {
        (&self.simplex[0].0, self.simplex[0].1)
    }

    /// Spread of merit values across the simplex (convergence signal).
    pub fn spread(&self) -> f64 {
        (self.simplex[self.dim].1 - self.simplex[0].1).abs()
    }

    /// Run `iters` NM iterations (each costs 1–2 merit evaluations, worst
    /// case dim+2 on a shrink). Returns evaluations spent.
    pub fn run(&mut self, iters: usize, merit: &mut impl FnMut(&[f64]) -> f64) -> usize {
        let n = self.dim as f64;
        // Gao–Han adaptive coefficients.
        let (alpha, beta, gamma, delta) = (
            1.0,
            1.0 + 2.0 / n,
            (0.75 - 1.0 / (2.0 * n)).max(0.1),
            1.0 - 1.0 / n,
        );
        let mut evals = 0;

        for _ in 0..iters {
            // Centroid of all but worst.
            let mut centroid = vec![0.0; self.dim];
            for (x, _) in &self.simplex[..self.dim] {
                for (c, xi) in centroid.iter_mut().zip(x) {
                    *c += xi / n;
                }
            }
            let worst = self.simplex[self.dim].clone();
            let second_worst = self.simplex[self.dim - 1].1;
            let best = self.simplex[0].1;

            let point_along = |t: f64| -> Vec<f64> {
                centroid
                    .iter()
                    .zip(&worst.0)
                    .map(|(c, w)| c + t * (c - w))
                    .collect()
            };

            let xr = point_along(alpha);
            let fr = merit(&xr);
            evals += 1;

            if fr < best {
                // Try expansion.
                let xe = point_along(alpha * beta);
                let fe = merit(&xe);
                evals += 1;
                self.simplex[self.dim] = if fe < fr { (xe, fe) } else { (xr, fr) };
            } else if fr < second_worst {
                self.simplex[self.dim] = (xr, fr);
            } else {
                // Contraction (outside if reflection helped at all).
                let (xc, fc) = if fr < worst.1 {
                    let xc = point_along(alpha * gamma);
                    let fc = merit(&xc);
                    (xc, fc)
                } else {
                    let xc = point_along(-gamma);
                    let fc = merit(&xc);
                    (xc, fc)
                };
                evals += 1;
                if fc < worst.1.min(fr) {
                    self.simplex[self.dim] = (xc, fc);
                } else {
                    // Shrink toward best.
                    let best_x = self.simplex[0].0.clone();
                    for entry in self.simplex[1..].iter_mut() {
                        for (xi, bi) in entry.0.iter_mut().zip(&best_x) {
                            *xi = bi + delta * (*xi - bi);
                        }
                        entry.1 = merit(&entry.0);
                        evals += 1;
                    }
                }
            }
            self.order();
        }
        evals
    }
}

/// Augmented-Lagrangian state over an `Eval`-producing objective.
pub struct AugLag {
    pub lambda_eq: Vec<f64>,
    pub mu: f64,
    pub(super) last_violation: f64,
}

impl AugLag {
    pub fn new(n_eq: usize) -> Self {
        Self {
            lambda_eq: vec![0.0; n_eq],
            mu: 10.0,
            last_violation: f64::INFINITY,
        }
    }

    /// Scalar merit for an evaluation under current multipliers.
    pub fn merit(&self, e: &Eval) -> f64 {
        let mut m = e.cost;
        for (c, l) in e.eq.iter().zip(&self.lambda_eq) {
            m += l * c + 0.5 * self.mu * c * c;
        }
        for g in &e.ineq {
            let v = g.max(0.0);
            m += 0.5 * self.mu * v * v;
        }
        m
    }

    /// Multiplier update from the current best point's residuals. The penalty
    /// weight only escalates when the violation has stopped improving — an
    /// unconditional ramp turns the merit surface into a cliff wall that
    /// Nelder–Mead cannot navigate (observed: mu at 1e7 stalling refinements
    /// one residual-width from feasibility).
    pub fn update(&mut self, e: &Eval) {
        for (l, c) in self.lambda_eq.iter_mut().zip(&e.eq) {
            *l += self.mu * c;
        }
        let violation = Self::violation(e);
        if violation > 0.8 * self.last_violation {
            self.mu = (self.mu * 4.0).min(1e5);
        }
        self.last_violation = violation;
    }

    /// Worst constraint violation (scaled units; <= 1 means within tol).
    pub fn violation(e: &Eval) -> f64 {
        let eq = e.eq.iter().fold(0.0_f64, |a, c| a.max(c.abs()));
        let ineq = e.ineq.iter().fold(0.0_f64, |a, g| a.max(g.max(0.0)));
        eq.max(ineq)
    }
}
