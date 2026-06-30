# Cruise Mission 2 Diagnosis

## Static Checks

- `main_alg1_no_obstacle.m` uses `r0=[0;0;15]`, `rf=[30;30;15]`, `v0=[0;10;0]`, and `vf=[-5*sqrt(2);5*sqrt(2);0]` for cruising mission 2.
- `solve_alg1_cvx.m` keeps the paper SOCP form: `vbar=tf*v`, `abar=ts*a`, `square_pos(tf)<=ts`, Euler dynamics, velocity/thrust/tilt SOC constraints, `minimize(ts-sigma*tf)`, and the paper sigma update.

## Candidate A Feasibility

- Candidate A `tf = 5.254059180 s`, `gap = 3.112e-09`.
- Maximum velocity violation: 0.000e+00.
- Maximum acceleration violation: -4.719e-09.
- Maximum tilt violation: -1.735e-10 rad.
- Maximum r dynamics residual: 5.922e-11.
- Maximum vbar dynamics residual: 1.146e-14.
- Feasibility conclusion: feasible under the implemented checks.

## Closest Candidate to 6.96 s

- Closest candidate: `E_double_negative`.
- `tf = 6.371770875 s`, absolute difference from 6.96 s is 0.588229125 s.
- Its minimum angle from `v0` is 135.000 deg; its counterclockwise rotation from `v0` is 135.000 deg.

## 315 Degree Adjustment Note

- Candidate A has minimum angle 45.000 deg and counterclockwise rotation 45.000 deg from `v0` to `vf`.
- A phrase like "315 deg adjustment" is naturally associated with the long rotation direction for a 45 deg shortest-angle change. Under the current convex model, the vehicle is not forced to take that long rotation, and Candidate A solves to a shorter feasible trajectory.

## Solver Robustness

- mosek: status `Solved`, tf 5.254059180, message ``.
- sdpt3: status `Inaccurate/Solved`, tf 5.254059177, message ``.
- sedumi: status `unavailable_or_failed`, tf NaN, message `Algorithm 1 第 1 次迭代失败，CVX 状态为 "Failed"。`.

## Control Node Diagnostic

- formal_N_minus_1_control: status `Solved`, tf 5.254059180, delta from formal 0.000000000.
- diagnostic_N_control: status `Solved`, tf 5.254059195, delta from formal 0.000000014.

## Conclusion

Candidate A is a feasible solution and is significantly shorter than the paper-reported 6.96 s. Since the other benchmark cases match the paper, the most likely explanations are: the paper text for mission 2 has a parameter typo, the reported mission 2 time has a typo, or the paper used an additional hidden convention/setting that is not stated in the text. This diagnostic does not modify the formal mission 2 parameters.
