equality-constraine-LQR-compare

In this repo we compare three approaches to solve equalit constrained LQR:

1. Laine, Forrest, and Claire Tomlin. "Efficient computation of feedback control for equality-constrained lqr." 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019.

2. Sideris, Athanasios, and Luis A. Rodriguez. "A riccati approach to equality constrained linear quadratic optimal control." Proceedings of the 2010 American Control Conference. IEEE, 2010.

3. Factor graph based approach


All functions start with "ecLQR_" are main solvers


We need to compare their
1. final cost
2. constraint violation
3. speed ( it is hard to compare for gtsam based method because it uses c++)
