# Equality Constrained LQR

- [Equality Constrained LQR](#equality-constrained-lqr)
  - [Benchmarks Implemented](#benchmarks-implemented)
  - [Benchmarking Results](#benchmarking-results)
  - [Directory Structure](#directory-structure)
  - [Install Instructions](#install-instructions)
    - [Matlab](#matlab)
    - [C++](#c)

In this repo we present our factor graph-based approach to solving equality constrained LQR problems, as well as implementations of other versions as benchmarks.  Please see [our paper](https://arxiv.org/abs/2011.01360) for additional details.

## Benchmarks Implemented
We compare three to solve equality constrained LQR problems:

1. Laine, Forrest, and Claire Tomlin. "Efficient computation of feedback control for equality-constrained lqr." 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019.

2. Sideris, Athanasios, and Luis A. Rodriguez. "A riccati approach to equality constrained linear quadratic optimal control." Proceedings of the 2010 American Control Conference. IEEE, 2010.

3. Our factor graph based approach

We also compare 2 additional trajectory optimization approaches (which do not generate feedback policies):

1. Matlab's QP solver `quadprog` (note: `lsqlin` could probably also be used)

2. KKT-based constrained least squares

## Benchmarking Results
**TODO**: update this section

We need to compare their
1. final cost
2. constraint violation
3. speed ( it is hard to compare for gtsam based method because it uses c++)

Benchmark:
Intel i7-8809G 3.10GHz CPU

First a low-dimensional example to demonstrate linear runtime with trajectory length T.  We use state and control dimensions n=m=3, and with m-1 dimensional local constraints at every time step:
| T | 100 | 200 | 300 | 400 | 500  | 600 |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
Laine (ms)  | **0.88** | **1.06** | **1.67** | **2.01** | **2.35** | **2.81**
Ours (ms) | 2.32 | 3.17 | 4.30 | 4.68 | 5.86 | 6.86

Now we show a comparison for larger control dimensions by fixing T=100 and increasing n and m together:

n, m | 10 | 20 | 30 | 40 | 50 | 60 
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
Laine (ms)  | **3.74** | 14.5 | 44.1 | 83.5 | 152.3 | 247.7
Ours (ms) | 3.81 | **11.8** | **27.1** | **51.2** | **99.0** | **170.2**

## Directory Structure
**TODO**: fill in this section
All functions start with "ecLQR_" are main solvers

C++ code is in `src`, `tests`, and `scripts`.

## Install Instructions

### Matlab

Dependencies:
* [GTSAM](https://github.com/borglab/gtsam) with the matlab wrapper.

After installing GTSAM with the matlab wrapper, no additional installation is required.

### C++

Dependencies:
* [GTSAM](https://github.com/borglab/gtsam)

From the root repo directory, run
```bash
mkdir build
cd build
cmake ..
make
```

To run the unit tests, run
```bash
make check
```

To run the scripts (e.g. `laine_benchmarks.cpp`), simply run
```bash
make laine_benchmarks.run
```
