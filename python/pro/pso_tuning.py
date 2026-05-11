import numpy as np
from simulator import simulate

def run_pso(q_init, target, obs_list, axes_list):

    n_particles = 30   # tăng diversity
    n_iter = 30

    bounds = np.array([
        [5, 30],    # gamma
        [0.1, 5],   # eta
        [1, 20],    # mu
        [1, 30]     # ai
    ])

    dim = 4

    X = np.random.uniform(bounds[:,0], bounds[:,1], (n_particles, dim))
    V = np.zeros_like(X)

    pbest = X.copy()
    pbest_val = np.full(n_particles, np.inf)

    gbest = None
    gbest_val = np.inf

    prev_best = np.inf

    for it in range(n_iter):

        for i in range(n_particles):

            cost = simulate(X[i], q_init, target, obs_list, axes_list)

            if cost < pbest_val[i]:
                pbest_val[i] = cost
                pbest[i] = X[i]

            if cost < gbest_val:
                gbest_val = cost
                gbest = X[i]

        # ===== RESET nếu bị stuck =====
        if it > 5 and abs(prev_best - gbest_val) < 1e-3:
            print("[PSO] RESET SWARM")
            X = np.random.uniform(bounds[:,0], bounds[:,1], X.shape)

        prev_best = gbest_val

        # ===== UPDATE =====
        r1 = np.random.rand(n_particles, dim)
        r2 = np.random.rand(n_particles, dim)

        V = 0.9*V + 2*r1*(pbest - X) + 2*r2*(gbest - X)
        X += V

        X = np.clip(X, bounds[:,0], bounds[:,1])

        print(f"[PSO] Iter {it} | Best = {gbest_val:.3f}")

    return gbest