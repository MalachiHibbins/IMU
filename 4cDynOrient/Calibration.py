import AdvKalman
import numpy as np

def calculate_Q(as_, max_cov_cal):
    a1 = as_[:, 0][:max_cov_cal]
    a2 = as_[:, 1][:max_cov_cal]
    a3 = as_[:, 2][:max_cov_cal]
    q1, q2, q3, q4 = AdvKalman.euler2EP([a1, a2, a3])
    Q = np.cov(np.vstack([q1, q2, q3, q4]))
    return Q