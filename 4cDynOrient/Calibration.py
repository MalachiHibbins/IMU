import AdvKalman
import numpy as np

def calculate_R(as_, max_cov_cal):
    a1 = as_[:, 0][0:max_cov_cal]
    a2 = as_[:, 1][0:max_cov_cal]
    a3 = as_[:, 2][0:max_cov_cal]
    print(len(a1), len(a2), len(a3))
    q = AdvKalman.euler2EP([a1, a2, a3])
    
    Ea1 = q[1].mean()
    Ea2 = q[2].mean()
    Ea3 = q[3].mean()
    E = AdvKalman.euler2EP([Ea1, Ea2, Ea3])
    print(E)
    sum = np.zeros((4, 4))
    
    for qi in q.T:
        s = (qi-E)*(qi-E).T
        sum += s
    
    R = sum / (max_cov_cal - 1)
    return R