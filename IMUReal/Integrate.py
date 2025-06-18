import numpy as np

def T(euler_angles):
    psi, theta, phi = euler_angles
    return (1 / np.cos(theta)) * np.array([
        [0, np.sin(phi), np.cos(phi)],
        [0, np.cos(phi) * np.cos(theta), -np.sin(phi) * np.cos(theta)],
        [np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi) * np.sin(theta)]
    ])


def integrate(omegas, dt, eulers_initial=np.array([0, 0, 0])):
    """
    Integrate the angular velocities to get the euler angles.
    :param omegas: A 2D numpy array of shape (n, 3) where n is the number of time steps and each row contains the angular velocities (omega_x, omega_y, omega_z).
    :param dt: The time step for integration.
    :param eulers_initial: Initial euler angles.
    """
    eulers = [eulers_initial] # A list to store all the calulated euler angles
    for omega in omegas:
        T_matrix = T(eulers[-1]) # Calculate the transformation matrix T for the current euler angles
        euler_angles_dot = T_matrix @ omega # Calculate the time derivative of the euler angles
        new_euler_angles = eulers[-1] + euler_angles_dot * dt 
        eulers.append(new_euler_angles)
    return np.array(eulers[:-1]) # Exclude the last element to match the length of omegas

