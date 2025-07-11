import numpy as np

def T(euler_angles):
    """
    Calculates the transformation matrix T which determines the time derivative of the euler angles.
    Args:
        euler_angles (np.ndarray): Euler angles in the order [psi, theta, phi].
    returns:
        np.ndarray: Transformation matrix T (shape: (3, 3)).
    """
    psi, theta, phi = euler_angles
    return (1 / np.cos(theta)) * np.array([
        [0, np.sin(phi), np.cos(phi)],
        [0, np.cos(phi) * np.cos(theta), -np.sin(phi) * np.cos(theta)],
        [np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi) * np.sin(theta)]
    ])


def integrate(omegas, dt, eulers_initial=np.array([0, 0, 0])):
    """
    Integrate the angular velocities to get the euler angles.
    Args:
        omegas (np.ndarray): Angular velocities in the order [omega_x, omega_y, omega_z] (shape: (n, 3)).
        dt (float): Time step for integration.
        eulers_initial (np.ndarray): Initial euler angles in the order [psi, theta, phi] (shape: (3,)).
    Returns:
        np.ndarray: Integrated euler angles in the order [psi, theta, phi] (shape: (n, 3)).
    """
    eulers = [eulers_initial] # A list to store all the calulated euler angles
    for omega in omegas:
        T_matrix = T(eulers[-1]) # Calculate the transformation matrix T for the current euler angles
        euler_angles_dot = T_matrix @ omega # Calculate the time derivative of the euler angles
        new_euler_angles = eulers[-1] + euler_angles_dot * dt 
        eulers.append(new_euler_angles)
    return np.array(eulers[:-1]) # Exclude the last element to match the length of omegas

