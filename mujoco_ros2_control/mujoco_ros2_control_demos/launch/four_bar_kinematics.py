import numpy as np

def analyze_fourbar(theta2, omega_2, epsilon_2, l1, l2, l3, l4):
    # Krok 2: długość BD
    BD = np.sqrt(l1**2 + l2**2 - 2 * l1 * l2 * np.cos(theta2))

    # Krok 3: gamma
    gamma = np.arccos((l3**2 + l4**2 - BD**2) / (2 * l3 * l4))

    # Krok 4: theta3
    numerator_theta3 = l4 * np.sin(gamma) - l2 * np.sin(theta2)
    denominator_theta3 = l1 + l3 - l2 * np.cos(theta2) - l4 * np.cos(gamma)
    theta3 = 2 * np.arctan(numerator_theta3 / denominator_theta3)

    # Krok 5: theta4
    numerator_theta4 = l2 * np.sin(theta2) - l3 * np.sin(gamma)
    denominator_theta4 = l2 * np.cos(theta2) + l4 - l1 - l3 * np.cos(gamma)
    theta4 = 2 * np.arctan(numerator_theta4 / denominator_theta4)

    # omega3 and omega4
    omega_3 = (
        omega_2 * (l2 * np.sin(theta4 := theta4) - theta2) / (l3 * np.sin(gamma))
    )
    omega_4 = omega_2 * (l2 * np.sin(theta3 - theta2)) / (l4 * np.sin(gamma))

    # epsilon3
    numerator_e3 = (
        epsilon_2 * l2 * np.sin(theta2 - theta4)
        + omega_2**2 * l2 * np.cos(theta2 - theta4)
        - omega_4**2 * l4
        + omega_3**2 * l3 * np.cos(theta4 - theta3)
    )
    denominator_e3 = l3 * np.sin(theta4 - theta3)
    epsilon_3 = numerator_e3 / denominator_e3

    # epsilon4
    numerator_e4 = (
        epsilon_2 * l2 * np.sin(theta2 - theta3)
        + omega_2**2 * l2 * np.cos(theta2 - theta3)
        - omega_3**2 * l3 * np.cos(theta4 - theta3)
        + omega_3**2 * l3
    )
    denominator_e4 = l4 * np.sin(theta4 - theta3)
    epsilon_4 = numerator_e4 / denominator_e4

    return {
        "theta1": np.pi,
        "theta2": theta2,
        "theta3": theta3,
        "theta4": theta4,
        "gamma": gamma,
        "omega2": omega_2,
        "omega3": omega_3,
        "omega4": omega_4,
        "epsilon2": epsilon_2,
        "epsilon3": epsilon_3,
        "epsilon4": epsilon_4,
    }