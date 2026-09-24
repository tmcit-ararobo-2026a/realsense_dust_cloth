"""
Physics model and trajectory fitting for cloth projectile under aerodynamic drag.
"""

import numpy as np
from scipy.optimize import curve_fit


class ClothPhysicsModel:
    GRAVITY = 9.80665

    @staticmethod
    def simulate_trajectory(v0: float, angle_deg: float, k_drag: float,
                            start_pos=(0.0, 0.0, 0.0), dt=0.01, max_t=2.5):
        """
        Simulate 3D trajectory under gravity and drag.
        Equation: a = -g - k * |v| * v
        where k = 0.5 * rho * Cd * A / m
        """
        angle_rad = np.deg2rad(angle_deg)
        x, y, z = float(start_pos[0]), float(start_pos[1]), float(start_pos[2])
        vx = v0 * np.cos(angle_rad)
        vy = 0.0
        vz = v0 * np.sin(angle_rad)

        t_list = [0.0]
        pts = [(x, y, z)]

        t = 0.0
        while t < max_t:
            speed = np.sqrt(vx * vx + vy * vy + vz * vz)
            if speed < 1e-4:
                break
            ax = -k_drag * speed * vx
            ay = -k_drag * speed * vy
            az = -ClothPhysicsModel.GRAVITY - k_drag * speed * vz

            vx += ax * dt
            vy += ay * dt
            vz += az * dt

            x += vx * dt
            y += vy * dt
            z += vz * dt
            t += dt

            t_list.append(t)
            pts.append((x, y, z))

            if z < -0.1 and t > 0.1:  # Hit ground
                break

        return np.array(t_list), np.array(pts)

    @staticmethod
    def fit_trajectory(time_array, points_array, initial_guess=(5.0, 50.0, 0.08)):
        """
        Fit recorded points [(x, y, z)] with time to the physical model.
        Returns: fitted (v0, angle_deg, k_drag)
        """
        if len(time_array) < 5:
            return None

        # Project 3D points to horizontal distance s = sqrt(x^2 + y^2) and vertical z
        s_obs = np.sqrt(points_array[:, 0]**2 + points_array[:, 1]**2)
        z_obs = points_array[:, 2]
        t_obs = time_array - time_array[0]

        def model_func(t, v0, angle_deg, k_drag):
            _, sim_pts = ClothPhysicsModel.simulate_trajectory(
                v0, angle_deg, k_drag, start_pos=(0.0, 0.0, z_obs[0]), dt=0.005, max_t=np.max(t_obs) + 0.1
            )
            sim_s = np.sqrt(sim_pts[:, 0]**2 + sim_pts[:, 1]**2)
            sim_z = sim_pts[:, 2]
            sim_t = np.linspace(0, 0.005 * len(sim_pts), len(sim_pts))
            
            interp_s = np.interp(t, sim_t, sim_s)
            interp_z = np.interp(t, sim_t, sim_z)
            return np.concatenate([interp_s, interp_z])

        obs_target = np.concatenate([s_obs, z_obs])

        try:
            popt, _ = curve_fit(
                model_func, t_obs, obs_target,
                p0=initial_guess,
                bounds=([0.5, 20.0, 0.001], [25.0, 75.0, 0.5]),
                maxfev=2000
            )
            return {
                "v0_mps": float(popt[0]),
                "angle_deg": float(popt[1]),
                "drag_k": float(popt[2]),
                "max_height_m": float(np.max(z_obs)),
                "flight_distance_m": float(np.max(s_obs)),
                "flight_duration_s": float(np.max(t_obs))
            }
        except Exception as e:
            return {
                "v0_mps": float(initial_guess[0]),
                "angle_deg": float(initial_guess[1]),
                "drag_k": float(initial_guess[2]),
                "max_height_m": float(np.max(z_obs)),
                "flight_distance_m": float(np.max(s_obs)),
                "flight_duration_s": float(np.max(t_obs)),
                "error": str(e)
            }

