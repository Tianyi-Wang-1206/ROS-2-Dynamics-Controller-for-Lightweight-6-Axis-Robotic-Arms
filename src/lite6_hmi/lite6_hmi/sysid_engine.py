import numpy as np
import math
import os
from scipy.signal import savgol_filter
from scipy.optimize import lsq_linear
from scipy.interpolate import interp1d
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory

class SysIdEngine:
    """
    Mathematical engine for System Identification.
    """
    def __init__(self):
        # Initialize Pinocchio for Inverse Dynamics (RNEA) calculations
        pkg_path = get_package_share_directory('lite6_description')
        urdf_path = os.path.join(pkg_path, 'urdf', 'lite6.urdf')
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        self.N_f = 7          
        self.f0 = 0.1         
        self.w = 2 * np.pi * self.f0
        self.T_total = 10.0
        
        self.a = np.zeros((6, self.N_f), dtype=np.float64)
        self.b = np.zeros((6, self.N_f), dtype=np.float64)
        self.generate_fourier_coefficients()

        # Data buffer to be populated by the external caller (GUI Node)
        self.record_data = [] 

    def generate_fourier_coefficients(self):
        """
        Generate symmetrical Fourier coefficients satisfying boundary conditions.
        Ensures zero position, velocity, and acceleration at t=0 and t=T_total.
        """
        np.random.seed(42) 
        # Maximum absolute deflection limits for each joint (radians)
        max_pos_limits = [0.4, 0.6, 0.5, 1.0, 1.0, 3.0]
        
        N = self.N_f

        for i in range(6):
            # Generate base shape using standard uniform distribution
            self.a[i, 0:N-1] = np.random.uniform(-1.0, 1.0, N-1)
            self.b[i, 0:N-2] = np.random.uniform(-1.0, 1.0, N-2)
            
            # Enforce boundary conditions to prevent motor shocks at start/end
            self.a[i, N-1] = -np.sum(self.a[i, 0:N-1])
            
            C1 = -np.sum([(l+1) * self.b[i, l] for l in range(N-2)])
            C2 = -np.sum([self.b[i, l] / (l+1) for l in range(N-2)])
            D = (1.0 - 2.0*N) / (N * (N - 1.0))
            self.b[i, N-2] = (C1 / N - N * C2) / D
            self.b[i, N-1] = (-C1 / (N - 1.0) + (N - 1.0) * C2) / D

            # Simulate trajectory over T_total to find maximum theoretical offset
            t_samples = np.linspace(0, self.T_total, 200)
            max_offset = 0.0
            
            for t in t_samples:
                q_offset, _, _ = self.get_fourier_point(i, t, np.zeros(6))
                max_offset = max(max_offset, abs(q_offset))
            
            # Scale coefficients to match the desired maximum amplitude limits
            if max_offset > 0:
                scale_factor = max_pos_limits[i] / max_offset
                self.a[i, :] *= scale_factor
                self.b[i, :] *= scale_factor

    def get_fourier_point(self, i, t, q0):
        """
        Calculate target position, velocity, and acceleration at time t.
        """
        q = q0[i]
        dq = 0.0
        ddq = 0.0
        for l in range(1, self.N_f + 1):
            wl = self.w * l
            a_il = self.a[i, l-1]
            b_il = self.b[i, l-1]
            
            q += (a_il / wl) * np.sin(wl * t) - (b_il / wl) * np.cos(wl * t) + (b_il / wl)
            dq += a_il * np.cos(wl * t) + b_il * np.sin(wl * t)
            ddq += -a_il * wl * np.sin(wl * t) + b_il * wl * np.cos(wl * t)
        return q, dq, ddq

    def calculate_least_squares(self) -> str:
        """
        Execute SVD Least Squares on recorded data and return formatted YAML string.
        """
        if not self.record_data:
            return "# Error: No data recorded for System Identification."
        
        t_seq = np.array([d['t'] for d in self.record_data], dtype=np.float64)
        q_seq = np.array([d['q'] for d in self.record_data], dtype=np.float64)
        dq_seq = np.array([d['dq'] for d in self.record_data], dtype=np.float64)
        tau_seq = np.array([d['tau_cmd'] for d in self.record_data], dtype=np.float64)

        dt_mean = np.mean(np.diff(t_seq))
        dt_uniform = dt_mean
        t_uniform = np.arange(t_seq[0], t_seq[-1], dt_uniform)

        f_q = interp1d(t_seq, q_seq, axis=0, kind='linear', fill_value="extrapolate")
        f_dq = interp1d(t_seq, dq_seq, axis=0, kind='linear', fill_value="extrapolate")
        f_tau = interp1d(t_seq, tau_seq, axis=0, kind='linear', fill_value="extrapolate")
        
        q_uniform = f_q(t_uniform)
        dq_uniform = f_dq(t_uniform)
        tau_uniform = f_tau(t_uniform)

        window_length = int(0.1 / dt_uniform) 
        if window_length % 2 == 0:
            window_length += 1
            
        ddq_smooth = np.zeros_like(dq_uniform)
        for i in range(6):
            ddq_smooth[:, i] = savgol_filter(dq_uniform[:, i], window_length, polyorder=3, deriv=1, delta=dt_uniform)

        I_a = np.zeros(6, dtype=np.float64)
        F_v = np.zeros(6, dtype=np.float64)
        F_c = np.zeros(6, dtype=np.float64)

        for i in range(6):
            Y = []
            W = []
            for k in range(len(t_uniform)):
                q = q_uniform[k]
                dq = dq_uniform[k]
                ddq = ddq_smooth[k]
                tau_cmd = tau_uniform[k]
                
                if abs(dq[i]) < 0.05:
                    continue
                
                tau_rnea = pin.rnea(self.model, self.data, q, dq, ddq)
                y = tau_cmd[i] - tau_rnea[i]
            
                w = [ddq[i], dq[i], math.tanh(300.0 * dq[i])]
                
                Y.append(y)
                W.append(w)
                
            Y = np.array(Y, dtype=np.float64)
            W = np.array(W, dtype=np.float64)
            
            if len(Y) < 200:
                continue 
            
            result = lsq_linear(W, Y, bounds=(0.0, np.inf))
            theta = result.x

            I_a[i] = theta[0]
            F_v[i] = theta[1]
            F_c[i] = theta[2]

        def format_list(arr):
            return "[" + ", ".join([f"{val:.4f}" for val in arr]) + "]"

        yaml_str = "# --- Identified Dynamic Parameters ---\n"
        yaml_str += f"armature:           {format_list(I_a)}\n"
        yaml_str += f"friction_v_nominal: {format_list(F_v)}\n"
        yaml_str += f"friction_c_nominal: {format_list(F_c)}\n"
        
        return yaml_str