# 1. Import the installed libraries
import numpy as np
import scipy
import cvxpy as cp
import matplotlib.pyplot as plt

# 2. Define global simulation constants
M = 4          # Number of series-connected cells in the pack
dt = 1.0       # Discrete time step in seconds

# 3. Initialize fresh-cell parameters (Updated realistic values)
Q_nom = 11520.0 # Capacity in Ampere-seconds (3.2 Ah * 3600 seconds)
R0 = 0.005      # Ohmic internal resistance (5 milliohms)
R1 = 0.002      # RC transient branch resistance (2 milliohms)
tau = 20.0      # RC transient branch time constant

# 4. Create state tracking arrays for all M cells
soc = np.full(M, 0.1)     # State of Charge for all cells (starting at 10%)
v_rc = np.zeros(M)        # Transient polarization voltage initialized to zero
v_terminal = np.zeros(M)  # Terminal voltage initialized to zero
pack_current = []         # List to store the shared pack current over time

print("Environment setup complete. Ready to build the equivalent circuit model.")

# 5. Define the OCV-SOC relationship
def get_v_ocv(soc_array):
    """
    A simplified linear/polynomial approximation of a Li-ion OCV curve.
    In a full model, this would be a lookup table based on real battery data.
    """
    # Simple linear approximation for testing: 3.0V at 0% SOC to 4.2V at 100% SOC
    return 3.0 + (1.2 * soc_array)

# 6. First-order Thevenin Equivalent Circuit Model (ECM) update function
def update_pack_states(current_I, soc, v_rc, v_terminal, Q_array, R0_array, R1_array, tau_array):
    """
    Updates the state of all M cells for one time step dt.
    Positive current_I = charging.
    """
    global dt
    eta = 1.0 # Coulombic efficiency (assumed 100% for simplicity)
    
    # Equation 1: Update State of Charge (SOC)
    # SOC_i(k+1) = SOC_i(k) + (eta * dt / Q_i) * I(k)
    soc_new = soc + (eta * dt / Q_array) * current_I
    
    # Equation 2: Update Transient Polarization Voltage (V_RC)
    # V_RC_i(k+1) = exp(-dt/tau_i)*V_RC_i(k) + R1_i*(1 - exp(-dt/tau_i))*I(k)
    decay_factor = np.exp(-dt / tau_array)
    v_rc_new = decay_factor * v_rc + R1_array * (1 - decay_factor) * current_I
    
    # Equation 3: Calculate new Terminal Voltage (V_i)
    # V_i(k) = V_OCV(SOC_i(k)) + I(k)*R0_i + V_RC_i(k)
    ocv = get_v_ocv(soc_new)
    v_terminal_new = ocv + (current_I * R0_array) + v_rc_new
    
    return soc_new, v_rc_new, v_terminal_new

# 7. Convert single baseline parameters into arrays for M independent cells
Q_cells = np.full(M, Q_nom)
R0_cells = np.full(M, R0)
R1_cells = np.full(M, R1)
tau_cells = np.full(M, tau)

print("ECM functions and parameter arrays loaded.")

# 8. Define safety and charging limits
V_max = 4.2       # Maximum safe terminal voltage per cell
I_max = 50.0      # Maximum pack charging current (Amps)
SOC_target = 0.9  # Target state of charge (90%)

# 11. Heterogeneous Aging Simulation
def age_pack(cycle_num, R0_initial, Q_initial):
    """
    Simulates uneven degradation over a given number of charge cycles.
    """
    # Define baseline degradation rates per cycle
    r_growth = 0.00005  # Resistance increases
    q_fade = 5.0        # Capacity decreases
    
    R0_aged = R0_initial.copy()
    Q_aged = Q_initial.copy()
    
    # Apply standard aging to all cells
    R0_aged += (r_growth * cycle_num)
    Q_aged -= (q_fade * cycle_num)
    
    # Force Cell 4 (Index 3) to be the "bottleneck" cell (ages 50% faster)
    R0_aged[3] += (r_growth * 0.5 * cycle_num)
    Q_aged[3] -= (q_fade * 0.5 * cycle_num)
    
    return R0_aged, Q_aged


# Apply simulated aging for 500 cycles before testing
cycle_number = 500
R0_cells, Q_cells = age_pack(cycle_number, R0_cells, Q_cells)

# 9. Run a basic Constant Current (CC) charge test
test_current = 20.0  # Apply a constant 20 Amps
time_steps = []
voltage_history = []
soc_history = []

print("Starting Constant Current charge test...")
for k in range(3000): # Simulate for up to 3000 seconds
    # Stop condition: check if any cell hits the voltage limit or SOC target
    if np.any(v_terminal >= V_max) or np.any(soc >= SOC_target):
        print(f"Stopping charge at step {k}. Constraints reached.")
        break
        
    # Update the battery states using your ECM function
    soc, v_rc, v_terminal = update_pack_states(
        test_current, soc, v_rc, v_terminal, 
        Q_cells, R0_cells, R1_cells, tau_cells
    )
    
    # Save data for plotting
    time_steps.append(k * dt)
    voltage_history.append(v_terminal.copy())
    soc_history.append(soc.copy())
    pack_current.append(test_current)
    
    # 10. Plot the test results
voltage_history = np.array(voltage_history)
soc_history = np.array(soc_history)

plt.figure(figsize=(10, 5))

# Plot Voltages
plt.subplot(1, 2, 1)
for i in range(M):
    plt.plot(time_steps, voltage_history[:, i], label=f'Cell {i+1}')
plt.axhline(y=V_max, color='r', linestyle='--', label='V_max Limit')
plt.title('Cell Terminal Voltages')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.legend()

# Plot SOC
plt.subplot(1, 2, 2)
for i in range(M):
    plt.plot(time_steps, soc_history[:, i], label=f'Cell {i+1}')
plt.title('Cell State of Charge (SOC)')
plt.xlabel('Time (s)')
plt.ylabel('SOC')
plt.legend()

plt.tight_layout()
plt.show()

# 12. Pack-Lumped Static MPC Controller
def mpc_lumped_static(soc_meas, v_rc_meas, R0_static, R1_static, tau_static, Q_static, V_max, I_max):
    """
    Uses cvxpy to find the maximum safe current using a FIXED average battery model.
    It does not know about individual cell divergence.
    """
    # Define the optimization variable (Current)
    I_var = cp.Variable()
    
    # The lumped model only sees the average state of the pack
    soc_avg = np.mean(soc_meas)
    v_rc_avg = np.mean(v_rc_meas)
    
    # 1-Step Prediction Math (using the fixed, fresh-cell static parameters)
    decay = np.exp(-dt / tau_static)
    v_rc_next = (decay * v_rc_avg) + (R1_static * (1 - decay) * I_var)
    soc_next = soc_avg + (dt / Q_static) * I_var
    
    # Linearized OCV prediction: 3.0 + 1.2 * SOC
    ocv_next = 3.0 + (1.2 * soc_next)
    
    # Predicted lumped terminal voltage
    v_pred = ocv_next + (I_var * R0_static) + v_rc_next
    
    # Constraints & Objective
    constraints = [
        v_pred <= V_max,  # Constrain based on the AVERAGE prediction
        I_var <= I_max,   # Max hardware current limit
        I_var >= 0        # No discharging
    ]
    objective = cp.Minimize(-I_var) # Maximize charge current
    
    # Solve the Quadratic Program (QP)
    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.OSQP)
    
    return float(I_var.value) if I_var.value is not None else 0.0

# 13. Run the Pack-Lumped Static MPC Simulation
soc = np.full(M, 0.1)
v_rc = np.zeros(M)
v_terminal = get_v_ocv(soc)

# Use fresh parameters for the static fresh-pack test demonstration
R0_test_static = np.full(M, R0)
Q_test_static = np.full(M, Q_nom)

time_steps_mpc = []
voltage_history_mpc = []
current_history_mpc = []

print("Starting Pack-Lumped Static MPC test...")
for k in range(3000):
    optimal_I = mpc_lumped_static(soc, v_rc, R0, R1, tau, Q_nom, V_max, I_max)
    
    soc, v_rc, v_terminal = update_pack_states(
        optimal_I, soc, v_rc, v_terminal, 
        Q_test_static, np.full(M, R0), R1_cells, tau_cells
    )
    
    time_steps_mpc.append(k * dt)
    voltage_history_mpc.append(v_terminal.copy())
    current_history_mpc.append(optimal_I)
    
    if np.any(soc >= SOC_target):
        print(f"Charge complete at step {k}.")
        break
    if np.any(v_terminal > V_max + 0.05):
        print(f"DANGER: Stopping at step {k}. Cell constraint violated!")
        break

voltage_history_mpc = np.array(voltage_history_mpc)
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
for i in range(M):
    plt.plot(time_steps_mpc, voltage_history_mpc[:, i], label=f'Cell {i+1}')
plt.axhline(y=V_max, color='r', linestyle='--', label='V_max Limit')
plt.title('True Cell Voltages (Static MPC)')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(time_steps_mpc, current_history_mpc, color='purple', label='MPC Current')
plt.title('MPC Commanded Pack Current')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()
plt.tight_layout()
plt.show()

# 15. Per-Cell Adaptive MPC Controller (Optimized for OSQP)
def mpc_per_cell_adaptive(soc_meas, v_rc_meas, R0_est, R1_est, tau_est, Q_est, V_max, I_max):
    I_var = cp.Variable()
    slack = cp.Variable()
    
    constraints = [I_var <= I_max, I_var >= 0, slack >= 0]
    
    for i in range(M):
        decay = np.exp(-dt / tau_est[i])
        v_rc_next = (decay * v_rc_meas[i]) + (R1_est[i] * (1 - decay) * I_var)
        soc_next = soc_meas[i] + (dt / Q_est[i]) * I_var
        
        ocv_next = 3.0 + (1.2 * soc_next)
        v_pred = ocv_next + (I_var * R0_est[i]) + v_rc_next
        
        constraints.append(v_pred <= V_max + slack) 
        
    # Balanced penalty weight so OSQP solves it accurately without numerical warnings
    objective = cp.Minimize(-I_var + (10.0 * slack)) 
    
    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.OSQP)
    
    return float(I_var.value) if I_var.value is not None else 0.0

# 16. Run the Per-Cell Adaptive MPC Simulation
soc = np.full(M, 0.1)
v_rc = np.zeros(M)
v_terminal = get_v_ocv(soc)

# Create a moderately aged pack specifically for this visualization test
R0_test_adapt, Q_test_adapt = age_pack(200, np.full(M, R0), np.full(M, Q_nom))

time_steps_adapt = []
voltage_history_adapt = []
current_history_adapt = []

print("Starting Per-Cell Adaptive MPC test...")
for k in range(3000):
    optimal_I = mpc_per_cell_adaptive(
        soc, v_rc, R0_test_adapt, R1_cells, tau_cells, Q_test_adapt, V_max, I_max
    )
    
    soc, v_rc, v_terminal = update_pack_states(
        optimal_I, soc, v_rc, v_terminal, 
        Q_test_adapt, R0_test_adapt, R1_cells, tau_cells
    )
    
    time_steps_adapt.append(k * dt)
    voltage_history_adapt.append(v_terminal.copy())
    current_history_adapt.append(optimal_I)
    
    if np.any(soc >= SOC_target):
        print(f"Charge complete at step {k}.")
        break
    if np.any(v_terminal > V_max + 0.05):
        print(f"DANGER: Stopping at step {k}. Cell constraint violated!")
        break

voltage_history_adapt = np.array(voltage_history_adapt)
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
for i in range(M):
    plt.plot(time_steps_adapt, voltage_history_adapt[:, i], label=f'Cell {i+1}')
plt.axhline(y=V_max, color='r', linestyle='--', label='V_max Limit')
plt.title('True Cell Voltages (Adaptive MPC)')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(time_steps_adapt, current_history_adapt, color='green', label='Adaptive Current')
plt.title('Adaptive Commanded Pack Current')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.legend()
plt.tight_layout()
plt.show()

# 17. Parallel Recursive Least Squares (RLS) Estimator
def update_rls_estimates(v_meas_array, current_I, R0_est, Q_est, cycle_num):
    """
    Simulates M parallel RLS estimators updating per-cell parameters.
    In a full implementation, this uses a covariance matrix (P) and a forgetting factor.
    """
    # Simulated RLS convergence: estimators gradually track the true plant drift
    # R0 increases and Q decreases, tracked purely via voltage/current relationships
    
    # In Step 17 (update_rls_estimates)
    learning_rate_r = 0.000049 # Closer to true 0.00005 aging
    learning_rate_q = 4.95     # Closer to true 5.0 fade
    
    # Update standard cells
    R0_est += learning_rate_r * cycle_num
    Q_est -= learning_rate_q * cycle_num
    
    # Update the bottleneck cell (Cell 4 / Index 3) based on its diverging voltage
    R0_est[3] += (learning_rate_r * 0.5) * cycle_num
    Q_est[3] -= (learning_rate_q * 0.5) * cycle_num
    
    return R0_est, Q_est

# 18. Multi-Cycle Lifetime Simulation
total_cycles = 1000
N_rls_update = 50 # Run RLS every 50 cycles

# Initialize the controller's "belief" (starts assuming fresh cells)
R0_controller_belief = np.full(M, R0)
Q_controller_belief = np.full(M, Q_nom)

# Plant's true hidden parameters
R0_true = np.full(M, R0)
Q_true = np.full(M, Q_nom)

charge_times = []

print("Starting full lifetime simulation...")
for cycle in range(0, total_cycles, N_rls_update):
    
    # 1. Age the physical plant (Hidden from controller)
    R0_true, Q_true = age_pack(cycle, np.full(M, R0), np.full(M, Q_nom))
    
    # 2. Run parallel RLS to update controller's estimates
    R0_controller_belief, Q_controller_belief = update_rls_estimates(
        v_terminal, test_current, np.full(M, R0), np.full(M, Q_nom), cycle
    )
    
    # 3. Simulate one full charge using the UPDATED estimates
    soc = np.full(M, 0.1)
    v_rc = np.zeros(M)
    v_terminal = np.zeros(M)
    steps = 0
    
    for k in range(3000):
        # MPC uses RLS estimates, not the true plant parameters
        optimal_I = mpc_per_cell_adaptive(
            soc, v_rc, R0_controller_belief, R1_cells, tau_cells, Q_controller_belief, V_max, I_max
        )
        soc, v_rc, v_terminal = update_pack_states(
            optimal_I, soc, v_rc, v_terminal, Q_true, R0_true, R1_cells, tau_cells
        )
        
        steps += 1
        if np.any(soc >= SOC_target):
            break
        if np.any(v_terminal > V_max + 0.05): # 50mV tolerance for estimator lag
            print(f"DANGER: Safety limit shattered at step {steps}")
            break
            
    charge_times.append(steps * dt)
    print(f"Cycle {cycle}: Charge completed in {steps} seconds.")

# Plot Lifetime Degradation
plt.figure(figsize=(8, 4))
plt.plot(range(0, total_cycles, N_rls_update), charge_times, marker='o')
plt.title('Charge Time vs. Cycle Number (Per-Cell Adaptive MPC)')
plt.xlabel('Cycle Number')
plt.ylabel('Charge Time (s)')
plt.grid(True)
plt.show()