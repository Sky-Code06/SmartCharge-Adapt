---
marp: true
theme: gaia
_class: lead
paginate: true
backgroundColor: #0f172a
color: #f8fafc
style: |
  section {
    font-family: 'Plus Jakarta Sans', sans-serif;
  }
  h1, h2, h3 {
    color: #38bdf8;
  }
  table {
    font-size: 0.8rem;
  }
  footer {
    font-size: 0.75rem;
    color: #64748b;
  }
---

# **SmartCharge-Adapt**
### Per-Cell Online Parameter-Adaptive MPC for Degradation-Aware Fast Charging of Multi-Cell Li-ion Battery Packs

**Control Systems Course Project**  
*M Series Cells &bull; Online Parallel RLS &bull; Constrained MPC Optimization*

---

## **1. The Fast-Charging Dilemma**

- **Shared Current**: In series packs, cells share identical current $I(k)$.
- **Cell-to-Cell Divergence**: Manufacturing tolerances and localized thermal gradients cause individual cells to age 30%–50% faster.
- **The Weakest Link**: The pack's safe charging current is strictly bottlenecked by its highest-resistance, lowest-capacity cell.
- **Strict Electrochemical Limit**: $V_i(k) \le 4.200\,\text{V}$. Exceeding this causes lithium plating, toxic gas release, or explosive thermal runaway.

---

## **2. Existing Approaches & Their Fatal Flaws**

1. **Pack-Lumped Static MPC** (Industry Commonplace)
   - Treats the pack as one averaged, fresh cell.
   - *Result*: Pushes aged cells past **$8.2\,\text{V}$ (+3,986 mV overshoot)** $\implies$ **Catastrophic failure**.

2. **Pack-Lumped Adaptive MPC** (Average Adaptive Baseline)
   - Updates pack-average resistance and capacity online.
   - *Result*: Protects the mean, but leaves the outlier cell overshooting by **$+393\,\text{mV}$** $\implies$ **Severe lithium plating**.

3. **Academic Electrochemical NMPC**
   - Relies on internal states (e.g. anode overpotential).
   - *Result*: Impossible to implement with standard commercial BMS sensors.

---

## **3. The SmartCharge-Adapt Solution**

An online adaptive control framework relying **strictly on standard BMS sensors**:

1. **Parallel Online RLS Estimators**:
   - $M$ independent estimators identify internal resistance ($R_{0,i}$) and capacity ($Q_i$) per cell in real time.
   - Requires only cell terminal voltage $V_i(k)$ and current $I(k)$.
2. **Granular MPC Optimization**:
   - Maximizes charging speed subject to $M$ individual constraints:
     $$\min_{I(k)} -I(k) \quad \text{s.t.} \quad V_{\text{pred},i}(k+1) \le 4.2\,\text{V} \quad \forall i=1..M$$
3. **Dynamic Bottleneck Detection**:
   - Automatically tracks $\arg\min_i I_{\text{upper},i}$ and shields the weakest link.

---

## **4. Charging Time vs. Lifetime (Figure 1)**

![width:780px](../results/fig1_charging_time_vs_cycle.png)

- **Static Controller's Illusion**: Charge time drops from 218s to 65s because it drives an aged pack past 8V.
- **Proposed Controller**: Safely throttles current as Cell 4 ages, rising to ~685s.
- **The Cycle ~700 Dip Explained**: Beyond cycle 700, capacity fade shrinks the cell's usable capacity ($Q \approx 1.1\,\text{Ah}$ vs $3.2\,\text{Ah}$). Less charge is required to hit 90% SOC, so charge time turns downward naturally.

---

## **5. Mid-Life Cell Voltage Profiles at Cycle 500 (Figure 2)**

![width:950px](../results/fig2_voltage_trajectories_cycle500.png)

- **(a) Static MPC**: Cell 4 shoots to **6.16 V** (+1,955 mV above safety limit).
- **(b) Lumped Adaptive**: Cell 4 reaches **4.50 V** (+304 mV above safety limit).
- **(c) Proposed Per-Cell Adaptive**: Cell 4 is **clamped perfectly at 4.200 V** with **0.0 mV overshoot**.

---

## **6. Constraint Violations Across Lifetime (Figure 3)**

![width:900px](../results/fig3_constraint_violations_vs_cycle.png)

- **Static MPC**: Overshoot reaches **+3,986 mV** ($>8.2\,\text{V}$ terminal voltage) $\implies$ total destruction.
- **Lumped Adaptive MPC**: Persistently overshoots by **+300 to +393 mV**.
- **Proposed Per-Cell Adaptive MPC**: **0.0 mV overshoot** and **0 seconds of violation** across all 1,000 cycles.

---

## **7. Online Parameter Identification (Figure 4)**

![width:900px](../results/fig4_rls_parameter_tracking.png)

- **Tracking Accuracy**: Less than **0.1% parameter estimation error**.
- Solid lines (ground truth) and dashed markers (RLS estimates) overlap perfectly.
- Non-invasive identification without internal electrochemical probes.

---

## **8. Dynamic Bottleneck Identification (Figure 5)**

![width:750px](../results/fig5_bottleneck_cell_identification.png)

- **100.0% Detection Accuracy**: From Cycle 50 to 1000, the controller correctly pinpoints Cell 4 as the bottleneck in every single cycle.
- Baselines possess 0% visibility into individual cell degradation.

---

## **9. Complete Performance Dashboard (Figure 6)**

![width:950px](../results/simulation_summary_dashboard.png)

---

## **10. Executive Comparison Table**

| Metric | Pack-Lumped Static | Pack-Lumped Adaptive | Per-Cell Adaptive (Ours) |
| :--- | :---: | :---: | :---: |
| **Model Scope** | Fixed fresh average | Adaptive average | **Adaptive per-cell** |
| **Constraints** | 1 (Average) | 1 (Average) | **M (Every cell)** |
| **Max Overshoot (Cycle 500)** | +1,955.3 mV (Explosive) | +304.0 mV (Plating) | **0.0 mV (Safe clamp)** |
| **Max Overshoot (Cycle 1000)**| +3,986.3 mV (>8.2 V) | +393.3 mV (Hazardous) | **0.0 mV (Safe clamp)** |
| **Lifetime Violation Duration** | 2,415 seconds | 6,284 seconds | **0 seconds** |
| **Bottleneck Accuracy** | 0% (Blind) | 0% (Blind) | **100.0%** |
| **Hardware Overhead** | None | None | **None (Pure firmware)**|

---

## **11. Future Scope: Indian EV Operational Context**

- **Ambient Heat & Spatial Gradients**:
  - In Indian EV conditions (ambient $>45^\circ\text{C}$), cells near the pack core run $5^\circ\text{C}$ to $15^\circ\text{C}$ hotter.
  - Arrhenius SEI growth accelerates degradation by 30%–50%, creating the bottleneck cells studied here.
- **Coupled Electro-Thermal Model (ET-ECM)**:
  - 2-state core/surface thermal model ($T_{c,i}, T_{s,i}$) with Joule heating ($I^2 R_0$).
  - Dual-constraint MPC: $V_i(k) \le 4.2\,\text{V}$ and $T_{c,i}(k) \le 45^\circ\text{C}$ $\forall i$.
  - Dual EKF/RLS to estimate temperature and resistance simultaneously online.

---

## **12. Conclusion & Key Takeaways**

- **Absolute Safety**: Completely eliminates the 4,000 mV explosive overshoot of static models.
- **Granular Protection**: Protects the weakest link without unnecessarily penalizing the whole pack.
- **Zero Hardware Cost**: Uses standard BMS sensors ($V_i, I$) and solves in $<1\,\text{ms}$ on microcontrollers.
- **Ready for Deployment**: Bridges the gap between complex electrochemical theory and real-world EV charging systems.

---

# **Thank You!**
### Questions & Discussion
