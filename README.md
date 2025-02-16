# tramola
Tramola Source Code Repository

# To use python 3
catkin_make -DPYTHON_EXECUTABLE=/home/tramola/vision/bin/python3.8


Tuning PID (Proportional-Integral-Derivative) constants properly depends on your application and system response. Here’s a structured approach to tuning:

---

### **1. Understanding PID Constants**
- **`Kp` (Proportional Gain):** Determines how aggressively the controller reacts to the current error.
- **`Ki` (Integral Gain):** Accumulates past errors to correct steady-state errors.
- **`Kd` (Derivative Gain):** Predicts future errors to smooth out the response.

---

### **2. Manual Tuning Method**
#### **Step 1: Start with Only `Kp`**
- Set `Ki = 0` and `Kd = 0`
- Increase `Kp` until you get oscillations (fast response but unstable)
- Reduce `Kp` slightly for stability

#### **Step 2: Tune `Kd` (Damping)**
- Increase `Kd` gradually to reduce oscillations
- Too much `Kd` will make the system slow to respond

#### **Step 3: Tune `Ki` (Eliminate Steady-State Error)**
- Increase `Ki` slowly to correct any steady-state error
- Too much `Ki` can cause oscillations or instability

---

### **3. Ziegler-Nichols Method (More Systematic)**
1. Set `Ki = 0`, `Kd = 0`
2. Increase `Kp` until the system oscillates with a constant amplitude (critical gain `Kcr`)
3. Measure the oscillation period `Pcr`
4. Set the gains based on this table:

| Type | `Kp` | `Ki` | `Kd` |
|------|------|------|------|
| P    | 0.5 × Kcr | 0 | 0 |
| PI   | 0.45 × Kcr | 1.2 × Kp / Pcr | 0 |
| PID  | 0.6 × Kcr | 2 × Kp / Pcr | Kp × Pcr / 8 |

---

### **4. Fine-Tuning Tips**
- **System Too Slow?** → Increase `Kp`
- **Overshooting?** → Increase `Kd`, decrease `Kp`
- **Steady-State Error?** → Increase `Ki`
- **Oscillations?** → Decrease `Kp` or increase `Kd`

Would you like to tune PID for a specific system (e.g., motor, drone, temperature control)?