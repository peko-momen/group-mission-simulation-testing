# Challenge 1: Adaptive Pure Pursuit Benchmarking & Stress-Testing

## Core Goal
Act as an independent Validation and Verification (V&V) squad. You are handed an "Adaptive Pure Pursuit" control node from the Navigation Team. Your objective is not to optimize, rewrite, or patch their code. Your job is to independently discover its mathematical breaking points, map its operational boundaries under severe stress, and build an automated script to report its structural vulnerabilities.

---

## Responsibilities 
* **Test:** Design the multi-dimensional parameter sweep matrices and programmatically execute the test configurations.
* **Telemetry & Logging:** Build the auxiliary ROS 2 monitoring node or data collection scripts to extract live state updates from active topics (e.g., `/odom`, `/cmd_vel`).
* **Report & Forensic:** Consolidate the discovered failure modes, map the pass/fail boundaries, and formulate the underlying engineering reasoning behind why specific configurations failed.

---

## Technical Requirements
You must design and implement a custom testing script or dedicated evaluation node in ROS 2 (Python/C++) based entirely on your own engineering judgment, criteria, and system logic.

### 1. Custom Benchmarking & Test Matrix Design
You are responsible for determining how to systematically stress this control module. You must establish a clear, multi-dimensional parameter sweep matrix. You must decide:
* Which specific control variables (e.g., linear velocities $v$, lookahead distances $L_t$), path geometries (e.g., chicanes, sharp curves, sudden offsets), or dynamic factors to alter during your testing sweeps.
* How to systematically scale the intensity or difficulty of these test sweeps to expose the exact thresholds where control stability degrades.

### 2. Independent Corner-Case Conception & Implementation
Based entirely on your own engineering thinking, your team must brainstorm, select, and intentionally implement distinct physical or environmental corner cases within the simulation. 
* You must decide what constitutes a valid, non-ideal "corner case" capable of breaking an adaptive tracking controller.
* You must programmatically inject these conditions into your simulation test loop to observe, record, and map how the control loop responds to severe system degradation.

### 3. Quantitative Criteria & Evaluation
You must define your own quantitative evaluation metrics. Your automated testing system must actively monitor the live system states to objectively declare whether a specific configuration run is a **Pass** or a **Fail**. You must mathematically define your own numerical boundaries for tracking errors, path deviations, settling times, or steering oscillations.

---

## Deliverables
At the end of this challenge, each team must submit a unified workspace containing:
1. **The Automation Tool:** A runnable testing script or ROS 2 node that executes your custom benchmarks and stress parameter sweeps automatically without manual GUI intervention.
2. **The Stress-Testing Results:** Raw logged output data (.csv or .json) captured by your script documenting the state profiles of each run.
3. **The Validation Report:** A formal engineering document listing:
   * The custom evaluation criteria and corner cases your team invented.
   * A clear verification ledger mapping out which specific corner cases passed and which ones failed.
   * (Optional) Your technical forensic reasoning explaining exactly *why* the controller failed to clear specific non-passed testing blocks.