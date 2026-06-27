# Challenge 2: Mini ERC-BARN Automated Testing Harness

## Core Goal
Transition from testing an isolated module to architecting a system-level automated benchmarking framework. Inspired by the Benchmarking Autonomous Robot Navigation (BARN) challenge methodology, you will build an automated evaluation harness tailored specifically for the European Rover Challenge (ERC) planetary navigation task.

---

## Phase A: Literature Review & Logic Study (60 Minutes)
Before designing your system architecture or writing infrastructure code, your team has exactly 1 hour to study the core concepts, logic, execution methodology, and evaluation criteria of the BARN Challenge using the following official links:
* **Repository Architecture:** https://github.com/Daffan/the-barn-challenge
* **Evaluation & Methodology Principles:** https://people.cs.gmu.edu/~xiao/Research/BARN_Challenge/BARN_Challenge26.html

Your goal during this hour is to absorb how a standardized benchmarking framework functions, how it structures its headless automation loops, and how it handles evaluation logic cleanly.

---

## Phase B: Implementation Requirements
Using your custom rover model and baseline environments developed during the Solo Mission, you will implement an automated ERC testing framework[cite: 1]. 

To isolate and evaluate the reliability of your benchmarking harness without being blocked by an incomplete autonomous navigation stack, **you must implement a Teleop Node to act as a placeholder for the autonomous system agent.** By driving the rover with intentionally distinct control styles (e.g., smooth vs. erratic driving profiles), you must prove that your benchmarking system successfully captures, scores, and outputs different evaluation results reflective of that driving behavior.

### 1. Multi-Tiered Mars Yard Environments
Using your pre-made assets, you must configure 3 distinct simulation environments representing 3 different levels of difficulty mimicking a planetary Mars yard. The choice of obstacle density, terrain hazards, and chokepoints is entirely up to your team (simple geometric obstacles are completely acceptable to prove the concept).

### 2. The Sequential ERC Scoring & Automation Engine
Your framework must strictly enforce and monitor the rules of the ERC navigation task:
* **One-Command Launch:** A single master terminal command must spin up the simulation headless, load the specific difficulty level, spawn the rover, and inject the 4 critical target waypoints.
* **Strict Sequential Checkpoint Tracking:** The rover must traverse the 4 waypoints in exact sequential order: 
  $$\text{Start} \rightarrow \text{Waypoint 1} \rightarrow \text{Waypoint 2} \rightarrow \text{Waypoint 3} \rightarrow \text{Waypoint 4}$$
  Your scoring engine must validate this sequence in real time. If a checkpoint is bypassed out of order, it cannot register as a valid hit.
* **Active Crash Termination:** Your harness must continuously monitor the simulation state for collisions. **The instant a crash is detected, the framework must programmatically close the entire simulation immediately, kill the active ROS 2 graph, and assign a final performance score of exactly ZERO.**

### 3. Comprehensive Terminal & Output Logging
Upon the termination of any test run (whether due to success, timeout, or a catastrophic crash), your framework must print a comprehensive, clear report directly to the terminal window and export it to a local log file. The output must explicitly include:

```text
============================================================
ERC NAV BENCHMARK RUN TERMINATION REPORT
============================================================
Test Scenario ID             : [Level_1 / Level_2 / Level_3]
Final Run Status             : [Target Reached / Catastrophic Collision / Timeout]
Checkpoints Cleared          : [X / 4] 
Sequence Compliance Status   : [PASSED / FAILED_VIOLATION]
Total Traversal Latency      : [XX.XX seconds]
Actual Distance Traveled     : [XX.XX meters]
Normalized Performance Score : [X.XX] (Normalized against optimal path difficulty)
Driving Smoothness Index     : [High / Moderate / Poor] (Based on teleop input variance)
============================================================