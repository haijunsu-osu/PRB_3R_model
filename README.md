# Pseudo-Rigid Body 3R Model Simulator

This web application is a high-precision simulator for partially compliant four-bar mechanisms, specifically implementing the **Pseudo-Rigid Body (PRB) 3R model**. 

## Purpose

The PRB 3R model is used to accurately predict the large deflection behavior of cantilever beams subjected to tip loads. By modeling a flexible beam as a series of rigid segments connected by torsion springs, the complex non-linear differential equations of beam deflection are transformed into a solvable kinematic problem.

This simulator allows engineers and researchers to:
- **Visualize** the motion of a compliant linkage in real-time.
- **Analyze** the input torque required to drive the mechanism through its full cycle.
- **Calculate** the potential energy stored in the flexible segments.
- **Optimize** mechanism dimensions and material properties for specific mechanical outputs.

## Launch App

[**Launch Pseudo-Rigid Body 3R Model Simulator**](https://ai.studio/apps/721192f9-4545-4477-b929-3c9255b928a2?fullscreenApplet=true)

## Key Features

- **Real-time Kinematic Solver**: Uses numerical methods to solve the non-linear loop closure equations of the 3R model.
- **Dynamic Analysis**: Live plots of crank torque and potential energy relative to the initial assembly state.
- **Interactive Parameters**: Adjust beam length, crank radius, material properties (Young's Modulus), and PRB characteristic parameters ($\gamma$, $K_\Theta$).
- **Technical Visualization**: A clean, CAD-like interface with zoom and pan support for detailed inspection of the mechanism's path.

## Citation

If you use this model or simulator in your research, please cite the original paper:

**Su, H. (January 7, 2009). "A Pseudorigid-Body 3R Model for Determining Large Deflection of Cantilever Beams Subject to Tip Loads." ASME. J. Mechanisms Robotics. May 2009; 1(2): 021008. [DOI: 10.1115/1.3046148](https://doi.org/10.1115/1.3046148)**
