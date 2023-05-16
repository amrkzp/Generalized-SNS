# Motion Control of Redundant Robots with Generalized Hard Inequality Constraints
Repository for the implementation of motion control algorithms for kinematically redundant robots under generalized hard inequality constraints. 

## Description
The algorithms presented in this repository offer efficient solutions for addressing redundancy resolution in velocity and acceleration commands while considering joint and Cartesian inequality constraints simultaneously. They provide real-time constraint customization without the need for parameter tuning. These algorithms are generalized versions of the Saturation in the Null Space (SNS) method, incorporating both joint and task-space constraints.

A significant advantage of these algorithms is their faster run-time execution compared to state-of-the-art local Quadratic Programming (QP) solvers. This feature is particularly beneficial for real-time applications involving redundant robots.

Simulation results are included in this repository to demonstrate the effectiveness of these algorithms across different robotic systems. 

## Getting Started
The codes in this repository are written in MATLAB. To get started:

1. Clone or download this repository to your local machine.
2. Open MATLAB and navigate to the downloaded repository folder.
3. Run the [`Example_Vel_Planar_6DOF_Robot.m`](Example_Vel_Planar_6DOF_Robot.m) script to execute the example for a planar 6-degree-of-freedom robot at velocity-level control.

## Todo
- [x] Add the algorithms for Velocity-level control.
- [ ] Add the algorithms for acceleration-level control.
- [ ] Add more simulation examples with different robot models.
- [ ] Add Python scripts.
- [ ] Enhance documentation for better code understanding.

## Publications
Here are some of our publications related to this project:

1. Kazemipour, A., Khatib, M., Al Khudir, K., Gaz, C. and De Luca, A., 2022. "Kinematic control of redundant robots with online handling of variable generalized hard constraints," *IEEE Robotics and Automation Letters*, 7(4), pp.9279-9286. [(Link)](https://doi.org/10.1109/LRA.2022.3190832)
2. A. Kazemipour, M. Khatib, K. Al Khudir, A. De Luca, "Motion control of redundant robots with generalised inequality constraints," *3rd Italian Conference on Robotics and Intelligent Machines*, Rome, ITA, pp. 138-140, 2021. [(Link)](https://zenodo.org/record/6367968#.ZGN90XZBwQ8)

## Help
If you encounter any problems or issues with the code, please raise a new issue with a description of the problem, the steps to reproduce it, and the expected behavior.

## Author
This project was developed as part of my Master's thesis in Control Engineering at the Sapienza Università di Roma.

* Amirhossein Kazemipour: PhD Student at ETH Zurich, [LinkedIn](https://www.linkedin.com/in/amrkzp), [Email](mailto:akazemi@ethz.ch), [Scholar](https://scholar.google.com/citations?user=RnD-rFcAAAAJ&hl=en)

For any questions or concerns, feel free to reach out.

## Acknowledgments
Thanks to [Prof. Alessandro De Luca](http://www.diag.uniroma1.it/deluca/), [Dr. Maram Khatib](https://www.linkedin.com/in/maram-khatib-a1b72815/), [Dr. Khaled Al Khudir](https://uk.linkedin.com/in/khaled-al-khudir-242ab65b), and [Dr. Claudio Gaz](https://www.kingston.ac.uk/staff/profile/dr-claudio-gaz-1442/) for their support and guidance throughout the research project.


## License
Shield: [![CC BY 4.0][cc-by-shield]][cc-by]
This work is licensed under a
[Creative Commons Attribution 4.0 International License][cc-by].

[![CC BY 4.0][cc-by-image]][cc-by]

[cc-by]: http://creativecommons.org/licenses/by/4.0/
[cc-by-image]: https://i.creativecommons.org/l/by/4.0/88x31.png
[cc-by-shield]: https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg
