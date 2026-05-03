# Learn-It-All: Full-Stack Embodied AI Quadruped Robot

<p align="center">
  <b>A low-cost full-stack quadruped robot project for embodied AI beginners.</b>
</p>

<p align="center">
  Mechanical Design · Isaac Lab · MuJoCo · Reinforcement Learning · IK Control · Sim-to-Real Deployment
</p>

<p align="center">
  <a href="#overview">Overview</a> ·
  <a href="#demo">Demo</a> ·
  <a href="#pipeline">Pipeline</a> ·
  <a href="#repositories">Repositories</a> ·
  <a href="#roadmap">Roadmap</a> ·
  <a href="#contact">Contact</a>
</p>

---

## Overview

Embodied AI is becoming one of the most important directions in robotics and artificial intelligence. However, the entry barrier is still high for beginners because a complete embodied AI workflow usually involves mechanical design, simulation, reinforcement learning, controller design, and real-world deployment.

This project aims to provide a **low-cost full-stack quadruped robot platform**. With a hardware cost of around **1500 RMB**, beginners can experience the complete development pipeline of an embodied AI quadruped robot.

The project covers:

- Mechanical design and URDF export
- Isaac Lab simulation and reinforcement learning training
- MuJoCo simulation and sim-to-sim validation
- Locomotion policy training and ONNX export
- Real-world deployment on Jetson Nano or Raspberry Pi
- IK controller validation on the real robot

This project is suitable for:

- Embodied AI beginners
- Undergraduate innovation and entrepreneurship projects
- Graduation design projects
- Robotics education and research practice
- Students who want to learn simulation, reinforcement learning, and deployment through a real robot system

Unlike a minimal embodied AI demo, this project also provides an additional **IK controller pipeline**, so users can learn both learning-based control and model/controller-based robot motion control.

---

## Demo

### Reinforcement Learning Policy Deployment

https://github.com/user-attachments/assets/0e937bb2-8c6b-4ac0-97f3-5103152a0ea5

https://github.com/user-attachments/assets/c23fae7b-19d8-4e9f-8c7c-3008be9b9506

### IK Controller on Real Robot

https://github.com/user-attachments/assets/35e0f9d1-ff54-483b-a3b3-77c9c0c3400d

---

## Pipeline

The full-stack development pipeline of this project is shown below.

<p align="center">
  <img width="816" alt="pipeline" src="https://github.com/user-attachments/assets/0eb33d81-4255-4275-a9ac-46993806f269" />
</p>

The overall workflow includes mechanical design, robot model conversion, simulation environment construction, reinforcement learning training, policy export, and real-world deployment.

---

## Repositories

This project is organized as multiple repositories. Each repository focuses on one part of the full-stack quadruped robot development pipeline.

| Repository | Description | Status |
|---|---|---|
| [Learn-It-All-Full-Stack-EmbodiedAI-Quadruped-Robot](https://github.com/12311112/Learn-It-All-Full-Stack-EmbodiedAI-Quadruped-Robot) | Main project page and full-stack overview | In progress |
| [Learn-It-All-Mechanical-Design](https://github.com/12311112/Learn-It-All-Mechanical-Design) | SolidWorks mechanical design, URDF export, model checking, and bill of materials | In progress |
| [Learn-It-All-rl-isaaclab](https://github.com/12311112/Learn-It-All-rl-isaaclab) | Isaac Lab simulation environment, USD conversion, MDP design, and RL training | In progress |
| [Learn-It-All-rl-mujuco](https://github.com/12311112/Learn-It-All-rl-mujuco) | MuJoCo model conversion, locomotion training, sim-to-sim validation, and ONNX export | In progress |
| [Learn-It-All-deployment-sim2real](https://github.com/12311112/Learn-It-All-deployment-sim2real) | Real-world deployment, policy inference, hardware interface, and IK controller validation | In progress |

---

## Main Work

### 1. Mechanical Design

The quadruped robot body is designed in SolidWorks. The mechanical repository provides CAD files, URDF export files, model checking instructions, and the bill of materials.

The exported URDF model can be checked using:

[https://viewer.robotsfan.com/](https://viewer.robotsfan.com/)

This step ensures that the robot structure, joint configuration, and exported model are correct before simulation.

---

### 2. Isaac Lab Simulation and Training

The URDF model is converted into a USD model for Isaac Lab. Based on the robot model, we build the Markov decision process, define the observation space, action space, reward function, and physical parameters, and then perform parallel reinforcement learning training in Isaac Lab.

This part is used to learn the standard Isaac Lab workflow for embodied AI and legged robot locomotion training.

---

### 3. MuJoCo Simulation and Sim-to-Sim Validation

The URDF model is also converted into a MuJoCo XML scene description file. In MuJoCo, we build the locomotion task, train the policy, and export the trained policy as an ONNX model.

This part provides another mainstream simulator workflow and supports sim-to-sim comparison between Isaac Lab and MuJoCo.

---

### 4. Real-World Deployment

The trained reinforcement learning policy is deployed on Jetson Nano. Raspberry Pi can also be used because the inference computation requirement is not high.

The deployment repository includes:

- RL policy inference
- Hardware communication
- Real robot control
- IK controller validation
- Sim-to-real experiment records

---

## Why This Project?

The goal of this project is not only to build a quadruped robot, but also to provide a complete learning path for embodied AI beginners.

We hope users can learn:

- How to design and export a robot model
- How to build simulation environments
- How to train locomotion policies
- How to use Isaac Lab and MuJoCo
- How to export and deploy policies
- How to validate both RL policies and IK controllers on real hardware

The core idea of this project is:

> Low cost, full experience, and reproducible learning.

---

## Roadmap

- [x] Build the full-stack project structure
- [x] Complete the mechanical design
- [x] Export and check the URDF model
- [x] Build the Isaac Lab training pipeline
- [x] Build the MuJoCo training pipeline
- [x] Validate RL policy deployment on real hardware
- [x] Validate IK controller on real hardware
- [ ] Improve documentation for each sub-repository
- [ ] Add detailed assembly tutorial
- [ ] Add more training tutorials
- [ ] Add troubleshooting documents
- [ ] Build a community contribution guide

---

## Community

We aim to build an open and beginner-friendly embodied AI robotics community.

If you have questions, suggestions, or want to contribute, you are welcome to open an issue on GitHub.

You can also contact us by email.

---

## Contact

Email: 1790815185@qq.com

---








