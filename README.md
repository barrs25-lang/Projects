# Overview
Repository to house my personal projects.

---

## Projects

### [Autodrive Simulator CICD](Autodrive%20Simulator%20CICD)
MATLAB/Simulink autonomous vehicle controller simulator with a full CI/CD pipeline (Jenkins, GitLab CI, Azure Pipelines). Uses a clothoid toolbox for smooth path generation and evaluates tracking accuracy, comfort, and stability across real-world and synthetic tracks (VIR, Pikes Peak, Figure-8). See [README_MATLAB.md](Autodrive%20Simulator%20CICD/README_MATLAB.md) for details.

### [Adaptive Control HMMWV](Adaptive%20Control%20HMMWV)
Final project implementing and testing an adaptive control law on a HMMWV vehicle model, co-simulated in the Chrono physics engine via PyChrono. See [NoteOnProject.md](Adaptive%20Control%20HMMWV/NoteOnProject.md) for setup instructions.

### [Ball Beam w Disturbance Rejection](Ball%20Beam%20w%20Disturbance%20Rejection)
Controller design for the classic ball-and-beam system with disturbance rejection. *(Write-up in progress.)*

---

## Cloning the Repo
To clone this repo and have access to all of the files in your local directory, perform the following commands in a terminal window within the directory you would like to clone to:
```
git clone https://github.com/barrs25-lang/Projects.git
git pull
git lfs pull
```
