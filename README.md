# 7-DOF Full-Car Suspension Simulation

This repository contains the MATLAB implementation and dynamic analysis of a 7-DOF full-car suspension model subjected to harmonic and random road excitations.

The model represents the vertical dynamics of a vehicle through the heave, roll, and pitch motions of the sprung mass and the vertical motions of the four unsprung masses.

## Model Description

The full-car model includes the following degrees of freedom:

1. Vehicle-body heave motion
2. Vehicle-body roll motion
3. Vehicle-body pitch motion
4. Front-left unsprung-mass motion
5. Front-right unsprung-mass motion
6. Rear-left unsprung-mass motion
7. Rear-right unsprung-mass motion

The second-order dynamic equations are represented using 14 first-order state variables and are numerically integrated in MATLAB.

## Repository Sections

### Harmonic Road Excitation

This section investigates the response of the vehicle suspension system under a multi-harmonic road profile.

The analyses include:

- Full-car suspension simulation
- Heave, roll, and pitch responses
- Unsprung-mass displacements and accelerations
- Tire and suspension deflections
- Tire vertical forces
- Suspension forces
- Suspension-stiffness comparison
- Damping-ratio comparison
- Vehicle-speed comparison
- Time-domain response analysis
- Frequency-domain analysis
- Phase-plane diagrams

### Random Road Excitation

This section investigates the suspension response under randomly generated road profiles.

The analyses include:

- Random road-profile generation
- Different road-quality classes
- Road displacement and velocity calculation
- Vehicle response under stochastic excitation
- Time-domain analysis
- FFT and power spectral density analysis
- Comparison of dominant vibration frequencies

## Numerical Method

The equations of motion are solved using a fifth-order Runge–Kutta integration scheme implemented directly in MATLAB.

The state vector contains:

- Roll and pitch rates
- Vertical velocity of the vehicle body
- Vertical velocities of the four unsprung masses
- Roll and pitch angles
- Vertical displacements of the four unsprung masses
- Vertical displacement of the vehicle body

## Parameter Studies

The repository includes studies of the effects of:

- Suspension stiffness
- Suspension damping ratio
- Vehicle longitudinal speed
- Harmonic road-input parameters
- Random road class
- Tire stiffness and damping

## Signal Processing

The simulation outputs are evaluated using several time- and frequency-domain tools, including:

- Fast Fourier Transform
- Periodogram
- Power spectral density
- Filtered and unfiltered signal comparison
- Amplitude-spectrum analysis
- Phase diagrams

## Repository Structure

```text
7dof-full-car-suspension-simulation/
├── Harmonic/
│   ├── main/
│   ├── parameter-studies/
│   ├── signal-processing/
│   ├── utilities/
│   └── README.md
├── Random/
│   ├── main/
│   ├── road-profile/
│   ├── signal-processing/
│   ├── data/
│   └── README.md
└── README.md
