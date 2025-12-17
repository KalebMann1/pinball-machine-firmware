# Pinball Machine Firmware

This repository contains the firmware experiments and iterations for my custom
embedded pinball machine project. The system is built around an Arduino-style
microcontroller (ESP32) and drives real pinball hardware: flippers, bumpers,
switches, and scoring logic.

The sketches in this repo represent different stages of the project:
core game loop development, slot-machine mini-game logic, and early sound tests.

---

## Project Overview

The goal of this project is to create a **fully custom pinball machine** with:

- Real-time scoring and game state management
- Flippers, bumpers, standup targets, spinner, and rollover switches
- Mini-games (e.g., slot machine, roulette / bonus logic)
- Audio feedback through a sound module
- Visual feedback through LED / display hardware (MAX7219 matrix in later stages)

The firmware is written in C/C++ using the Arduino toolchain for the ESP32.

---

## Repository Structure

- `Pinballmachinelogic_V3/`  
  Latest version of the main game logic sketch. Handles core game state,
  scoring, and integration with multiple playfield elements.

- `PinballMachineV2/`  
  Earlier iteration of the game logic used while tuning rules,
  scoring values, and switch handling.

- `SlotMachineManager/`  
  Focused sketch for the **slot-machine mini-game**, including:
  - Random reel selection
  - Payout logic for 2-of-a-kind and 3-of-a-kind
  - Integration hooks for the main scoring system

- `SoundTesting/`  
  Small test sketch used to exercise the sound module (DFPlayer) and verify:
  - Track triggering for specific events
  - Timing and overlap behavior
  - Wiring and signal integrity

---

## Key Features (Firmware)

- **Event-driven game loop**  
  Reads inputs from playfield switches and updates game state and score
  in response (bumper hits, target hits, drains, etc.).

- **Mini-game logic**  
  Slot-machine mechanic that awards bonus points based on symbol matches,
  with adjustable payout rules.

- **Hardware integration**  
  Designed to interface with:
  - Solenoid drivers for flippers and bumpers
  - Switch matrix / IO expanders for targets and rollovers
  - LED / display drivers for score and effects
  - DFPlayer-style audio module for sound effects

---

## Development Notes

These sketches were created while iterating on:

- Game rules and scoring balance
- Handling mechanical switch bounce and noisy signals
- Separating responsibilities into “manager” modules
  (e.g., SlotMachineManager, Sound manager in later versions)

Later work consolidated these ideas into a more structured codebase, but this
repo captures the evolution of the firmware and my embedded problem-solving
process.

---

## Skills Demonstrated

- Embedded C/C++ for Arduino / ESP32
- Real-time control of electromechanical devices (solenoids, switches)
- Game logic design and state management
- Hardware debugging and incremental prototyping
