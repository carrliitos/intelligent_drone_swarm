# ESP32 Drone Swarm Coordination System
> Benzon Carlitos Salazar (salazarbc24@uww.edu)

## Project Overview

This project aims to develop a coordinated drone swarm system using ESP32 microcontrollers, implementing a hierarchical 
structure with a **mothership** (leader drone) and multiple **child drones** (followers). The primary goal is to achieve 
synchronized and adaptive behavior in the swarm, capable of handling complex tasks such as mapping, patrolling, 
search-and-rescue, and collaborative exploration.

## Objectives

- **Centralized Command System**: Develop a control system where the mothership processes inputs from both the 
environment and follower drones. It will act as the primary decision-maker, coordinating the actions of the child drones 
based on gathered telemetry data.
- **Real-Time Communication**: Design protocols to enable efficient, low-latency data exchange between the mothership 
and child drones, ensuring effective collaboration within the swarm.

## Core Functionalities

### 1. Formation Flying
   - The swarm follows a formation led by the mothership, dynamically adjusting based on environmental feedback and 
   telemetry data from child drones.
   
### 2. Synchronized Movements for Mission Execution
   - **Mapping**: Plan and execute synchronized movements to cover specified areas for mapping purposes.
   - **Patrolling**: Implement patrolling routes that can be dynamically adjusted based on environmental conditions.
   - **Search-and-Rescue**: Enable swarm-based exploration and identification for search-and-rescue operations.

### 3. Collaborative Exploration and Decision-Making
   - Enable the swarm to collectively explore areas, assess sensory data, and make decisions on further actions based 
   on real-time environmental inputs.
