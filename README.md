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

## ESP-Drone Communication Configuration

> Source: https://docs.espressif.com/projects/espressif-esp-drone/en/latest/communication.html

### Communication Hierarchy

```
==================  =================== =======================
Terminal            Mobile/PC           ESP-Drone
Application Layer   APP                 Flight Control Firmware
Protocol Layer      CRTP                CRTP
Transport Layer     UDP                 UDP
Physical Layer      Wi-Fi STA (Station) Wi-Fi AP (Access Point)
==================  =================== =======================
```

### UDP Communication

#### UDP Port

```
=====================   =================== =======================
App                     Direction           ESP-Drone
=====================   =================== =======================
192.168.43.42::2399     TX/RX               192.168.43.42::2390
=====================   =================== =======================
```

#### UDP Packet Structure

```
/* Frame format:
 * +=============+-----+-----+
 * | CRTP                      |   CKSUM   |
 * +=============+-----+-----+
 */
```

- The packet transmitted by the UDP: CRTP + verification information. 
- CRTP: As defined by the CRTP packet structure, it contains Header and Data, as detailed in the CRTP protocol section. 
- CKSUM: the verification information. Its size is 1 byte, and this CKSUM is incremented by CRTP packet byte.

**CKSUM Calculation Method**

```
#take python as an example: Calculate the raw cksum and add it to the end of the packet
raw = (pk.header,) + pk.datat
cksum = 0
for i in raw:
  cksum += i
cksum %= 256
raw = raw + (cksum,)
```

#### CRTP Protocol

The ESP-Drone project continues the CRTP protocol used by the Crazyflie project for flight instruction sending, flight 
data passback, parameter settings, etc.

CRTP implements a stateless design that does not require a handshake step. Any command can be sent at any time, but for 
some log/param/mem commands, the TOC (directory) needs to be downloaded to assist the host in sending the information 
correctly. The implemented Python API (cflib) can download param/log/mem TOC to ensure that all functions are available.

#### CRTP Packet Structure

The 32-byte CRTP packet contains one byte of Header and 31 bytes of Payload. Header records the information about the 
ports (4 bits), channels (2 bits), and reserved bits (2 bits).

```
  7   6   5   4   3   2   1   0
+---+---+---+---+---+---+---+---+
|     Port      |  Res. | Chan. | 
+---+---+---+---+---+---+---+---+
|            DATA 0             |
+---+---+---+---+---+---+---+---+
:   :   :   :   :   :   :   :   :
+---+---+---+---+---+---+---+---+
|            DATA 30            |
+---+---+---+---+---+---+---+---+

========    ========    ==============  =============================
Field       Byte        Bit             Description
========    ========    ==============  =============================
Header      0           0 ~ 1           Target data channel
\           0           2 ~ 3           Reserved for transport layer
\           0           4 ~ 7           Target data port
Data        1 ~ 31      0 ~ 7           The data in this packet
========    ========    ==============  =============================
```

#### Port Allocation

```
======  =====================   ===================================================================================
Port    Target                  Purpose
======  =====================   ===================================================================================
0       Console                 Read console text that is printed to the console on the Crazyflie using consoleprintf.
2       Parameters              Get/set parameters from the Crazyflie. Parameters are defined using a macro in the Crazyflie source-code
3       Commander               Send control set-points for the roll/pitch/yaw/thrust regulators
4       Memory access           Access non-volatile memories like 1-wire and I2C (only supported for Crazyflie 2.0)
5       Data logging            Set up log blocks with variables that will be sent back to the Crazyflie at a specified period. Log variables are defined using a macro in the Crazyflie source-code
6       Localization            Packets related to localization
7       Generic Setpoint        Allows to send setpoint and control modes
13      Platform                Used for misc platform control, like debugging and power off
14      Client-side debugging   Debugging the UI and exists only in the Crazyflie Python API and not in the Crazyflie itself.
15      Link layer              Used to control and query the communication link
======  =====================   ===================================================================================
```

Most of the modules in the firmware that are connected to the port are implemented as tasks. If an incoming CRTP packet 
is delivered in the messaging queue, the task is blocked in the queue. At startup, each task and other modules need to 
be registered for a predefined port at the communication link layer.

Details of the use of each port can be found at [CRTP - Communicate with Crazyflie](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/).


### Hardware

### LEDs

```
=================== ===== =================
Status               LED   Action
=================== ===== =================
POWER_ON            WHITE Fully lit
SENSORS CALIBRATION BLUE  Blinking slowly
SYSTEM READY        BLUE  Blinking
UDP_RX              GREEN Blinking
LOW_POWER           RED   Fully lit
=================== ===== =================
```

### Buttons

```
======= ======= ==============
Buttons   IO     Function
======= ======= ==============
SW1     GPIO1   Boot, Normal
SW2     EN      Reset
======= ======= ==============
```
