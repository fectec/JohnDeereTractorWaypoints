  ^~^  ,
 ('Y') )
 /   \/ 
(\|||/)

# John Deere Tractor Waypoints

Main project in collaboration with *John Deere* for the undergrad course “**Design of Advanced Embedded Systems**”, which delves mainly into *Design and Analysis of Algorithms*, *Digital Signal Processing*, *Shared-Memory Architecture* and *Communication Interfaces*.

<p align="center">
  <img src="https://github.com/user-attachments/assets/b687bccc-9adf-476f-9d3e-f90a392c2321" alt = "NUCLEO-H755ZI-Q" width="100" height="200"/>
</p>

It consists of a **John Deere Tractor (Servo Steering Vehicle)**. The *NUCLEO-H755ZI-Q* development board with *STM32H755ZIT6U MCU* receives waypoints, with which it generates a route for the tractor. First, it waits for the coordinates of the current position given by the *John Deere Global Positioning Device*. Then, the vehicle is able to follow the route based on the information delivered by the Deere tool.
