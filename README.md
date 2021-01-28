# Platooning-LEGOs

## 1. Description
An open physical exemplar for engineering self-adaptive cyber-physical system-of-systems

- Overview of Platooning-LEGOs
<img src="/image/overview.png" width="450">

## 2. Equipment Requirements
- 3 LEGO EV3 bricks (https://education.lego.com/en-us/products/lego-mindstorms-education-ev3-core-set/5003400#lego-mindstorms-education-ev3)
- 3 laptops with Visual Studio Code installed (https://code.visualstudio.com/Download)

## 3. Physical environment setting

### Basic EV3 brick setting
- Installation: https://pybricks.github.io/ev3-micropython/startinstall.html
- How to use EV3 brick: https://pybricks.github.io/ev3-micropython/startbrick.html
- Creating and running programs: https://pybricks.github.io/ev3-micropython/startrun.html

### Robot building instruction
- Build the EV3 veihicle: [here](/physical%20implementation/ev3-vehicle.pdf)
- Attach sensors: [ultrasonic sensor](/physical%20implementation/ev3-ultrasonic-sensor.pdf), [color sensor](/physical%20implementation/ev3-color-sensor.pdf), and touch sensor (optional)

<img src="/image/vehicle.png" width="750">

### Road environment setting
- [Setting instruction](/physical%20implementation/road%20environment.pdf)
<img src="/image/road%20environment.png" width="550">

## 4. Preliminary for software implementation

### Bluetooth connection with laptop and EV3 brick
- Turn on Bluetooth and make Bluetooth visible
<img src="/image/bluetooth-on.png">

- Pair one EV3 Brick to another EV3 Brick and laptop
<img src="/image/bluetooth-pair.png">

- Detailed instruction: https://pybricks.github.io/ev3-micropython/messaging.html

### Changing the EV3 brick name
- Instruction: https://pybricks.github.io/ev3-micropython/startlinux.html#changing-the-ev3-brick-name

## 5. Adaptation strategy development
Sample codes for three-vehicle platoon are provided. Template code is also offered to implement your own adaptation strategies. The sample codes and template are under the directory [software implementation](/software%20implementation).

- Install the sample codes
  1. Create a new EV3 project for each vehicle
  2. Copy & paste sample codes for each vehicle
  3. Download the code to EV3 brick
  4. Run the three vehicles on the road together

## 6. Sample experiment results
<p>
  <img src="/image/experiment%20results.png" width="600"/>
</p>

- DEMO video

[<img src="/image/screenshot.PNG" width="600"/>](https://www.youtube.com/watch?v=tRSoTPq5EEI)

## 7. References
- EV3 MicroPython tutorial: https://pybricks.github.io/ev3-micropython/index.html
- Robot building instructions: 
  - EV3 vehicle: https://le-www-live-s.legocdn.com/sc/media/lessons/mindstorms-ev3/building-instructions/ev3-rem-driving-base-79bebfc16bd491186ea9c9069842155e.pdf
  - Ultrasonic sensor: https://le-www-live-s.legocdn.com/sc/media/lessons/mindstorms-ev3/building-instructions/ev3-ultrasonic-sensor-driving-base-61ffdfa461aee2470b8ddbeab16e2070.pdf
  - Color sensor: https://le-www-live-s.legocdn.com/sc/media/lessons/mindstorms-ev3/building-instructions/ev3-rem-color-sensor-down-driving-base-d30ed30610c3d6647d56e17bc64cf6e2.pdf
