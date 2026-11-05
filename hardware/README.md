# Hardware

This folder contains the mechanical design and manufacturing files for the mobile robot platform developed in the thesis *Mobile Robot Platform for Robotics Education*.

The robot combines commercially available components with custom-designed mechanical parts. Its main components include a Raspberry Pi 5, an ESP32 microcontroller, two geared DC motors with encoders, a motor driver, and an RPLIDAR sensor. The custom-designed parts form the chassis and provide mounting and connection points for the components.

## Purchased Components

| Component | Quantity | Purpose |
|---|---:|---|
| Raspberry Pi 5 (8 GB) | 1 | Main computer running Ubuntu and ROS 2 |
| Adafruit HUZZAH32 ESP32 Feather | 1 | Embedded motor control and encoder acquisition |
| Slamtec RPLIDAR A2M8 | 1 | LiDAR measurements for mapping |
| 12 V geared DC motors with encoders | 2 | Drive motors and wheel rotation feedback |
| L298N motor driver | 1 | Motor speed and direction control |
| 3S LiPo battery | 1 | Power source for the motor system |
| Buck converter (rated above 5 A) | 1 | Voltage conversion |
| Logic level shifter (5 V / 3.3 V) | 1 | Voltage-level conversion between electronics |
| Motor shaft mounting hubs | 2 | Connect the motor shafts to the wheel adapters |
| LEGO Technic 8069 rims and rear wheels (or similar) | 2 | Drive wheels |
| Caster wheel | 1 | Additional chassis support |
| Fuse holder and 10 A fuse | 1 set | Electrical protection |
| Electrical switch | 1 | Power switching |
| T-plug to XT60 cable | 1 | Battery connection |
| Breadboard | 1 | Prototyping electrical connections |
| WAGO connectors, jumper wires, M3 bolts, nuts, washers, double-sided tape, and cable ties | Various | Wiring and assembly |

### Component Links

- [Slamtec RPLIDAR A2](https://www.slamtec.com/en/lidar/a2)
- [DFRobot geared DC motor](https://www.dfrobot.com/product-634.html)
- [Adafruit HUZZAH32 ESP32 Feather](https://www.adafruit.com/product/3405) and [documentation](https://learn.adafruit.com/adafruit-huzzah32-esp32-feather)
- [L298N motor driver](https://www.kouluelektroniikka.fi/tuote/ohjelmoitava-elektroniikka/moduulit/moottorinohjaimet/4615220/moottoriohjain-l298n)
- [3S LiPo battery](https://www.akkuasiantuntija.fi/fi/artiklar/li-po-3s-111v-30c-2-200-mah-t-kontakti-vapex.html)
- [T-plug to XT60 cable](https://www.akkuasiantuntija.fi/fi/artiklar/laddkabel-t-plug-hane-till-xt60-14awg-500mm.html)
- [Motor shaft mounting hub](https://partco.shop/product/axle-adapter-6-25-4mm-m3-4004)
- [Caster wheel](https://partco.shop/product/ball-bearing-3-4-metal-4006)

The Raspberry Pi 5 is powered separately from a wall outlet. The battery supplies the motor system, so the robot's movement is limited by the Raspberry Pi power cable.

## Custom-Designed Parts

The following mechanical parts were designed for the robot. The links point to the files included in this folder.

### Chassis

The chassis supports the robot's mechanical and electronic components.

- [CAD model (`.psm`)](Chassis/chassis.psm)
- [DXF file](Chassis/chassis.dxf)
- [PDF drawing](Chassis/chassis.pdf)

### Motor Holders

The motor holders secure the two drive motors to the chassis.

- [CAD model (`.psm`)](Motor%20holders/motor_holder.psm)
- [Flat CAD model (`.psm`)](Motor%20holders/motor_holder%20-%20flat.psm)
- [DXF file](Motor%20holders/motor_holder.dxf)
- [PDF drawing](Motor%20holders/motor_holder.pdf)

### LiDAR Platform

The elevated platform provides a mounting location for the LiDAR sensor.

- [CAD model (`.psm`)](Lidar%20platform/lidar_platform.psm)
- [Flat CAD model (`.psm`)](Lidar%20platform/lidar_platform%20-%20flat.psm)
- [Flat DXF file](Lidar%20platform/lidar_platform%20-%20flat.dxf)
- [PDF drawing](Lidar%20platform/lidar_platform.pdf)

### Wheel Adapters

The wheel adapters connect the motor shaft mounting hubs to the LEGO wheel rims. Two adapter variants are included.

- [Adapter 484 CAD model (`.par`)](Wheel%20adapters/wheel_adapter_484.par)
- [Adapter 485 CAD model (`.par`)](Wheel%20adapters/wheel_adapter_485.par)
- [Adapter 484 STL](Wheel%20adapters/wheel_adapter_484.stl)
- [Adapter 485 STL](Wheel%20adapters/wheel_adapter_485.stl)
- [PDF drawing](Wheel%20adapters/wheel_adapter.pdf)

## CAD and Manufacturing Files

The folder contains several file formats:

| Format | Use |
|---|---|
| `.psm` | Sheet-metal CAD part files |
| `.par` | CAD part files |
| `.dxf` | 2D geometry for manufacturing |
| `.stl` | Mesh models for 3D printing |
| `.pdf` | Drawings for viewing and reference |

The CAD files can be inspected or modified in compatible CAD software. The DXF, STL, and PDF files provide alternative representations for manufacturing and documentation.

**Note:** Check the dimensions, material, and manufacturing settings before fabricating replacement parts. The repository files document the prototype design; they do not by themselves constitute a complete assembly or electrical safety guide.
