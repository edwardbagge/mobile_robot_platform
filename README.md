# Mobile Robot Platform

Market hardware:

1) Slamtec RPLIDAR 360deg Laser Scanner Development Kit (A2)
3) Raspberry Pi 5 8GB
4) Motor (x2)
    a) Geared 12V DC
    b) https://www.dfrobot.com/product-634.html?srsltid=AfmBOooisHisbAdQTIFWqWjwyqfrQ6lSMWhfAyFf_eR1mStN2Lgw48z1
5) Adafruit ESP32
    a) Site: https://www.adafruit.com/product/3405?srsltid=AfmBOopUAx100z8yAm6UoCGiwOnHXg8pmEAWBhQBoX1S4zC-emE5ZuI-
    b) Guide: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather
6) L298N motor driver
    a) Link: https://www.kouluelektroniikka.fi/tuote/ohjelmoitava-elektroniikka/moduulit/moottorinohjaimet/4615220/moottoriohjain-l298n?gad_source=1&gad_campaignid=18512997805&gclid=CjwKCAjwhLPOBhBiEiwA8_wJHC9BXevq7BbO3PPyvmkiGMVP1ICW89LMHhSXFAoyNezPm5dvVIyZ-RoCt-4QAvD_BwE
7) 3S LiPo battery
    a) https://www.akkuasiantuntija.fi/fi/artiklar/li-po-3s-111v-30c-2-200-mah-t-kontakti-vapex.html?gad_source=1&gad_campaignid=23517029518&gclid=CjwKCAjw1N7NBhAoEiwAcPchp0HhxQxsXMXeCo7GeVmALIXp3xbAS1I--QNepleqVuBZ9lA_ofojqhoCBx8QAvD_BwE
8) Multiple Wago connectors, jumper wires, 3mm bolts; nuts; washers, double-sided tape, zip-ties, and other "prototyping"-stuff
9) Breadboard
10) Levelshifter (5V/3.3V)
11) Mounting hub (part that is attached to 6mm motor-shaft)
    a) https://partco.shop/product/axle-adapter-6-25-4mm-m3-4004

Custom hardware:

1) Chassis
2) Elevated platform for Lidar
3) Holders for motors

ROS 2 notes:

1) `ros2_ws/src/robot_bringup/config/floor_safe_params.yaml` is the main bringup parameter file for the base and lidar stack.
2) `ros2_ws/src/robot_bringup/config/slam_params.yaml` contains the current `slam_toolbox` parameters.
