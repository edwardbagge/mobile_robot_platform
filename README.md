# mobile_robot_platform

Hardware:

1) RPLIDAR 360deg Laser Scanner Development Kit (A2)
2) Raspberry Pi 5 8GB
3) DC Motor (x2)
    a) GB37Y3530-43.8EN
    b) DC12V 255 RPM
4) Adafruit ESP32
    a) Site: https://www.adafruit.com/product/3405?srsltid=AfmBOopUAx100z8yAm6UoCGiwOnHXg8pmEAWBhQBoX1S4zC-emE5ZuI-
    b) Guide: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather
5) L298N motor driver
    a) Link: https://www.kouluelektroniikka.fi/tuote/ohjelmoitava-elektroniikka/moduulit/moottorinohjaimet/4615220/moottoriohjain-l298n?gad_source=1&gad_campaignid=18512997805&gclid=CjwKCAjwhLPOBhBiEiwA8_wJHC9BXevq7BbO3PPyvmkiGMVP1ICW89LMHhSXFAoyNezPm5dvVIyZ-RoCt-4QAvD_BwE
6) 3S LiPo battery
    a) https://www.akkuasiantuntija.fi/fi/artiklar/li-po-3s-111v-30c-2-200-mah-t-kontakti-vapex.html?gad_source=1&gad_campaignid=23517029518&gclid=CjwKCAjw1N7NBhAoEiwAcPchp0HhxQxsXMXeCo7GeVmALIXp3xbAS1I--QNepleqVuBZ9lA_ofojqhoCBx8QAvD_BwE

Done setups and tests:

1) RPI 5 has Ubuntu 24.04 and rplidar.
    a) Slamtec A2M8 works and recognizes surroundings.
2) Arduino IDE has ESP32 expansion-pack
3) ESP32 can control DC-motors
    a) Single motor
    b) Dual motor
    c) PWM

To be done:

1) Encoders on motors work?