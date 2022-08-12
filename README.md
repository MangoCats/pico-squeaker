# pico-squeaker
HTTP server for Raspberry Pi Pico W which controls two PWM outputs and an amplifier control

Project Bill of Materials (adafruit part numbers)

- (5526) Raspberry Pi Pico W
- (987) 3.7W Class D Amplifier MAX98306
- (4755) Solar battery charger bq24074
- (5367)x2 5V 5W solar panels
- (353) 3.7V 6600mAh Lithium Ion Battery Pack
- (905) Weatherproof enclosure - project box
- (743)x2 DC Power Cable set
- (261) 2pin cable (power out from charger)
- (2193) 100uF 16v capacitors
- (1950) 6" 20 Female/Female jumper wires
- (1951) 3" 20 Female/Female jumper wires
- piezo speakers x4 [these](https://amazon.com/gp/product/B08SLZBKCH) or [these](https://amazon.com/gp/product/B09L7LHZML) worked well
- hookup wire from amplifier to speakers
- custom wooden box with hinged lid & pole mount
- #6x1/2" screws to mount solar panels to box

Software Development

- Thonny for micropython development, seems to work better when installed using pip3 instead of apt
- [Official docs](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html#raspberry-pi-pico-w)
- [micropython](https://www.cnx-software.com/2022/07/03/getting-started-with-wifi-on-raspberry-pi-pico-w-board/#wifi-with-micropython) rp2-pico-w-latest.uf2 file for Pico W
- [flash_nuke.uf2](https://www.raspberrypi.com/documentation/microcontrollers/raspberry-pi-pico.html#resetting-flash-memory) file for Pico


