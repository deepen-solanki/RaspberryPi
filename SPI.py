# SPI communication on the Raspberry Pi. This sends data out as an SPI master. 
# Pin configuration is as follows for Bus 0
#
# Pin 19 - MOSI
# Pin 21 - MISO
# Pin 23 - SCLK
# Pin 24 - CS0
#
# Pin configuration is as follows for Bus 1
#
# Pin 38 - MOSI
# Pin 35 - MISO
# Pin 40 - SCLK
# Pin 11 - CS1
#
# 
# These should be connected to your receiving device in the same fashion, i.e - MOSI (Pi) -- MOSI (Device)
# For this example you can leave MISO unconnected as we are only sending data out from the Pi
#
# Make sure SPI is enabled on the Pi. If not, follow these steps
# 
# Open up the terminal, type "sudo raspi-config" (without the " ")
# Interfacing Options -> SPI -> Enable
# Reboot the Pi

import spidev

spi0 = spidev.SpiDev()
spi0.open(0,0) #Bus 0 
spi0.max_speed_hz = 400000
spi0.mode = 0

spi1 = spidev.SpiDev()
spi1.open(1,1) #Bus 1
spi1.max_speed_hz = 400000
spi1.mode = 0

while 121:
  a = 12 #Any value between 0-255
  spi0.xfer2([a])
  spi1.xfer2([a])
