# GMSL UART server app

This app is used on AD-GMSL522-SL GMSL evaluation kit to interpret the UART commands sent by the GMSL GUI
and translate them to I2C transfers for communication with AD-GMSL522-SL internal GMSL parts.

This app was developed for Viper GMSL Eval Board but it can be used compiled and used on any platform.
The only requirement is to have one USB device port configured to advertise as a Teensy microcontroller

## App parameters and usage example
Usage: gmsl-uart-server [OPTION...]

  -d, --debug                Print debug information
  -i, --i2c=FILE             I2C device
  -s, --serial=FILE          Serial device
  -v, --verbose              Produce verbose output
  -?, --help                 Give this help list
      --usage                Give a short usage message
  -V, --version              Print program version

  Use verbose mode to print all the interpreted commands received from the GUI
  Use debug mode to print all transfers payload
