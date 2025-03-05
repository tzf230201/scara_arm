# Pusirobot Stepper Motor

The PMC007CxSxPx controller uses the reverse EMF of a two-phase winding to realize sensorless blocking detection. Its accuracy is influenced by various factors such as current, subdivision, voltage, motor parameters, and especially motor speed and phase inductance.  
The blocking threshold range is usually set between -10 and 10 (stall length).

---

## Boot/Firmware Configuration (`config.txt`)

```bash
# Enable audio, I2C, and SPI
dtparam=audio=on
dtparam=i2c_arm=on
dtparam=spi=on

# Enable MCP2515 CAN interface
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=25
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=23
dtoverlay=spi-bcm2835-overlay
```

---

## How to Change the Node ID and Baudrate

**Example: Default Node ID is "3" using channel `can0`**  
*Note: Changes won't apply until the communication is reset.*

| Action                                      | CAN Command                        |
|--------------------------------------------|-----------------------------------|
| Change Node ID from "3" to "2"              | `603#2F02200002000000`             |
| Set Baudrate to 500kbps                     | `603#2F03200006000000`             |
| Save all settings to non-volatile memory    | `603#2F07200002000000`             |
| Reset communication                         | `000#8203`                         |

---

## PP Mode Tutorial

| Command Description               | CAN Command                        |
|----------------------------------|-----------------------------------|
| Set current to 1200mA             | `603#2b0b6000B0040000`             |
| Set subdivision to 32             | `603#2b0a600010000000`             |
| Set working mode to PP mode       | `603#2f05600004000000`             |
| Set PP mode acceleration to 32000 pps | `603#232d6001007d0000`         |
| Set PP mode deceleration to 32000 pps | `603#232d6002007d0000`         |
| Set PP mode speed to 10000 pps    | `603#232e600310270000`             |
| Set target position to 32000 pulses| `603#232e6004007d0000`             |
| Set control word                  | `603#2b2e600110000000`             |

---

## SP Mode Tutorial

| Command Description               | CAN Command                        |
|----------------------------------|-----------------------------------|
| Set Motion Mode                   | `603#2f05600000000000`             |
| Set Acceleration Coefficient      | `603#2f08600008000000`             |
| Set Deceleration Coefficient      | `603#2f09600008000000`             |
| Set Synchronized Positioning Speed| `603#231d6001007d0000`             |
| Set Synchronized Positioning Position | `603#231d6002007d0000`         |
| Set Group ID                      | `603#2f06200001000000`             |
| Start Synchronized Motion         | `000#0A01`                         |