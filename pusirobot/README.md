# Pusirobot Stepper Motor

Our model : PMC007C3SEP2M


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
## Run this before run the main program

```bash
sudo ip link set can0 down
sudo ip link set can0 txqueuelen 1000  # if less then no permission error comes
sudo ip link set can0 type can bitrate 1000000 loopback off
sudo ip link set can0 up
```




## How to Change the Node ID and Baudrate

**Example: Default Node ID is "3" using channel `can0`**  
*Note: Changes won't apply until the communication is reset.*

| Action                                      | CAN Command                        |
|--------------------------------------------|-----------------------------------|
| Change Node ID from "3" to "2"              | `603#2F02200002000000`             |
| Set Baudrate to 500kbps                     | `603#2F03200006000000`             |
| Save all settings to non-volatile memory    | `603#2F07200002000000`             |
| Reset communication                         | `000#8203`                         |

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


## SP Mode Tutorial

| Command Description               | CAN Command                        |
|----------------------------------|-----------------------------------|
| Set working mode to Position mode | `603#2f05600000000000`             |
| Set Acceleration Coefficient      | `603#2f08600008000000`             |
| Set Deceleration Coefficient      | `603#2f09600008000000`             |
| Set SP mode Speed| `603#231d6001007d0000`             |
| Set SP mode Position | `603#231d6002007d0000`         |
| Set Group ID to "1"                     | `603#2f06200001000000`             |
| Start Synchronized Motion         | `000#0A01`                         |

## PVT Mode Commands

| Command Description                        | CAN Command                        |
|-------------------------------------------|-----------------------------------|
| Set Peak Phase Current Threshold           | `603#2b0b600040060000`             |
| Set Microstep                              | `603#2b0a600020000000`             |
| Set Node ID (default: 3)                   | `603#2f02200003000000`             |
| Set Group ID (default: 0) to 1             | `603#2f06200001000000`             |
| Set Work Mode (default: 0, `2`: PVT mode)  | `603#2f05600002000000`             |
| Set PVT Mode (range: 0~2)                  | `603#2f10600200000000`             |
| Set PVT Points                             | `603#2f10600364000000`             |
| Set PVT Position                           | `603#2310601100000000`             |
| Set PVT Speed (PPS)                        | `603#2310601200000000`             |
| Set PVT Time (ms)                          | `603#2310601300000000`             |
| PVT Control (`0`: Stop, `1`: Start, `2`: Add to Queue, `3`: Clear Queue) | `603#2f10600100000000` |


## PDO Settings

### Read TPDO1 Mappings

| Command Description                                      | CAN Command                        |
|----------------------------------------------------------|-----------------------------------|
| Read the number of mapped application objects in TPDO1    | `603#40001a0000000000`             |
| Read TPDO1 mapping for the 1st application object         | `603#40001a0100000000`             |
| Read TPDO1 mapping for the 2nd application object         | `603#40001a0200000000`             |
| Read TPDO1 mapping for the 3rd application object         | `603#40001a0300000000`             |

### Expected Response

You will receive a response similar to this:

`583#43001A0320000C60`

### Response Interpretation

- **0x583** → COB-ID: SDO response from Node ID 3.  
- **0x43** → Command Specifier (CS): 4-byte length response.
- **0x001A** → Little Endian of **0x1A00**: Index for TPDO1 Mapping.  
- **0x01** → Sub-index 1 of the Mapping Parameter.  
- **0x20** → Data length 32 bits.  
- **0x00** → Reserved.  
- **0x0C60** → Little Endian of **0x600C**: Index representing Motor Position index.


## Notes
**I found a mistake in the user manual**

In the pmc007cxsxp2_user_manual_v0.2.6_en.doc, page 41 states that bit 6 should be 0 for absolute positioning and 1 for relative positioning.

However, after testing, I found this to be reversed: bit 6 is 1 for absolute positioning and 0 for relative positioning.

<br><br>
The PMC007CxSxPx controller uses the reverse EMF of 2-phase winding to realize 
sensor less blocking detection.

it's simple and safe, but the main problem is, It's accuracy is affected by many factors, such as it's current, subdivision parameter, voltage, especially the motor speed and phase inductance.


The blocking threshold range is usually set between -10 and 10.

<br>

**The `PMC007C3EP2_HB_ENC3_V394_G.bit` file is the firmware with PVT mode unlocked.** The seller said:

*"The PVT mode is an optional function, you know pmc007 has many models, they got different functions. Almost no one needs this PVT mode, it can be replaced by the pp mode at most situations. So we made the firmware for you to upgrade it, but this will use the PUSICAN, then the PCAN adapter"*


<br>

**Group id need to be set after changing operation mode**
```python
group_id = 5
init_operation_mode(0)
init_change_group_id(group_id)
```

