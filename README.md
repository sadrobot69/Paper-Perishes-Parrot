# Paper-Perishes-Parrot

Experimental project using three Arduino UNO boards and MCP2515 controllers to
study spoofing on Classical CAN and the Parrot defense proposed by Dagan and
Wool.

## Flat project structure

Every variant except `simple` uses the same flat structure:

```text
<variant>/
├── <variant>.ino   Arduino sketch entry point
├── NodeConfig.h    selects the node to compile
├── Protocol.h      shared application protocol
├── A.ino           simulated motor
├── B.ino           sensor and, when applicable, Parrot defense
└── C.ino           attacker for the selected variant
```

Each node implementation is isolated in its own `.ino` file. There is no
longer a shared `CanLabNode.h` containing the three behaviors, and there are no
duplicate copies of `Protocol.h` in node subdirectories.

## Selecting and uploading a node

Arduino compiles all `.ino` files in a sketch directory together. Therefore,
`NodeConfig.h` selects which file provides `setup()` and `loop()`:

```cpp
#define ACTIVE_NODE NODE_A  // NODE_A, NODE_B, or NODE_C
```

To program the three boards with a variant such as `attack`:

1. Open `attack/attack.ino` in Arduino IDE.
2. Set `ACTIVE_NODE` to `NODE_A` in `NodeConfig.h`, select A's serial port, and
   upload.
3. Set it to `NODE_B`, select B's port, rebuild, and upload.
4. Set it to `NODE_C`, select C's port, rebuild, and upload.
5. Open C's Serial Monitor at 115200 baud and send `a` to start or stop the
   periodic attack. Send `1` to transmit a single attack frame.

Do not open `A.ino`, `B.ino`, or `C.ino` as independent sketches. They are the
implementation units of the flat sketch. The file named after the directory,
such as `attack.ino`, is only the Arduino-required entry point; the attack
itself is implemented directly in `C.ino`.

## Scenarios

### `attack`: spoofing without a defense

This is the baseline scenario without Parrot. B publishes legitimate samples
using identifier `0x120`, while C uses the same identifier to transmit valid
SENSOR messages containing a false distance of 10 cm. Since CAN does not
authenticate the source of a frame, A cannot distinguish B from C and processes
the false sample, placing the simulated motor in STOP.

This scenario shows that knowing the identifier and payload format is enough to
impersonate B. C can use its own session value because A interprets a session
change as a possible sensor restart. Neither the CAN identifier nor the session
authenticates the sender.

### `deffense`: Parrot implemented in B

This variant implements Parrot exclusively in B. When B receives a frame with
its own identifier `0x120`, it treats it as a spoofing attempt and transmits a
continuous sequence of zero-filled Dmessages. Collisions and automatic
retransmissions increase C's Transmit Error Counter (TEC), eventually forcing C
into error-passive and then BUS-OFF.

Parrot cannot necessarily prevent the first spoofed message. B detects the
attack by receiving that same frame from the bus, but A may already have
received and processed it. The defense acts against subsequent attacker
transmissions and attempts to remove C from the bus; it cannot retract a frame
that has already completed successfully.

### `attack-DoS-zeros`: zero-payload evasion

This variant implements the evasion described in Section 4.5.5 of Mert Dieter
Pesé's thesis,
[*Bringing Practical Security to Vehicles*](https://rtcl.eecs.umich.edu/rtclweb/assets/dissertations/2022/mpese_thesis_v3.pdf#page=151).
C transmits `0x120` with eight zero data bytes, exactly matching the Dmessage
used by B.

If B and C transmit the same bit sequence, there is no dominant/recessive
difference capable of generating a CAN error. The retransmission snowball does
not begin, so C's TEC does not grow toward BUS-OFF. These frames are not valid
SENSOR samples, but A records them as `ZERO_DATA` and enters STOP. Continuous
traffic can also saturate the bus and the MCP2515 receive buffers. The result is
primarily a Parrot evasion combined with a denial-of-service attack.

### `attack-one-shot-reset`: One-Shot and preventive reset

This variant implements an adaptive attack based on MCP2515 behavior and
research into CAN error confinement. C transmits valid 10 cm SENSOR samples but
enables `CANCTRL.OSM`, causing each frame to be attempted only once. A collision
with Parrot therefore creates an isolated error instead of the synchronized
automatic retransmissions required to form the snowball.

Isolated errors can still increase the TEC, so C monitors the counter and
resets the MCP2515 when it reaches approximately 32. The reset clears TEC and
REC, restores 500 kbit/s operation, and re-enables One-Shot mode. This allows C
to continue spoofing B without reaching BUS-OFF. The attack is outside Parrot's
original threat model because it assumes that compromised firmware can
reconfigure and reset the external CAN controller through SPI.

The main technical sources are the
[MCP2515 datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Family-Data-Sheet-DS20001801K.pdf),
Cho and Shin's work on
[BUS-OFF attacks](https://rtcl.eecs.umich.edu/rtclweb/assets/publications/2016/ktcho_busoff_CCS_16.pdf),
and the
[CANOX analysis](https://www.usenix.org/conference/usenixsecurity21/presentation/serag).

## Classical CAN frame fields

- SOF: one dominant bit.
- Arbitration: 11-bit standard identifier and RTR.
- Control: IDE, reserved bit, and DLC.
- Data: between 0 and 8 bytes.
- CRC: 15-bit CRC sequence and delimiter.
- ACK: ACK slot and delimiter.
- EOF: seven recessive bits.
- Intermission: three recessive bits between frames.
