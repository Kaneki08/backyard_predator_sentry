# AS5600 encoder bench test (NUCLEO-L476RG)

Standalone firmware that tests one AS5600 magnetic angle sensor module on a NUCLEO-L476RG. It runs over a USB serial console. It probes the sensor, checks the magnet, streams the angle, and runs noise and speed tests. It also checks that the firmware recovers when the sensor is unplugged.

The module this guide covers is the common square blue AS5600 board (sold as "WWZMDiB AS5600 Magnetic Encoder 3.3V") with two headers: **VCC / OUT / GND** on one side and **DIR / SCL / SDA / GPO** on the other.

The driver ([Core/Src/as5600.c](Core/Src/as5600.c)) is the one intended for the final pan/tilt firmware. It is **read-only**: it never writes the BURN register (0xFF, one-time programmable and permanent) or ZPOS/MPOS/MANG/CONF. Zeroing is done in software, in RAM.

Contents:
1. [About the module](#1-about-the-module)
2. [Wiring](#2-wiring)
3. [Magnet placement](#3-magnet-placement)
4. [Build and flash](#4-build-and-flash)
5. [Serial console and commands](#5-serial-console-and-commands)
6. [Test procedure](#6-test-procedure)
7. [What to record](#7-what-to-record)
8. [Troubleshooting](#8-troubleshooting)
9. [Project layout and regenerating from CubeMX](#9-project-layout-and-regenerating-from-cubemx)

---

## 1. About the module

What the vendor schematic shows, and what it means for this test:

| Part (schematic) | What it does | Consequence |
|---|---|---|
| VCC → chip pin 1 (VDD5V), and through **R1 = 0 Ω** to pin 2 (VDD3V3); C1 100 nF, C2 10 µF | The chip runs in **3.3 V mode** | Power VCC from **3.3 V only**. 5 V would put 5 V on VDD3V3. The AGC range is **0 to 128**, which is the firmware default |
| **R3 = 10 kΩ** SCL → VCC, **R2 = 10 kΩ** SDA → VCC | I2C pull-ups to 3.3 V | No extra pull-ups needed for short wires |
| DIR (pin 8) → header, **no resistor** | DIR **floats** unless you wire it | **You must wire DIR to GND.** The datasheet does not allow DIR to float (direction undefined) |
| GPO (pin 5, "PGO") → header, plus **R4 = 1 kΩ to GND** | PGO is held low, so the chip is in **programming option B** | **OUT becomes a burn input. See the warning below** |
| OUT (pin 3) → header | Angle output (analog/PWM) | Not used by this test. **Leave unconnected** |

Header pin order: **VCC, OUT, GND** on the 3-pin side (square pad = VCC). **DIR, SCL, SDA, GPO** on the 4-pin side (square pad = DIR).

> ⚠️ **Never connect OUT to GND, not even briefly.** With PGO held low by R4, the datasheet's programming option B (DS000365 p.23) is armed. If OUT is pulled to GND for ≥ 100 ms, the magnet is turned more than 18°, and OUT is pulled to GND again for ≥ 100 ms, the chip **permanently burns** ZPOS/MPOS into its one-time-programmable memory. This board has the 10 µF VDD3V3 capacitor that burning needs, so it really can do this on 3.3 V. Leave OUT unconnected, keep loose jumper wires and scope ground clips away from it, and never wire it to an MCU pin.
>
> This test firmware reads RAW ANGLE, which a burn would not affect, but a burn would permanently change ANGLE and OUT for the final system. In option B, the datasheet also says OUT is held high by an internal pull-up until programming is done, so expect no angle signal on OUT.
>
> **Optional, recommended for the final sentry:** desolder **R4 (marked `102`)**. PGO then returns to its internal pull-up, which selects option A (I2C-only programming), and OUT loses its burn function. Nothing in this test requires removing it.

**Optional 30-second check** (module unpowered, multimeter on resistance), because clone boards vary:

| Measure | Expect |
|---|---|
| SCL → VCC and SDA → VCC | about 10 kΩ each (R3, R2) |
| GPO → GND | about 1 kΩ (R4 present) |
| DIR → VCC and DIR → GND | open, or very high (DIR floating, as the schematic shows) |

If a reading is far off, note it on the [results sheet](#7-what-to-record). If DIR reads about 0 Ω to VCC, do **not** wire it to GND, because that would short the supply.

---

## 2. Wiring

Six wires: VCC, GND, SCL, SDA, and DIR to GND. Power the module from the Nucleo **3V3** pin so the I2C lines are at 3.3 V.

| Module pin | Connect to | Nucleo header location | Notes |
|---|---|---|---|
| **VCC** | **3V3** | **CN6 pin 4** (Arduino power header, labelled `3V3`) | **3.3 V only.** Never 5 V |
| **OUT** | **leave unconnected** | n/a | **Never to GND** (see the warning in [section 1](#1-about-the-module)) |
| **GND** | **GND** | **CN5 pin 7** (Arduino digital header, next to D14/D15) | |
| **DIR** | **GND** | **CN6 pin 6 or 7** | **Required.** DIR floats on this board. GND gives an angle that increases **clockwise**, viewed from the magnet side |
| **SCL** | **PB8** (Arduino **D15**) | **CN5 pin 10** (the top pin of the 10-pin digital header) | 10 kΩ pull-up on the module |
| **SDA** | **PB9** (Arduino **D14**) | **CN5 pin 9** | 10 kΩ pull-up on the module |
| **GPO** | leave unconnected | n/a | Already pulled to GND by R4 on the board |

DIR is on the 4-pin header and GND is on the 3-pin header, so DIR needs its own wire to a Nucleo GND pin.

The module's 10 kΩ pull-ups are enough for short wires. If the wires are longer than about 20 cm, or `scan`/the stream shows occasional I2C errors, add **4.7 kΩ** from SCL to 3V3 and from SDA to 3V3 near the Nucleo.

On the ST morpho headers, PB8 is CN10 pin 3 and PB9 is CN10 pin 5, if those are easier to reach.

```
 NUCLEO-L476RG  Arduino headers (top view, USB at the top)

   CN6 (power, left side)            CN5 (digital, right side, top)
   1  NC                             10  D15 / PB8  ── SCL
   2  IOREF                           9  D14 / PB9  ── SDA
   3  RESET                           8  AVDD
   4  3V3   ── VCC                    7  GND        ── GND
   5  5V    (do not use)              6  D13
   6  GND   ── DIR                    ...
   7  GND
   8  VIN

   Module 3-pin header:  VCC ── 3V3 (CN6-4)    OUT  ×  (NOT connected, never GND)    GND ── CN5-7
   Module 4-pin header:  DIR ── GND (CN6-6)    SCL ── CN5-10    SDA ── CN5-9    GPO  ×
```

### Notes on the wiring
- **Pin choice.** The STM32L476RG datasheet lists PB8 = I2C1_SCL and PB9 = I2C1_SDA, both on alternate function AF4. The NUCLEO-L476RG user manual (UM1724) puts them on Arduino D15 and D14. No solder-bridge changes are needed.
- **Keep the I2C wires short** (≤ 20 cm), and twist SCL and SDA each with a GND wire if they are longer. 400 kHz I2C allows at most a 300 ns rise time.
- **Serial.** The USB cable to the Nucleo's ST-LINK both powers the board and provides the serial console. USART2 on PA2/PA3 is routed to the ST-LINK virtual COM port by default. No extra wires are needed.

---

## 3. Magnet placement

- Use a **diametrically magnetized** magnet, with its N and S poles on opposite sides of the round edge. A typical choice is 6 mm diameter × 2 to 3 mm thick. An axially magnetized disc (poles on the flat faces) gives a poor or no angle reading unless it is spun on its edge.
- **Center the rotation axis over the chip package**, not over the center of the board, which may differ on generic modules. The Hall sensors lie on a 1 mm-radius circle at the center of the package. The datasheet allows at most **0.25 mm** of axis offset for a 6 mm magnet. More offset shows up as a non-linear angle (see test step 3).
- **Air gap:** 0.5 to 3 mm between the magnet face and the top of the chip. The field at the chip needs to be 30 to 90 mT.
- Adjust the gap until the firmware reports **Magnet OK**, with AGC near the middle of its range, **about 64 of 128**. A high AGC means the field is weak, so move the magnet closer. A low AGC means the field is strong, so move it farther away.
- Keep steel parts (screws, brackets, motor housings) away from the magnet, and mount the module with non-magnetic hardware (nylon or brass).

---

## 4. Build and flash

### Requirements
- **STM32CubeIDE.** Verified with **1.19.0**.
- **STM32CubeL4 firmware package V1.18.2.** CubeIDE downloads it the first time it is needed. On the development PC it is already installed at `%USERPROFILE%\STM32Cube\Repository\STM32Cube_FW_L4_V1.18.2`. The HAL sources the build needs are also copied into this project's `Drivers/` folder, so building does not require the package. Only regenerating from the `.ioc` does.
- A USB cable for the Nucleo's ST-LINK. Leave the Nucleo jumpers at their defaults (JP5 on `U5V`, JP6 `IDD` fitted).

### Option A: STM32CubeIDE (GUI)
1. **File → Import… → General → Existing Projects into Workspace → Next.**
2. **Select root directory:** browse to `firmware/as5600_bench`. Tick `as5600_bench` and **leave "Copy projects into workspace" unticked**, so you edit the files in the repo. Click **Finish**.
3. Select the project, then **Project → Build Project** (the hammer icon). The Console should end with `Build Finished. 0 errors, 0 warnings.`
4. Plug in the Nucleo, then **Run → Debug As → STM32 C/C++ Application**. Accept the defaults on the first launch. The debugger flashes the board and stops at `main()`. Press **Resume (F8)**.
   For flashing without debugging, use **Run → Run As → STM32 C/C++ Application**.
5. If CubeIDE asks to upgrade the ST-LINK firmware, accept.

### Option B: command line (no GUI)
Run from a terminal in the repo root (PowerShell shown):

```powershell
# Build (Debug configuration) into a temporary workspace
& "C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\stm32cubeidec.exe" --launcher.suppressErrors -nosplash `
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild `
  -data "$env:TEMP\as5600_ws" -import "$PWD\firmware\as5600_bench" -build "as5600_bench/Debug"

# Flash over the Nucleo's ST-LINK (SWD), verify, and reset
& "C:\ST\STM32CubeCLT_1.22.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" `
  -c port=SWD -w "firmware\as5600_bench\Debug\as5600_bench.elf" -v -rst
```

> **printf:** The firmware formats every number with integer fixed-point. It does **not** need "Use float with printf from newlib-nano" (`-u _printf_float`), so leave that linker option off.

---

## 5. Serial console and commands

1. Find the COM port in Device Manager → Ports: **"STMicroelectronics STLink Virtual COM Port (COMx)"**.
2. Open it with PuTTY, Tera Term, or the CubeIDE terminal: **115200 baud, 8 data bits, no parity, 1 stop bit, no flow control**.
   - Turn **local echo off**. The firmware echoes what you type.
   - Enter can send CR, LF, or CR+LF. All three work. Backspace works.
3. Press the black **RESET** button on the Nucleo to see the boot banner.

### Boot sequence
On reset, the firmware does the following:
1. Prints a banner showing the clock (80 MHz) and the I2C timing register (`0x00702991` = 400 kHz).
2. Waits 20 ms for the sensor to power up.
3. Probes address 0x36 and prints `Probe 0x36: ACK` or `NACK`.
4. If the sensor answers, reads STATUS/AGC/MAGNITUDE and prints a plain-English magnet verdict.
5. Starts streaming at **10 Hz**.

If the sensor is missing, the firmware says so and retries every 500 ms. Streaming starts as soon as the sensor appears.

### Stream format
```
#     t_ms   raw     deg   zeroed     unwrapped    vel_dps  magnet     agc
      5321  2817  247.59   247.59        247.59        0.0  OK          63
```
| Column | Meaning |
|---|---|
| `t_ms` | Milliseconds since reset |
| `raw` | RAW ANGLE register, 0 to 4095 (12-bit, 0.0879° per count) |
| `deg` | Raw angle in degrees, 0 to 359.91 |
| `zeroed` | Angle relative to the software zero, 0 to 359.91 |
| `unwrapped` | Multi-turn position relative to the zero. Keeps counting past 360 and below 0 |
| `vel_dps` | Average velocity since the previous line, in °/s |
| `magnet` | `OK`, `WEAK` (move closer), `STRONG` (move farther), `NO_MAGNET`, or `ERR` if the status read failed |
| `agc` | AGC value, 0 to 128 in 3.3 V mode. Ideal is about 64 |

Every line the firmware prints on its own that is not data starts with `#`: headers, errors, and reconnect messages. Filter them out of a log with `grep -v '^#'`.

The unwrap is tracked by a background read at 200 Hz, regardless of the stream rate. It stays correct up to 100 rev/s, even with `rate 1`.

### Commands
Commands are case-insensitive. `help` prints this list on the device.

| Command | What it does |
|---|---|
| `help` | List the commands |
| `stream on` / `stream off` | Start or stop the stream. With no argument, prints the current state |
| `rate <hz>` | Set the stream rate, 1 to 100 Hz. The default is 10 |
| `csv on` / `csv off` | Switch to machine-readable CSV (`t_ms,raw,deg,zeroed_deg,unwrapped_deg,vel_dps,md,ml,mh,magnet,agc`) |
| `zero` | Make the current angle the software zero, and reset the unwrapped position to 0. **RAM only**: nothing is written to the sensor, and a reset clears it |
| `diag` | Decode every register (ZMCO, ZPOS, MPOS, MANG, CONF fields, STATUS bits, RAW ANGLE, ANGLE, AGC, MAGNITUDE). Also prints the magnet verdict and link statistics (errors, recoveries, dropped lines) |
| `noise [n]` | Take `n` back-to-back reads (default 1000, max 100000) with the shaft **stationary**. Reports min, max, peak-to-peak, mean and standard deviation in counts and degrees. Any key aborts |
| `speed` | Time 1000 reads with each method (register read, fast read, and a full stream line). Reports µs per read and the maximum read rate |
| `scan` | Probe every I2C address (0x08 to 0x77), then 0x36 specifically. Runs bus recovery first if the bus is stuck |

---

## 6. Test procedure

Fill in the [results sheet](#7-what-to-record) as you go. Before starting, read the [OUT-pin warning](#1-about-the-module), wire per [section 2](#2-wiring) (including DIR to GND), flash the firmware, open the terminal, and press RESET.

### Step 1: Bus scan
1. Type `stream off`, then `scan`.
2. **Pass:** `0x36 ACK  <- AS5600`, `1 device(s) found`, `Probe 0x36: ACK - AS5600 present`.
3. **If it fails:**
   - `NACK` or 0 devices: check the VCC/GND wiring. Measure about 3.3 V between VCC and GND on the module. Check that SDA and SCL are not swapped (SDA = D14 = CN5 pin 9, SCL = D15 = CN5 pin 10).
   - `bus BUSY`: a line is held low, or floating because the module isn't powered. See [Troubleshooting](#8-troubleshooting).

### Step 2: Magnet check, then adjust the gap
1. Type `diag`.
2. Look at the `>>>` verdict line:
   - `Magnet NOT DETECTED`: no field, or the magnet is far off-center or axially magnetized.
   - `Magnet TOO WEAK - move the magnet CLOSER` (ML = 1).
   - `Magnet TOO STRONG - move the magnet FARTHER` (MH = 1).
   - `Magnet OK` (MD = 1, ML = 0, MH = 0).
3. Adjust the gap and repeat `diag` until you get **Magnet OK**, with **AGC about 40 to 90 (of 128)**. The firmware also prints a hint if AGC is near either end of the range.
4. While you are in `diag`, check the configuration. On a new part, `ZMCO 0` means it has never been burned, and ZPOS, MPOS and MANG should all be 0. CONF is normally 0x0000 (PM=NOM, HYST=OFF, SF=16x, FTH=slow filter only, WD=OFF). Generic modules are sometimes sold pre-programmed, so **record whatever you see**. The firmware uses RAW ANGLE, which ZPOS/MPOS/MANG do not affect, so a programmed part still tests correctly.
5. Rotate the magnet slowly through a full turn and repeat `diag` at a few angles. The verdict should stay OK, and AGC should not swing by more than about ±10. A large swing means the magnet is off-center or tilted.

### Step 3: Full turn by hand (smoothness and direction)
1. `stream on`. The rate is 10 Hz by default; for a smoother trace, use `rate 20`.
2. Turn the magnet **slowly** (about 10 s per revolution) **clockwise as seen from the magnet side** for one full turn, then counter-clockwise back.
3. **Pass:**
   - `deg` increases **smoothly** from 0 to 359.9 while turning clockwise (DIR wired to GND), then wraps to 0 once per turn.
   - No sudden jumps (more than ~5° between consecutive lines at this speed), and no values stuck at one number.
   - `magnet` shows `OK` all the way round.
   - `vel_dps` is positive in the increasing direction. About 36 °/s at 10 s/rev.
4. **Optional linearity check:** mark 0°, 90°, 180° and 270° on the rotating part with a protractor or printed dial. At each mark, `zero` at the first mark and read `zeroed`. Expect each mark within about ±1° (datasheet INL is ±1° max with no axis offset). Errors that look like a sine wave usually mean the magnet is off-axis.

### Step 4: Stationary noise test
1. Hold the shaft still, ideally clamped, with no one touching the bench.
2. `noise 10000` takes about 1.5 to 2 s. Then run `noise 10000` again at two other angles.
3. **Expected:**
   - Std dev **≤ ~0.2 count (≤ ~0.015°)**. The datasheet spec is 0.015° RMS max at the default slow filter, with a 30 to 90 mT field.
   - Peak-to-peak typically **1 to 3 counts** (≤ 0.26°).
   - A firmware warning appears if p-p exceeds 20 counts. That means the shaft moved or the magnet is marginal.
   - If `diag` showed a non-default SF (slow filter) setting, the noise will be higher. Record it.
4. Run `speed`. Record the µs per read and the reads/s it reports. All three methods should show `errors 0`, and the fast-read cross-check should print `consistent`.

### Step 5: Unwrap past 360° in both directions
1. `stream on`, then `zero`. The `unwrapped` column reads 0.00.
2. Turn **2 full turns in the increasing direction** (past 360°, then past 720°) and stop.
   **Pass:** `unwrapped` is about **+720** (± the angle you stopped at). `deg`/`zeroed` stay inside 0 to 360 and wrap each turn. `unwrapped` never jumps by ~360 at the wrap point.
3. Turn **3 full turns the other way** and stop.
   **Pass:** `unwrapped` goes back through 0 and reaches about **−360**, with no jump at the 0/4095 crossing.
4. Turn back to the starting mark. **Pass:** `unwrapped` reads within ±1° of 0.
5. Optionally, spin quickly (about 1 to 2 rev/s by hand). `unwrapped` must still count whole turns correctly. The 200 Hz tracker handles up to 100 rev/s.

### Step 6: Disconnect and reconnect mid-stream
1. `stream on` (10 Hz) and let it run.
2. **Pull the SDA wire** from the module while streaming.
   **Pass:**
   - Within a few lines, the firmware prints `# I2C error on RAW_ANGLE: ...`, then `# SENSOR OFFLINE after 3 consecutive errors. Recovering every 500 ms ...`.
   - The stream pauses, and a `# still offline (attempt N) ...` line appears every 10 attempts (about 5 s).
   - The console stays responsive: type `help` and it answers. The board does **not** hang or reset.
3. **Reconnect the wire.**
   **Pass:** within about 0.5 s, `# SENSOR BACK ONLINE after X ms (N recovery attempts).` appears, followed by the magnet verdict and a new stream header. Streaming resumes with valid angles.
4. Repeat with **only SCL** pulled, then with **VCC** pulled (the full power cycle a real unplug causes). Do one of the runs while **rotating the magnet while disconnected**. The firmware warns that turns made while disconnected are unknown. `deg` is correct again immediately; `unwrapped` continues by the shortest path.
5. Afterwards, `diag` shows `errors N | recoveries M`. M should equal the number of reconnects.

> Pull and reconnect **one wire at a time**, and never move the VCC wire onto a different header pin while the board is powered. Plugging VCC into 5V by mistake can damage this 3.3 V-only module. Make sure no loose wire end can touch the module's **OUT** pin and ground it.

---

## 7. What to record

Copy this table into the test log, one copy per sensor and magnet setup.

| Item | Value |
|---|---|
| Date / tester / firmware version (boot banner, e.g. `v1.0.0`) | |
| Module (WWZMDiB AS5600, board label/number), magnet part number, size, magnetization (diametric?) | |
| Optional resistance check: SCL→VCC, SDA→VCC, GPO→GND, DIR→VCC/GND (Ω); R4 still fitted? | |
| DIR wired to GND? External pull-ups added? | |
| Air gap (mm) and how it was measured; axis offset if known; wire length | |
| **Step 1:** scan result (addresses found), probe ACK? | |
| **Step 2:** magnet verdict; AGC; MAGNITUDE; AGC min/max over a full turn | |
| **Step 2:** ZMCO / ZPOS / MPOS / MANG / CONF from `diag` | |
| **Step 3:** direction (does clockwise from the magnet side increase?); any jumps or stuck values? | |
| **Step 3 (optional):** error at 90/180/270° marks | |
| **Step 4:** noise at 3 angles: std dev (counts, °), p-p (counts, °), mean raw | |
| **Step 4:** `speed`: µs/read and reads/s for each method; fast-read consistent? | |
| **Step 5:** unwrapped after +2 turns, after −3 turns, back at start (drift?) | |
| **Step 6:** for each wire pulled (SDA, SCL, VCC): time to go offline, time to recover (`after X ms`), any hang or reset? | |
| Stream lines dropped / UART errors (from `diag`) | |
| Notes: anything odd, plus a CSV log file name if one was captured | |

**Capturing a CSV log:** run `stream off`, `csv on`, `rate 50`, start the terminal's logging (PuTTY: Session → Logging → "All session output"), then `stream on`. When done, strip the `#` lines and echoed commands, e.g. `grep -E '^[0-9]' putty.log > run1.csv`.

---

## 8. Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| No output at all on the terminal | Wrong COM port or baud rate. Press RESET. Check that the board has been flashed (Debug: did you press Resume?) |
| `Probe 0x36: NACK` | No power (measure about 3.3 V from VCC to GND at the module), SDA/SCL swapped, or a loose jumper wire |
| `BUS BUSY` / `bus recovery: STILL NOT FREE` | A line is shorted to GND, or the module is unpowered (its pull-ups lose power and the lines sag). Check VCC and the wiring |
| Occasional `I2C error` or `BAD DATA` | Slow edges at 400 kHz. Shorten the wires and add 2.2 to 4.7 kΩ pull-ups to 3V3. Or drop to 100 kHz: in CubeMX set I2C1 → **I2C Speed Mode = Standard Mode**, regenerate and rebuild. The boot banner's TIMINGR then changes |
| Direction flips, or the angle is erratic even with the magnet OK | DIR is floating. Wire DIR to GND |
| `diag` shows ZMCO ≠ 0, or ZPOS/MPOS ≠ 0 | The part has been programmed, either by the seller or through the OUT pin (see the [warning](#1-about-the-module)). RAW ANGLE, and so this test, is unaffected. Record it |
| `Magnet NOT DETECTED` with the magnet present | Axially magnetized magnet lying flat, gap above about 3 mm, or the magnet far off-center |
| `deg` decreases while turning clockwise | DIR is not actually at GND (loose wire), or you are viewing from the PCB's back side. Clockwise is defined looking at the magnet side |
| Angle error that varies sinusoidally around the turn | Magnet off-axis (> 0.25 mm) or tilted. Re-center it over the chip package |
| `# N stream line(s) dropped` | The rate is too high for 115200 baud. Use `rate 50` or lower in table mode |

**Never** program the sensor's one-time memory: not with another tool or library (BURN_ANGLE or BURN_SETTING written to register 0xFF), and not by grounding OUT on this board (programming option B). It is permanent and cannot be undone.

---

## 9. Project layout and regenerating from CubeMX

```
firmware/as5600_bench/
├── as5600_bench.ioc        CubeMX config: STM32L476RGTx, 80 MHz (HSI16 -> PLL), I2C1 PB8/PB9 400 kHz,
│                           USART2 115200 + IRQ, TIM2 1 MHz free-running (us timestamps)
├── Core/Inc, Core/Src
│   ├── as5600.c/.h         AS5600 driver: instance-based, read-only, timeouts, bus recovery, unwrap  <- reusable
│   ├── serial_io.c/.h      Interrupt-driven UART TX ring buffer and RX; defines the HAL UART callbacks
│   ├── enc_cli.c/.h        Test application: boot checks, 200 Hz tracker, stream, commands
│   └── main.c              CubeMX-generated; three USER CODE hooks (below)
├── Drivers/                HAL and CMSIS copied by CubeMX
└── STM32L476RGTX_FLASH.ld  Linker script
```

Everything custom is either in its own file or inside `USER CODE` blocks, so **Project Manager → Generate Code** in CubeMX does not overwrite it. The only edits to `main.c` are these:

```c
/* USER CODE BEGIN Includes */
#include "enc_cli.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
enc_app_init(&hi2c1, &huart2, &htim2);
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */   (inside while (1))
enc_app_poll();
/* USER CODE END 3 */
```

If you regenerate, keep the following in the `.ioc`:
- The **PB8/PB9 user labels** `AS5600_SCL` and `AS5600_SDA`. The driver's bus recovery uses the `AS5600_SCL_Pin` and `AS5600_SDA_Pin` defines generated from them.
- **USART2 global interrupt enabled.**
- TIM2 with **Prescaler 79, Period 4294967295**.

Do not add your own `HAL_UART_*Callback` functions; `serial_io.c` defines them.

**Using the driver in the final two-sensor firmware:** make one `as5600_t` per bus, each initialised with its own `I2C_HandleTypeDef*` and SCL/SDA pins. Call `as5600_update()` from the control loop and `as5600_service()` from the main loop. Read `as5600_is_online()` before trusting a sample.
