# BLE Channel Sounding RTLS with nRF54L15

This repository contains a two-dimensional indoor Real-Time Location System
based on Bluetooth Low Energy Channel Sounding and Nordic Semiconductor
nRF54L15 development kits.

The final system consists of:

- three Channel Sounding initiators acting as fixed anchors;
- one multi-link Channel Sounding reflector acting as the mobile tag;
- three USB serial connections between the anchors and the host computer;
- a Python host application for measurement coordination, calibration,
  filtering, trilateration, visualization, validation, and CSV logging.

## Repository structure

| Directory | Purpose | Status |
|---|---|---|
| `channel_sounding_ras_anchor` | Final anchor firmware and Python RTLS application | Final system |
| `channel_sounding_ras_tag` | Three-link mobile reflector firmware | Final system |
| `channel_sounding_ras_initiator` | Original single-initiator prototype | Preliminary prototype |
| `channel_sounding_ras_reflector` | Original single-reflector prototype | Preliminary prototype |

The final RTLS is implemented by `channel_sounding_ras_anchor` and
`channel_sounding_ras_tag`. The other two applications are retained to
document the initial project phase.

## Hardware configuration

The final system requires four nRF54L15 development kits:

| Device | Role | Coordinate |
|---|---|---|
| Anchor 0 | CS Initiator | `(1.50, 0.00) m` |
| Anchor 1 | CS Initiator | `(0.00, 0.00) m` |
| Anchor 2 | CS Initiator | `(0.75, 1.20) m` |
| Mobile tag | Multi-link CS Reflector | Variable |

All coordinates refer to the antenna reference points. For two-dimensional
trilateration, the tag and anchor antennas should be placed at approximately
the same height.

## Requirements

- Four Nordic nRF54L15 DK boards
- nRF Connect SDK v2.9.2
- An nRF Connect SDK terminal with the matching toolchain active
- Python 3
- Python packages:
  - `numpy`
  - `matplotlib`
  - `pyserial`

Install the host dependencies in a virtual environment if required:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install numpy matplotlib pyserial
```

## Define local paths

Clone the repository and define its location:

```bash
git clone https://github.com/gaiarina11/Innovative-Wireless-Project-Gaia-Rina.git
cd Innovative-Wireless-Project-Gaia-Rina

export RTLS_REPO="$PWD"
export NCS_WORKSPACE="/opt/nordic/ncs/v2.9.2"
```

Change `NCS_WORKSPACE` if the nRF Connect SDK is installed elsewhere.

All `west` commands must be executed from an initialized NCS workspace or
from an nRF Connect SDK terminal. Running `west` outside the workspace can
produce `unknown command "flash"`.

## Build the final firmware

Build the multi-link tag:

```bash
cd "$NCS_WORKSPACE"

west build --sysbuild -p always \
  -b nrf54l15dk/nrf54l15/cpuapp \
  -d "$RTLS_REPO/channel_sounding_ras_tag/build_tag" \
  "$RTLS_REPO/channel_sounding_ras_tag"
```

Build the three anchor identities:

```bash
west build --sysbuild -p always \
  -b nrf54l15dk/nrf54l15/cpuapp \
  -d "$RTLS_REPO/channel_sounding_ras_anchor/build_anchor_0" \
  "$RTLS_REPO/channel_sounding_ras_anchor" \
  -- -DANCHOR_ID=0

west build --sysbuild -p always \
  -b nrf54l15dk/nrf54l15/cpuapp \
  -d "$RTLS_REPO/channel_sounding_ras_anchor/build_anchor_1" \
  "$RTLS_REPO/channel_sounding_ras_anchor" \
  -- -DANCHOR_ID=1

west build --sysbuild -p always \
  -b nrf54l15dk/nrf54l15/cpuapp \
  -d "$RTLS_REPO/channel_sounding_ras_anchor/build_anchor_2" \
  "$RTLS_REPO/channel_sounding_ras_anchor" \
  -- -DANCHOR_ID=2
```

## Identify and flash the boards

List the connected Nordic boards:

```bash
nrfutil device list
```

Assign the four reported device identifiers:

```bash
export TAG_DEVICE_ID="replace-with-tag-device-id"
export ANCHOR0_DEVICE_ID="replace-with-anchor-0-device-id"
export ANCHOR1_DEVICE_ID="replace-with-anchor-1-device-id"
export ANCHOR2_DEVICE_ID="replace-with-anchor-2-device-id"
```

Flash the boards:

```bash
cd "$NCS_WORKSPACE"

west flash \
  -d "$RTLS_REPO/channel_sounding_ras_tag/build_tag" \
  --dev-id "$TAG_DEVICE_ID"

west flash \
  -d "$RTLS_REPO/channel_sounding_ras_anchor/build_anchor_0" \
  --dev-id "$ANCHOR0_DEVICE_ID"

west flash \
  -d "$RTLS_REPO/channel_sounding_ras_anchor/build_anchor_1" \
  --dev-id "$ANCHOR1_DEVICE_ID"

west flash \
  -d "$RTLS_REPO/channel_sounding_ras_anchor/build_anchor_2" \
  --dev-id "$ANCHOR2_DEVICE_ID"
```

Device identifiers are intentionally not stored in this public repository
because they are specific to one physical setup.

## Identify the serial ports

List the available serial interfaces:

```bash
python3 -m serial.tools.list_ports -v
```

On macOS, the ports normally resemble `/dev/cu.usbmodem...`. On Linux, they
normally resemble `/dev/ttyACM...`.

Assign the ports according to the firmware identity programmed on each board:

```bash
export ANCHOR0_PORT="/dev/cu.usbmodem-replace-anchor-0"
export ANCHOR1_PORT="/dev/cu.usbmodem-replace-anchor-1"
export ANCHOR2_PORT="/dev/cu.usbmodem-replace-anchor-2"
```

The USB port order is not necessarily the same as the anchor identifier.
Always verify the `ANCHOR` field in the serial output.

## Run the final real-time tracking system

The following command uses the final Mode 3/PBR calibration and the
known-start branch-continuity configuration used for the report:

```bash
python3 \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/trilateration_network.py" \
  --port 0="$ANCHOR0_PORT" \
  --port 1="$ANCHOR1_PORT" \
  --port 2="$ANCHOR2_PORT" \
  --config \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/rtls_config_mode3_pbr.json" \
  --distance-source phase \
  --pbr-branch-tracking 0.75,0.85 \
  --pbr-branch-max-step 0.40 \
  --distance-filter-window 5 \
  --outlier-sigma 4 \
  --outlier-floor 0.20 \
  --position-alpha 0.25 \
  --max-age 6 \
  --max-skew 4 \
  --csv-log \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/measurements/new_run.csv"
```

Before starting this command, physically place the antenna reference point of
the tag at `(0.75, 0.85) m`.

The `--pbr-branch-tracking` option uses this known starting coordinate to align
the PBR branches. It is intended for tracking demonstrations and must not be
used to claim independent cold-start localization accuracy.

The Matplotlib live plot is enabled by default. Add `--no-plot` when only
terminal and CSV output are required.

## Independent validation

For an independent accuracy experiment:

1. select a position that was not used for calibration;
2. measure the physical antenna-reference coordinates;
3. keep the tag stationary;
4. do not enable PBR branch tracking;
5. record both successful and failed cold starts.

Example:

```bash
python3 \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/trilateration_network.py" \
  --port 0="$ANCHOR0_PORT" \
  --port 1="$ANCHOR1_PORT" \
  --port 2="$ANCHOR2_PORT" \
  --config \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/rtls_config_mode3_pbr.json" \
  --distance-source phase \
  --validation-point X,Y \
  --validation-samples 100 \
  --validation-warmup 10 \
  --max-age 6 \
  --max-skew 4 \
  --no-plot
```

Replace `X,Y` with the measured validation coordinate.

If cold-start PBR selects an incorrect phase branch, the failure must be
reported. It must not be corrected using the known validation position.

## Regenerate the report figures

The report figures are generated offline from saved CSV files. Their labels
and graphical style can therefore be changed without repeating the physical
measurements.

Define a writable Matplotlib configuration directory if necessary:

```bash
export RTLS_MPL_CONFIG="${TMPDIR:-/tmp}/rtls-mplconfig"
mkdir -p "$RTLS_MPL_CONFIG"
```

Generate the static-stability figure:

```bash
MPLCONFIGDIR="$RTLS_MPL_CONFIG" python3 \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/plot_rtls_report.py" \
  --csv \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/measurements/report_statico.csv" \
  --config \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/rtls_config_mode3_pbr.json" \
  --kind static \
  --ground-truth 0.75,0.85 \
  --output \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/report/stabilita_tag.png"
```

Generate the trajectory figure:

```bash
MPLCONFIGDIR="$RTLS_MPL_CONFIG" python3 \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/plot_rtls_report.py" \
  --csv \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/measurements/report_traiettoria.csv" \
  --config \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/rtls_config_mode3_pbr.json" \
  --kind trajectory \
  --output \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/report/traiettoria_tag.png"
```

Generate the trilateration-circle figure:

```bash
MPLCONFIGDIR="$RTLS_MPL_CONFIG" python3 \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/plot_rtls_report.py" \
  --csv \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/measurements/report_statico.csv" \
  --config \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/rtls_config_mode3_pbr.json" \
  --kind ranging \
  --output \
  "$RTLS_REPO/channel_sounding_ras_anchor/src/scripts/report/trilaterazione_circonferenze.png"
```

## Run the host-side test suite

The tests do not require connected boards:

```bash
cd "$RTLS_REPO"

python3 -m unittest discover \
  -s channel_sounding_ras_anchor/src/scripts/tests \
  -v
```

At the software revision used for the final report, all 28 discovered tests
passed. The test suite covers:

- serial record parsing;
- Mode 3 PBR and RTT data;
- token scheduling;
- anchor synchronization;
- calibration constraints;
- robust distance filtering;
- PBR branch tracking;
- trilateration solver behaviour;
- CSV logging;
- validation metrics.

Firmware compilation and on-air measurements remain separate verification
activities.

## Preliminary single-link prototype

The preliminary architecture used one initiator, one reflector, and a
single serial port. It is retained for comparison with the final distributed
three-anchor architecture.

After programming the preliminary initiator and reflector, run:

```bash
python3 \
  "$RTLS_REPO/channel_sounding_ras_initiator/src/scripts/trilateration.py" \
  --port /dev/cu.usbmodem-replace-with-initiator-port \
  --baud 115200
```

This prototype is not the final RTLS architecture.

## Known limitations

- PBR can select an incorrect phase branch after a cold start.
- The tracking demonstration requires a known initial coordinate.
- RTT currently has significant anchor-dependent bias and is used primarily
  as a diagnostic measurement.
- The three Channel Sounding procedures are serialized, reducing the global
  update rate.
- Indoor multipath, antenna orientation, and nearby objects affect the
  distance estimates.
- A low trilateration residual indicates geometric consistency, not
  necessarily low absolute position error.

## Report results

The repository contains the CSV datasets and plotting software used to
produce the figures included in the project report. Branch-tracked results
must be interpreted as tracking and stability demonstrations rather than
independent cold-start validation.
