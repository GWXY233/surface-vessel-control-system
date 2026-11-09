# Surface vessel control system

An Arduino Nano carrier board for a single brushed propulsion motor and two linked rudder servos. The board integrates a DRV8833 driver, encoder inputs, and a CLK/DIO four-digit display connector. One external buck converter supplies the board from a 2S battery.

The current hardware is RevL1 with corrected J4 silkscreen, a 69 × 54 mm outline, and four 3.5 mm mounting holes. The current V5.4 firmware targets 595 rpm and automatically starts after a valid steering receiver signal. CH1 controls steering. The power switch stops the system.

**V5.4 caps the numeric display at 600 rpm. A displayed value of 600 can represent a higher raw measurement. Motor regulation, serial diagnostics, and the 610 rpm shutdown threshold use the raw encoder feedback. Use raw logs or independent measurements to evaluate actual peak speed.**

Start with the [Chinese guide](README.md), [wiring](docs/wiring.md), [assembly](docs/assembly.md), and [known issues](docs/known-issues.md). The editable EasyEDA project and BOM are in `hardware/revl1/`. The original L298N single-servo implementation is retained in `legacy/l298n-single-servo/` for comparison.

The archived PCB passed its native DRC on 2026-10-21. Independent import restoration, the reported XT30 pin-fit issue, thermal performance, power transients, and sustained current capability remain to be validated. A zero DRC count does not establish those properties.

Original project material is released under the MIT License. See [third-party notices](THIRD_PARTY.md) for external libraries and bundled CAD resources.
