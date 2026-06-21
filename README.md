# WannSea Gashebel Firmware

Dieses Projekt nutzt das https://github.com/rp-rs/rp2040-project-template als Template.

<!-- Requirements -->
<details open="open">
  <summary><h2 style="display: inline-block" id="requirements">Requirements</h2></summary>
  
- The standard Rust tooling (cargo, rustup) which you can install from https://rustup.rs/

- Toolchain support for the cortex-m0+ processors in the rp2040 (thumbv6m-none-eabi)

- (by default) A [`probe-rs` installation](https://probe.rs/docs/getting-started/installation/)

- A [`probe-rs` compatible](https://probe.rs/docs/getting-started/probe-setup/) probe

  You can use a second
  [Pico as a CMSIS-DAP debug probe](debug_probes.md#raspberry-pi-pico). Details
  on other supported debug probes can be found in
  [debug_probes.md](debug_probes.md)

</details>

<!-- Installation of development dependencies -->
<details open="open">
  <summary><h2 style="display: inline-block" id="installation-of-development-dependencies">Installation of development dependencies</h2></summary>

```sh
rustup target install thumbv6m-none-eabi
cargo install flip-link
# Installs the probe-rs tools, including probe-rs run, our recommended default runner
cargo install --locked probe-rs-tools
# If you want to use elf2uf2-rs instead, do...
cargo install --locked elf2uf2-rs
```
If you get the error ``binary `cargo-embed` already exists`` during installation of probe-rs, run `cargo uninstall cargo-embed` to uninstall your older version of cargo-embed before trying again.

</details>


## Build

`cargo run`


## Info:

Green LED Blinking: Sending CAN throttle command
Red LED: Lever Locked

## Encoder / Hall kalibrieren

Der verwendete Encoder mit dem 5-poligen Stecker ist ein AS5600 und wird über I2C gelesen:

- Board-Stecker: `SDA`, `SCL`, `PWM/OUT`, `GND`, `3V3`
- Am AS5600: `VCC -> 3V3`, `GND -> GND`, `SDA -> SDA`, `SCL -> SCL`
- `DIR` habe ich fest mit `VCC` verbunden. Die Software-Richtung wird trotzdem in `src/main.rs` über `ENCODER_FORWARD_SIGN` eingestellt.
- `OUT/PWM` wird von der Firmware aktuell nicht benutzt.

Kalibrierung in `src/main.rs`:

1. Firmware flashen: `cargo run`.
2. USB-Serial-Log öffnen und `encoder raw: ...` ansehen. Unter Linux z.B.:
   ```sh
   ls /dev/ttyACM*
   picocom -b 115200 /dev/ttyACM0
   ```
   Falls der Port nicht `/dev/ttyACM0` ist, den neu erschienenen `/dev/ttyACM...` Port nehmen. Die Baudrate ist bei USB-Serial praktisch egal, `115200` passt aber für die meisten Terminalprogramme.
3. Gashebel in Null-/Ruheposition halten und diesen Wert bei `ENCODER_ZERO_RAW` eintragen.
4. Gashebel in die gewünschte Vorwärtsrichtung bewegen:
   - Wenn `offset` positiv wird: `ENCODER_FORWARD_SIGN = 1`
   - Wenn `offset` negativ wird: `ENCODER_FORWARD_SIGN = -1`
5. Am Vorwärts-Endanschlag den Betrag von `offset` bei `ENCODER_FORWARD_MAX_COUNTS` eintragen.
6. Am Rückwärts-Endanschlag den Betrag von `offset` bei `ENCODER_REVERSE_MAX_COUNTS` eintragen.
7. `ENCODER_DEADZONE_COUNTS` ist die symmetrische Nullzone um 0: bei `35` gilt also `-35..+35` als 0. Wenn der Hebel in Ruhe nicht sauber 0 bleibt, etwas größer machen.

Der 3-polige Hall-Stecker ist `HALL`, `GND`, `5V` und liegt in der Firmware auf `pins.a3`. Er wird aktuell nur als Kill-/Freigabe-Signal benutzt, nicht als Drehzahl- oder Richtungssensor. Wichtig: Die `HALL`-Leitung geht zum RP2040 und hat einen 3.3V-Pullup; der neue Sensor sollte deshalb einen Open-Collector/Open-Drain-Ausgang oder einen 3.3V-kompatiblen Ausgang haben, nicht aktiv 5V auf `HALL` treiben. Wenn der neue Hall-Sensor umgekehrt schaltet, ändere in `src/main.rs` nur:

```rust
const HALL_PRESENT_WHEN_LOW: bool = true;
```

auf `false`.

## Beschleunigung tunen

Der Sendemodus wird in `src/main.rs` gesetzt:

```rust
const DRIVE_MODE: DriveMode = DriveMode::Duty;
```

Optionen:

- `DriveMode::Duty`: alter Duty-Modus wie in `.inspiration_files/old_main.rs`
- `DriveMode::CurrentAndRpm`: Current/RPM-Modus wie die AI-Boot-Skripte

Im Duty-Modus ist bei VESC-ID `22` die Extended-ID `0x00000016`, weil `SET_DUTY = 0` und `0 << 8 | 22 = 0x16`. Payload ist Duty `* 100_000` als `i32` big-endian, mit dem 10er-Glättungsfenster aus der alten Firmware.

Im Current/RPM-Modus sendet die Firmware:

- kleines Gas: `SET_CURRENT`, also Drehmoment/Strom zum sauberen Anfahren
- danach: `SET_RPM`, also ERPM-Ziel mit Rampe zum ruhigeren Beschleunigen

- `SET_CURRENT`: `0x00000116`, Payload = Ampere `* 1000` als 4 Byte big-endian
- `SET_RPM`: `0x00000316`, Payload = ERPM als 4 Byte big-endian

Nur diese Tuning-Parameter in `src/main.rs` anfassen:

```rust
const MAX_FORWARD_CURRENT_A: f32 = 80.0;
const MAX_REVERSE_CURRENT_A: f32 = 35.0;
const MAX_FORWARD_ERPM: f32 = 15_000.0;
const MAX_REVERSE_ERPM: f32 = 6_000.0;
const FORWARD_CURRENT_RAMP_A_PER_S: f32 = 150.0;
const REVERSE_CURRENT_RAMP_A_PER_S: f32 = 70.0;
const FORWARD_RPM_RAMP_ERPM_PER_S: f32 = 25_000.0;
const REVERSE_RPM_RAMP_ERPM_PER_S: f32 = 8_000.0;
const RPM_MODE_MIN_THROTTLE: f32 = 0.08;
```

Tuning-Reihenfolge:

1. `MAX_FORWARD_CURRENT_A` zuerst niedrig lassen, z.B. 50-80 A. Danach schrittweise erhöhen. Für den 300A-Bereich nur hochgehen, wenn Kabel, Akku, VESC, Kühlung und Propeller das sicher können.
2. `MAX_REVERSE_CURRENT_A` deutlich niedriger wählen, wenn Rückwärtsfahrt sanfter sein soll.
3. `MAX_FORWARD_ERPM` und `MAX_REVERSE_ERPM` auf die gewünschte Maximaldrehzahl setzen.
4. Wenn das Boot beim Anfahren vorwärts zu träge ist: `FORWARD_CURRENT_RAMP_A_PER_S` erhöhen. Wenn es rupft: senken.
5. Für Rückwärtsfahrt entsprechend `REVERSE_CURRENT_RAMP_A_PER_S` niedriger lassen.
6. Wenn die Beschleunigung im Fahrbereich zu weich ist: die passende `*_RPM_RAMP_ERPM_PER_S` erhöhen. Wenn sie pumpt oder nervös wirkt: senken.
7. `RPM_MODE_MIN_THROTTLE` nur ändern, wenn der Übergang zwischen Anfahren und RPM-Regelung schlecht ist. Größer = länger Strommodus, kleiner = früher RPM-Modus.
