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

Green LED Blinking: Sending Duty
Red LED: Lever Locked

## Encoder / Hall kalibrieren

Der weiße Encoder auf dem 5-poligen Stecker ist ein AS5600 und wird über I2C gelesen:

- Board-Stecker: `SDA`, `SCL`, `PWM/OUT`, `GND`, `3V3`
- Am AS5600: `VCC -> 3V3`, `GND -> GND`, `SDA -> SDA`, `SCL -> SCL`
- `DIR` hast du fest mit `VCC` verbunden. Die Software-Richtung wird trotzdem in `src/main.rs` über `ENCODER_FORWARD_SIGN` eingestellt.
- `OUT/PWM` wird von der Firmware aktuell nicht benutzt.

Kalibrierung in `src/main.rs`:

1. Firmware flashen: `cargo run`.
2. Im Log `encoder raw: ...` ansehen.
3. Gashebel in Null-/Ruheposition halten und diesen Wert bei `ENCODER_ZERO_RAW` eintragen.
4. Gashebel in die gewünschte Vorwärtsrichtung bewegen:
   - Wenn `offset` positiv wird: `ENCODER_FORWARD_SIGN = 1`
   - Wenn `offset` negativ wird: `ENCODER_FORWARD_SIGN = -1`
5. Am Vorwärts-Endanschlag den Betrag von `offset` bei `ENCODER_FORWARD_MAX_COUNTS` eintragen.
6. Am Rückwärts-Endanschlag den Betrag von `offset` bei `ENCODER_REVERSE_MAX_COUNTS` eintragen.
7. `ENCODER_DEADZONE_COUNTS` ist die Nullzone. Wenn der Hebel in Ruhe nicht sauber 0 bleibt, etwas größer machen.

Der 3-polige Hall-Stecker ist `HALL`, `GND`, `5V` und liegt in der Firmware auf `pins.a3`. Er wird aktuell nur als Kill-/Freigabe-Signal benutzt, nicht als Drehzahl- oder Richtungssensor. Wichtig: Die `HALL`-Leitung geht zum RP2040 und hat einen 3.3V-Pullup; der neue Sensor sollte deshalb einen Open-Collector/Open-Drain-Ausgang oder einen 3.3V-kompatiblen Ausgang haben, nicht aktiv 5V auf `HALL` treiben. Wenn der neue Hall-Sensor umgekehrt schaltet, ändere in `src/main.rs` nur:

```rust
const HALL_PRESENT_WHEN_LOW: bool = true;
```

auf `false`.
