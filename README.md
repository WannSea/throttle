# WannSea Gashebel Firmware

Dieses Projekt nutzt das https://github.com/rp-rs/rp2040-project-template als Template.

## Installation

Rust über https://rustup.rs/ installieren und anschließend ausführen:

```sh
rustup target install thumbv6m-none-eabi
cargo install flip-link
cargo install --locked elf2uf2-rs
```

## Build

Es gibt zwei Möglichkeiten:

1. Den RP2040 mit gedrückter BOOTSEL-Taste per USB verbinden und die Firmware direkt bauen und flashen:

   ```sh
   cargo run
   ```

2. Eine UF2-Datei bauen, die später manuell auf das RP2040-USB-Laufwerk kopiert werden kann:

   ```sh
   cargo build --release
   elf2uf2-rs target/thumbv6m-none-eabi/release/rp2040-project-template throttle.uf2
   ```


## Info:

Green LED Blinking: Sending CAN throttle command
Red LED: Lever Locked

## Encoder / Hall

Der verwendete Encoder mit dem 5-poligen Stecker ist ein AS5600 und wird über I2C gelesen:

- Board-Stecker: `SDA`, `SCL`, `PWM/OUT`, `GND`, `3V3`
- Am AS5600: `VCC -> 3V3`, `GND -> GND`, `SDA -> SDA`, `SCL -> SCL`
- `DIR` habe ich fest mit `VCC` verbunden.
- `OUT/PWM` wird von der Firmware aktuell nicht benutzt.

## Hendriks Version

| Hendriks Originalversion | Aktuelle Änderungen |
|---|---|
| Aktualisierungsintervall: 100 ms / 10 Hz | Aktualisierungsintervall: 20 ms / 50 Hz |
| Glättungsfenster: 10 Werte | Glättungsfenster: 50 Werte |
| Nominelle vollständige Rampe: ca. 1 Sekunde | Nominelle vollständige Rampe: ca. 1 Sekunde |
| Bei Gasstellung null wird die gesamte Schleife übersprungen; es wird kein CAN-Frame gesendet | Null wird in die Glättung aufgenommen und weiterhin regelmäßig gesendet |
| Bei gezogenem Kill-Cord bzw. gesperrtem Motor wird kein expliziter Nullwert gesendet | Es wird sofort null gesendet und der Glättungsverlauf gelöscht |
| Keine wirksame Abwärtsrampe, wenn der Eingang exakt null wird | Lineare Abwärtsrampe über den gleitenden Mittelwert in ca. 1 Sekunde |
| Der alte Glättungsverlauf bleibt nach einer Sperre erhalten | Nach einer Sperre beginnt die Glättung mit einem leeren Verlauf |
| VESC `SetDuty`, vorzeichenbehaftet ±100.000 | Unverändert: VESC `SetDuty`, vorzeichenbehaftet ±100.000 |
| Ausgabe mit 10 Hz, aber nur bei einem Wert ungleich null | Feste Ausgabe mit 50 Hz, einschließlich null |
