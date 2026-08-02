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

## Veränderbare Parameter

Die wichtigsten Parameter stehen am Anfang von `src/main.rs`:

- `MAX_CURRENT_A`: maximal ausgegebener Strom bei vollständig ausgelenktem Gashebel, aktuell `500.0` A.
- `DELAY_MS`: Zeit zwischen zwei Ausgaben. `20` ms entsprechen 50 Hz.
- `ENCODER_ZERO_RAW`: Raw-Wert des Encoders in der neutralen Hebelposition.
- `ENCODER_FORWARD_MAX_RAW`: Raw-Wert bei vollständig nach vorne gedrücktem Gashebel.
- `ENCODER_REVERSE_MAX_RAW`: Raw-Wert bei vollständig nach hinten gedrücktem Gashebel.
- `ENCODER_DEADZONE_COUNTS`: Totzone von der Neutralstellung aus in jede Richtung. `50` bedeutet ungefähr 100 Counts Gesamttotzone.
- `PRETEND_HALL_PIN_ON`: Bei `true` wird der Hall-/Kill-Cord-Eingang für Tests als dauerhaft vorhanden behandelt.
- `ENABLE_USB_LOGGING`: Aktiviert bei `true` die Ausgabe der Encoderwerte über USB-Serial.

Die Encoder-Endwerte dürfen je nach Einbaurichtung des Sensors größer oder kleiner als `ENCODER_ZERO_RAW` sein. Entscheidend ist, dass die Werte an den tatsächlichen Endpositionen des Hebels eingetragen werden.

## USB-Debug-Ausgabe

Bei aktiviertem `ENABLE_USB_LOGGING` wird beispielsweise folgende Zeile ausgegeben:

```text
encoder raw: 2237, throttle_milli: 0, can: 00000116#00000000
```

Die drei Werte bedeuten:

- `encoder raw`: unverarbeiteter 12-Bit-Messwert des AS5600 von `0` bis `4095`.
- `throttle_milli`: aus Kalibrierung und Totzone berechnete Gasstellung. `-1000` entspricht dem einen Endanschlag, `0` der Neutralstellung und `+1000` dem anderen Endanschlag.
- `can`: Vorschau der tatsächlich gesendeten CAN-Nachricht im Format `CAN-ID#Payload`. `00000116` ist die `SetCurrent`-Nachricht für VESC-ID 22. Der achtstellige Payload enthält den vorzeichenbehafteten Stromwert als Ampere × 1.000. Im gesperrten Zustand ist dieser Payload immer `00000000`.


## SetDuty- und SetCurrent-Version

| SetDuty-Version | Aktuelle SetCurrent-Version |
|---|---|
| VESC `SetDuty`, CAN-ID `0x16` | VESC `SetCurrent`, CAN-ID `0x116` |
| Payload: vorzeichenbehafteter Duty-Wert von −100.000 bis +100.000 | Payload: vorzeichenbehafteter Stromwert von −500.000 bis +500.000 (Ampere × 1.000) |
| 50 Werte großes Glättungsfenster | Keine Glättung und keine Rampe |
| Vollständige Rampe dauert ca. 1 Sekunde | Ausgang folgt dem Gashebel unmittelbar |
| Ausgabe alle 20 ms / 50 Hz | Ausgabe alle 20 ms / 50 Hz |
| Null und der gesperrte Zustand werden explizit gesendet | Null und der gesperrte Zustand werden explizit gesendet |
| Encoder- und Hall-Auswertung wie in der aktuellen Version | Encoder- und Hall-Auswertung unverändert |
