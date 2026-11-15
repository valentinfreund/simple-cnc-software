# Firmware

```
lib/                      boardübergreifender Code (kein Hardwarezugriff)
  cncproto.h              Protokoll Host <-> Board (auch vom LinuxCNC-Treiber benutzt)
  stepgen.[ch]            Schritt/Richtungs-Erzeugung in einen Sample-Puffer
  cnc_core.[ch]           Zustandsautomat, Watchdog, Not-Halt, Protokoll,
                          Wegpunktfolger mit Taktausgleich
boards/
  weact-h723-spi/         fertiger Build: WeAct MiniSTM32H723, SPI zum Raspberry Pi
  weact-h723-w5500/       geplant: gleiches Board, Ethernet über W5500
tests/                    Tests der Bibliothek auf dem PC (ohne Hardware)
```

## Bauen

```bash
make -C firmware/tests                       # Bibliothek auf dem PC testen
make -C firmware/boards/weact-h723-spi       # Firmware bauen
make -C firmware/boards/weact-h723-spi flash # über USB-C flashen (DFU)
```

Benötigt werden die GNU-Arm-Toolchain (`arm-none-eabi-gcc`) und das Paket
STM32CubeH7. Pfade lassen sich mit `CROSS=...` und `CUBE_H7=...` übergeben,
siehe Makefile.

## Ein neues Board anlegen

Ein Board-Build ist ein Ordner unter `boards/` mit eigenem Makefile. Er muss

1. die Samples aus `cnc_core_render()` mit fester Abtastrate auf die
   Schritt/Richtungs-Pins ausgeben (Timer + DMA, FPGA-Schieberegister, ...),
2. jeden empfangenen 80-Byte-Frame an `cnc_core_frame()` geben und den
   zurückgelieferten Statusframe an den Host schicken,
3. die fünf Funktionen `cnc_board_*()` aus `cnc_core.h` bereitstellen
   (Eingänge lesen, Ausgänge, Analogwerte und Servo-Freigabe schreiben,
   Abspielposition im Ausgabepuffer melden).

`cnc_core_render()` und `cnc_core_frame()` dürfen sich nicht gegenseitig
unterbrechen (gleiche Interrupt-Priorität).
