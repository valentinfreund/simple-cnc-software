# Anleitung: Fräse mit LinuxCNC und Echtzeitboard

> **Stand:** Firmware und Treiber sind noch nicht auf echter Hardware
> gelaufen. Diese Anleitung beschreibt den vorgesehenen Aufbau; die
> Inbetriebnahme in Abschnitt 6 ist dafür da, Fehler gefahrlos zu finden.

## 1. Überblick

| Teil | Aufgabe |
|---|---|
| Raspberry Pi mit LinuxCNC | G-Code, Bahnplanung, Oberfläche; schickt jede Millisekunde eine Sollposition pro Achse |
| Echtzeitboard (WeAct MiniSTM32H723, STM32H723VGT6) | fährt von Sollposition zu Sollposition, erzeugt Schritt/Richtung, liest Schalter, überwacht Verbindung und Not-Halt |
| Servotreiber (Puls/Richtung) | regelt den Motor |

Fällt die Verbindung länger als 10 ms aus oder öffnet der Not-Halt-Eingang,
stoppt das Board die Pulse und nimmt die Servo-Freigabe weg. Wieder anlaufen
kann es nur, wenn LinuxCNC die Maschine neu einschaltet.

**Der Not-Halt-Eingang des Boards ersetzt keinen fest verdrahteten Not-Halt.**
Der Not-Halt-Taster muss zusätzlich die Leistungsversorgung der Servotreiber
(oder deren Freigabe) direkt trennen.

## 2. Pinbelegung des Boards (Build `weact-h723-spi`)

Die Belegung steht in `firmware/boards/weact-h723-spi/src/board.h` und lässt
sich dort ändern.

### Verbindung zum Raspberry Pi (3,3 V, Leitungen kurz halten)

| Signal | Board | Raspberry Pi (40-pol. Leiste) |
|---|---|---|
| NSS / CE0 | PA4 | Pin 24 (GPIO 8) |
| SCK | PA5 | Pin 23 (GPIO 11) |
| MISO | PA6 | Pin 21 (GPIO 9) |
| MOSI | PA7 | Pin 19 (GPIO 10) |
| Masse | GND | Pin 25 (GND) |

### Schritt und Richtung (Open-Drain, 5-V-fest)

| Achse | Gelenk | STEP | DIR |
|---|---|---|---|
| X | 0 | PB10 | PB14 |
| Y1 | 1 | PB11 | PB15 |
| Y2 | 2 | PB12 | PB8 |
| Z | 3 | PB13 | PB9 |

Anschluss an den Treiber: Plus-Eingang von Puls und Richtung (PUL+/DIR+) an
+5 V, Minus-Eingang (PUL−/DIR−) an den Board-Pin. Die Masse der 5-V-Quelle
muss mit GND des Boards verbunden sein. Der Pin zieht nach Masse, um den
Optokoppler im Treiber einzuschalten.

Es müssen die **5-V-Pulseingänge** des Treibers benutzt werden. Viele Treiber
haben daneben 24-V-Pulseingänge mit eigenen Klemmen; welche Klemme welche ist,
steht im Handbuch des Treibers.

Pulsform ab Werk: 4 µs Puls, mindestens 4 µs Pause, höchstens 125 kHz,
10 µs zwischen Richtungswechsel und nächstem Puls.

### Eingänge (3,3 V, interner Pull-up, aktiv bei Verbindung nach GND)

| Eingang | Pin | Verwendung in der Beispielkonfiguration |
|---|---|---|
| in.00 | PC0 | Referenzschalter X |
| in.01 | PC1 | Referenzschalter Y1 |
| in.02 | PA0 | Referenzschalter Y2 |
| in.03 | PA1 | Referenzschalter Z |
| in.04 | PA2 | Messtaster |
| in.05 | PA3 | frei |
| in.06 | PC4 | Alarm der Servotreiber |
| in.07 | PC5 | **Not-Halt**: Öffnerkontakt nach GND, offen = Not-Halt |

Die Eingänge vertragen **nur 3,3 V**. 24-V-Schalter und die Alarmausgänge der
Treiber brauchen einen Optokoppler dazwischen. Ohne angeschlossenen
Not-Halt-Kontakt meldet das Board Not-Halt; für Tischversuche PC5 mit GND
brücken.

### Ausgänge (3,3 V, Gegentakt, aktiv high)

| Ausgang | Pin | Verwendung |
|---|---|---|
| Freigabe | PD8 | Servo-Freigabe für alle Treiber |
| out.00 | PD9 | Kühlmittel |
| out.01 | PD10 | Sprühnebel |
| out.02 / out.03 | PD14 / PD15 | frei |
| LED | PE3 | aus = gesperrt, an = läuft, blinkt langsam = Fehler, blinkt schnell = Firmwarefehler |

Die Ausgänge liefern nur 3,3 V und wenige Milliampere. Der Freigabe-Eingang
der Treiber arbeitet üblicherweise mit 24 V; dafür wird eine Treiberstufe
(Optokoppler oder Transistor) gebraucht.

## 3. Servotreiber einstellen

Im Treiber einzustellen (Parameternummern laut Handbuch des Treibers):

- Betriebsart Lageregelung mit Pulseingang, Signalform **Puls + Richtung**
- elektronisches Getriebe auf **4000 Pulse pro Umdrehung**
  (bei 5 mm Spindelsteigung: 800 Pulse/mm, 1,25 µm pro Puls)
- Freigabe über den externen Eingang

Ein anderer Wert für Pulse pro Umdrehung ist möglich; dann `SCALE` in
`fraese.ini` anpassen. Die höchste Pulsfrequenz ist
`Geschwindigkeit [mm/s] × SCALE` und muss unter 125 kHz bleiben.

## 4. Firmware bauen und flashen

```bash
sudo apt install gcc-arm-none-eabi dfu-util
make -C firmware/boards/weact-h723-spi CUBE_H7=/pfad/zu/STM32Cube_FW_H7_V1.12.1
```

Flashen über USB-C: BOOT0-Taste gedrückt halten, Board einstecken (oder kurz
Reset drücken), Taste loslassen. Dann:

```bash
make -C firmware/boards/weact-h723-spi flash
```

## 5. Raspberry Pi einrichten

1. Das LinuxCNC-Image für den Raspberry Pi von linuxcnc.org auf eine SD-Karte
   schreiben und starten.
2. SPI einschalten: in `/boot/firmware/config.txt` die Zeile `dtparam=spi=on`
   eintragen, neu starten. Danach muss `/dev/spidev0.0` vorhanden sein.
3. Treiber installieren:
   ```bash
   sudo apt install linuxcnc-uspace-dev
   make -C linuxcnc/hal install
   ```
4. Konfiguration kopieren:
   ```bash
   mkdir -p ~/linuxcnc/configs
   cp -r linuxcnc/configs/fraese-xyyz ~/linuxcnc/configs/
   ```
5. In `fraese.ini` die Platzhalterwerte an die Maschine anpassen:
   Verfahrwege, Richtung und Geschwindigkeit der Referenzfahrt, `SCALE`.

## 6. Inbetriebnahme (in dieser Reihenfolge)

1. **Nur Board und Pi, keine Treiber angeschlossen.** PC5 mit GND brücken.
   Verbindung prüfen:
   ```bash
   halrun
   loadrt threads name1=servo-thread period1=1000000
   loadrt cncboard_spi
   addf cncboard.update servo-thread
   start
   show pin cncboard
   ```
   `cncboard.ok` muss TRUE sein, `cncboard.comm-errors` darf nicht hochzählen.
   Brücke an PC5 ziehen: `cncboard.estop-active` wird TRUE.
2. **Pulse messen.** In `halrun` weiter:
   ```
   setp cncboard.joint.0.scale 800
   setp cncboard.enable 1
   setp cncboard.joint.0.pos-cmd 10
   ```
   Das Board fährt die 10 mm mit seiner Höchstfrequenz (125 kHz) an: an PB10
   erscheinen 8000 Pulse (mit Pull-up nach 3,3 V oder 5 V messen, der Pin ist
   Open-Drain), `cncboard.joint.0.pos-board` steht danach auf 10.
   `halrun` beenden: die LED muss nach 10 ms langsam blinken (Verbindung weg).
3. **Ein Motor ohne Mechanik** (Kupplung offen): Treiber anschließen,
   LinuxCNC mit `fraese-xyyz` starten, Achse im Handbetrieb langsam fahren.
   Drehrichtung und Weg pro Umdrehung prüfen (10 mm Verfahrweg = 2
   Umdrehungen bei 5 mm Steigung).
4. **Not-Halt und Alarm testen**, bevor die Mechanik angekuppelt wird.
5. **Maschine:** erst Referenzschalter prüfen (`halshow`), dann Referenzfahrt
   mit kleiner Geschwindigkeit, dann Verfahrwege und Portalausrichtung
   (`HOME_OFFSET` von Y1/Y2).

## 7. Zeitverhalten

LinuxCNC schickt jede Millisekunde die Sollposition aller Achsen. Das Board
fährt in seiner eigenen Millisekunde geradlinig von der letzten zur neuen
Sollposition und berechnet die Pulsfrequenz dafür aus seiner exakt bekannten
eigenen Position. Es gibt nichts abzustimmen.

- **Gleichzeitigkeit:** Alle Achsen bekommen ihre Sollposition im selben
  Frame und geben ihre Pulse im selben Abtasttakt aus (ein gemeinsames
  Portregister, 2 µs Raster).
- **Taktausgleich:** Die Uhren von Raspberry Pi und Board laufen nie exakt
  gleich schnell, und der Takt des Pi schwankt. Das Board misst, wann die
  Frames im Mittel ankommen, und richtet sein Millisekundenraster danach aus.
  Einzelne verspätete Frames ändern daran nichts.
- **Verzögerung:** Die ganze Maschine läuft etwa 2 ms hinter den Sollwerten
  her, alle Achsen gleich. Auf die Bahn hat das keinen Einfluss.
- **Referenzfahrt und Antasten (G38):** Beides steuert LinuxCNC auf dem
  Rechner; das Board meldet nur einmal pro Millisekunde den Zustand der
  Schalter. Bis LinuxCNC einen Schaltvorgang sieht, vergehen 1,6 bis 2,6 ms,
  dazu kommen die 2 ms Verzögerung der Bewegung. Die gespeicherte Position
  liegt deshalb um etwa Geschwindigkeit × 4 ms neben der wahren
  Schaltposition (immer in Fahrtrichtung, also wiederholbar) und streut um
  Geschwindigkeit × ±0,5 ms. Bei 2 mm/s sind das 8 µm Versatz und ±1 µm
  Streuung. Deshalb langsam antasten (`HOME_LATCH_VEL`, Vorschub bei G38).
- **Verspätete oder verlorene Frames:** Kommt ein Sollwert mehr als 0,2 ms
  später als erwartet oder gar nicht, fährt das Board mit der aktuellen
  Geschwindigkeit weiter und holt die Abweichung mit dem nächsten Sollwert
  auf. `cncboard.late-commands` zählt diese Fälle. Zählt der Wert im Betrieb
  laufend hoch, ist der Rechner für 1 ms Takt zu unruhig (Latenztest von
  LinuxCNC laufen lassen).
- **Schleppfehler:** `joint.N.f-error` zeigt die vom Board gemessene
  Abweichung von der geplanten Bahn, ohne die konstante Verzögerung.
- `make -C firmware/tests sim` simuliert die Kette auf dem PC und gibt
  Verzögerung, Welligkeit und größte Abweichung aus.
