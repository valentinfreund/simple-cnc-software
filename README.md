# simple-cnc-software

Motorsteuerung für eine DIY-CNC-Fräse: **LinuxCNC** plant die Bahn, ein
**Echtzeitboard** (STM32H723) erzeugt die Schritt/Richtungs-Signale für die
Servotreiber und liest Schalter ein.

```
LinuxCNC (Raspberry Pi / PC)                 Echtzeitboard
  G-Code, Bahnplanung, Oberfläche   <---->     Schritterzeugung (500 kHz Abtastrate)
  HAL-Treiber cncboard_spi         64-Byte-    Ein-/Ausgänge, Not-Halt, Watchdog
  1 Frame pro Millisekunde         Frames
```

| Ordner | Inhalt |
|---|---|
| `firmware/lib/` | boardübergreifende Bibliothek: Protokoll, Schritterzeugung, Zustandslogik |
| `firmware/boards/` | fertige Builds pro Board und Schnittstelle (`weact-h723-spi`, geplant: `weact-h723-w5500`) |
| `firmware/tests/` | Tests der Bibliothek auf dem PC |
| `linuxcnc/hal/` | HAL-Treiber für LinuxCNC |
| `linuxcnc/configs/` | Beispielkonfiguration für eine Portalfräse (X, Y1/Y2, Z) |
| `docs/` | [Anleitung](docs/anleitung.md): Verdrahtung, Flashen, Einrichtung, Inbetriebnahme |
| `legacy/` | die frühere eigene Steuerung (Qt-Host + Firmware V1.1), nicht mehr gepflegt |

## Stand

| Teil | Stand |
|---|---|
| Bibliothek (`firmware/lib`) | auf dem PC getestet (`make -C firmware/tests`) |
| Firmware `weact-h723-spi` | kompiliert, **noch nicht auf Hardware getestet** |
| HAL-Treiber `cncboard_spi` | gegen die LinuxCNC-2.9-Header geprüft, **noch nicht in LinuxCNC gelaufen** |
| Beispielkonfiguration | **ungetestet**, Maschinenwerte sind Platzhalter |
| Spindel (PWM/0–10 V), Ethernet-Build | offen |

Lizenz: GPL-2.0 (siehe `LICENSE`).
