# weact-h723-w5500 (geplant)

Build für das WeAct-MiniSTM32H723 mit W5500-Ethernet-Modul (UDP statt SPI zum
Host). Noch nicht umgesetzt.

Der Port braucht nur eine andere Transportschicht: 80-Byte-Frames per UDP
empfangen, `cnc_core_frame()` aufrufen, Antwort zurückschicken. Schritterzeugung,
Pinbelegung und Zustandslogik bleiben wie in `../weact-h723-spi`. Auf der
LinuxCNC-Seite gehört dazu ein Treiber `cncboard_udp` neben `cncboard_spi`.
