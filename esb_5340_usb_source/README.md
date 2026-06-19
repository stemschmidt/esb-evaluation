# nRF5340 Audio ESB

USB Audio (UAC2) → cpuapp → IPC → cpunet → ESB

```
USB Host
   │
   │ UAC2 Explicit Feedback
   ▼
┌─────────────────────────────────┐
│ nRF5340                         │
│                                 │
│  cpuapp (Application Core)      │
│  ┌─────────────────────────┐    │
│  │ USB Audio Stack (UAC2)  │    │
│  │ 48kHz / 16bit / Stereo  │    │
│  │ Ringpuffer (8 Frames)   │    │
│  └───────────┬─────────────┘    │
│              │ IPC (RPMsg)      │
│              │ 194 Byte/ms      │
│  ┌───────────▼─────────────┐    │
│  │ cpunet (Network Core)   │    │
│  │ ESB PTX, 2Mbit/s        │    │
│  │ kein ACK, kein Retry    │    │
│  │ Pipe 0, Kanal 10        │    │
│  └─────────────────────────┘    │
└─────────────────────────────────┘
   │
   │ ESB 2.4GHz
   ▼
Empfänger (PRX)
```

## Paketformat

Jedes ESB-Paket hat 194 Byte:

| Byte  | Inhalt                        |
|-------|-------------------------------|
| 0-1   | Sequenznummer (Big Endian)    |
| 2-193 | PCM Audio (192 Byte = 1ms)    |

## Bauen

```bash
west build -b nrf5340dk_nrf5340_cpuapp@2.0.0 . --sysbuild
```

## Flashen

```bash
west flash
```

Oder mit J-Link:
```bash
west flash --runner jlink
```

## Konfiguration anpassen

### ESB Kanal ändern
In `cpunet/src/main.c`:
```c
#define ESB_RF_CHANNEL  10  // 2400 + 10 = 2410 MHz
```

### ESB Adressen ändern
In `cpunet/src/main.c` die Arrays `base_addr_0`, `base_addr_1` und `addr_prefix` anpassen.
Der Empfänger (PRX) muss dieselben Adressen verwenden.

## Nächste Schritte

- [ ] PRX-Empfänger implementieren
- [ ] Jitter-Buffer auf der Empfängerseite
- [ ] Feedback-Mechanismus: Empfänger → cpuapp (USB Explicit Feedback)
- [ ] Kanalauswahl / Frequency Hopping gegen Störer
