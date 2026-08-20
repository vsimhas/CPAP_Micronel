# Therapy session EDF logging (SD card)

## What this is

When therapy starts, CM7 writes a classic **EDF** (European Data Format) file to a
FAT-formatted SD card:

```
0:/SLINK/YYmmddNN.EDF
```

Signals (1 s records, 25 Hz):

| Label    | Meaning              | Unit  |
|----------|----------------------|-------|
| Flow     | Patient flow         | L/min |
| Pressure | Mask pressure        | cmH2O |
| Leak     | Unintentional leak   | L/min |
| TargetP  | Therapy target       | cmH2O |

View with **EDFbrowser**, MATLAB, Python (`pyedflib` / MNE), etc.

## OSCAR — important limitation

**OSCAR will not import these files as-is.**

OSCAR expects a **vendor SD-card layout** (e.g. ResMed `DATALOG/` + `SETTINGS/` +
`Identification.*` + proprietary companion files). Cloning that layout would mean
reproducing another manufacturer’s proprietary card format, which this project
does not do.

Practical paths:

1. Use **EDFbrowser** (or similar) for waveform review today.
2. Later: a **SomnaLink-specific OSCAR machine loader** (open-source plugin) that
   reads `SLINK/*.EDF` — that is the correct long-term OSCAR integration.

## Setup

1. Format the SD card as **FAT32** on a PC (not exFAT if possible).
2. Insert card; wait for SD self-test OK.
3. Optionally set wall time (no RTC on board):
   ```
   therapy edf datetime 26 08 19 22 30 00
   ```
4. `therapy start` … `therapy stop`
5. Check status:
   ```
   therapy edf status
   ```
6. Remove card and open `SLINK\*.EDF` in EDFbrowser.

## USB commands

| Command | Effect |
|---------|--------|
| `therapy edf status` | Enable/state/path/records/error |
| `therapy edf enable 0\|1` | Disable/enable auto-log on therapy start |
| `therapy edf datetime YY MM DD HH MI SS` | Header start time |

## Notes

- Logging soft-fails if the card is missing or not FAT-mountable; therapy still runs.
- Raw “dummy sector” SD test is skipped while EDF logging is enabled (avoids FAT corruption).
