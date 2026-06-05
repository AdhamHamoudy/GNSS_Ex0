# Ex0 — Intro to GNSS Navigation
### RINEX 4.0 → Offline Path Solver (Position · Velocity · UTC Time)

---

## Table of Contents
1. [Assignment Overview](#assignment-overview)
2. [Algorithm Explained](#algorithm-explained)
3. [Repository Structure](#repository-structure)
4. [Installation](#installation)
5. [Run Example](#run-example)
6. [Sample Results](#sample-results)
7. [Output Format](#output-format)
8. [Design Decisions](#design-decisions)
9. [Known Limitations](#known-limitations)

---

## Assignment Overview

This project is the first exercise in a GNSS (Global Navigation Satellite System) navigation course.

The goal is to implement a complete offline positioning pipeline **based solely on a RINEX 4.0 observation file** recorded by an Android device. Given raw satellite measurements, the algorithm computes a **1 Hz path** containing:

- **3D Position** — latitude, longitude, altitude (WGS-84)
- **3D Velocity** — ground speed and directional ECEF components
- **UTC Time** — one row per second, correctly converted from GPS system time

The outputs are a **CSV file** (for analysis) and a **KML file** (for Google Earth visualization).

> The NMEA and TXT files provided alongside the RINEX file are used **only for verification**. The algorithm does not read them — every position is computed purely from raw satellite pseudoranges.

Three recordings are included in this repository, each fully processed with CSV and KML output files ready to view.

---

## Algorithm Explained

The core algorithm runs once per second (epoch). Here is a step-by-step breakdown:

### Step 1 — Load and Parse the RINEX File

Android devices record raw GNSS measurements in **RINEX 4.0** format. Since the `georinex` library supports up to RINEX 3.03, the version header is patched in memory before parsing. No modification is made to the original file on disk.

The parser extracts:
- Satellite IDs (e.g. `G01`, `G15` for GPS)
- Pseudorange observables (`C1C`, `C1W`, `C1X`, or `C5Q` — first available wins)
- Signal-to-Noise Ratio (`S1C` etc.)
- Epoch timestamps

### Step 2 — 1 Hz Epoch Grouping

Raw RINEX files often contain measurements at sub-second intervals. Every raw timestamp is rounded to the nearest whole second and grouped into **1 Hz buckets**. When multiple measurements fall in the same second, the first is kept. This enforces the assignment's 1 Hz output requirement.

### Step 3 — Measurement Filtering

For each epoch, satellites are discarded if:

| Filter | Threshold | Reason |
|---|---|---|
| Pseudorange range | 15 000 km – 30 000 km | Rejects clearly erroneous measurements |
| Signal-to-noise ratio | SNR > 20 dB-Hz | Removes weak / obstructed signals |
| Minimum satellites | ≥ 5 per epoch | 4 is the theoretical minimum but leaves no redundancy; 5+ allows the solver to detect and down-weight outliers |
| Constellation | GPS (`G`) only | Avoids estimating a 5th unknown (inter-system clock bias) for Galileo / GLONASS |

### Step 4 — Satellite Position & Clock Correction

For each surviving satellite, the **Laika** library fetches orbital ephemeris and clock data at the signal's transmit time. Two physics corrections are then applied:

**a) Satellite Clock Bias**

The satellite's on-board atomic clock is never perfect. Its bias (in seconds) is provided in the navigation message. The corrected pseudorange is:

```
PR_corrected = PR_measured + sat_clock_bias × c
```

Where `c = 299 792 458 m/s` (speed of light).

**b) Sagnac Effect (Earth Rotation Correction)**

The satellite position is computed in an inertial frame at **signal transmission** time. By the time the signal **arrives**, Earth has rotated by a small angle:

```
θ = Ω_E × flight_time
```

Where `Ω_E = 7.2921151467 × 10⁻⁵ rad/s`. A rotation matrix is applied to bring the satellite ECEF coordinates into the receiver's frame at reception time.

### Step 5 — Least-Squares Position Solver

The receiver's state vector has 4 unknowns: `[X, Y, Z, receiver_clock_bias]`.

For each satellite `i`, the observation equation is:

```
PR_i = √[(X_sat - X)² + (Y_sat - Y)² + (Z_sat - Z)²]  +  clock_bias  +  ε_i
```

This nonlinear system is solved with **iteratively reweighted least squares** using a **soft-L1 (Huber-like) loss function**. The soft-L1 loss automatically down-weights large residuals caused by multipath or non-line-of-sight signals, making the solver robust without manual outlier removal.

**Warm start:** After the first epoch, the previous solution is used as the initial guess. This dramatically speeds up convergence and keeps the solution stable across consecutive epochs.

**Acceptance criterion:** A solution is accepted only when the RMS of the final residuals is below **100 m**. This correctly rejects epochs where the solver converged to a wrong local minimum, without discarding valid solutions that `scipy` may mark `success = False` due to gradient tolerance.

### Step 6 — Sanity Checks

Two physical sanity checks reject outlier epochs and reset the warm-start tracker:

| Check | Threshold | Action on failure |
|---|---|---|
| Altitude | –500 m to 3 000 m | Discard epoch + reset warm start |
| Ground speed | < 60 m/s (216 km/h) | Discard epoch + reset warm start |

Resetting the warm start on failure prevents one bad epoch from corrupting all subsequent positions.

### Step 7 — Velocity with EMA Smoothing

Velocity is derived as the finite difference of consecutive ECEF positions:

```
V_raw = (ECEF_current − ECEF_previous) / Δt
```

Raw GNSS velocity inherits all position noise, producing unrealistic second-to-second jumps. A simple **Exponential Moving Average (EMA)** filter smooths the result:

```
V_smooth = α × V_raw + (1 − α) × V_smooth_previous       (α = 0.4)
```

This suppresses per-second noise while still responding to real acceleration within 2–3 seconds.

### Step 8 — Coordinate Conversion and Output

The ECEF solution `[X, Y, Z]` is converted to geodetic coordinates `(lat, lon, alt)` using the WGS-84 ellipsoid model via Laika's `ecef2geodetic`. UTC time is obtained by subtracting the GPS–UTC leap second offset (18 s) from the GPS timestamp. Results are written to CSV and KML simultaneously.

---

## Repository Structure

```
ex0-gnss-navigation/
│
├── main.py                                          # Main solver — run this
├── README.md                                        # This file
│
├── gnss_log_2026_03_21_17_14_34_offline_path.csv   # Output — recording 1
├── gnss_log_2026_03_21_17_14_34_offline_path.kml   # Output — recording 1
│
├── gnss_log_2026_03_21_17_17_57_offline_path.csv   # Output — recording 2
├── gnss_log_2026_03_21_17_17_57_offline_path.kml   # Output — recording 2
│
├── gnss_log_2026_03_22_08_44_21_offline_path.csv   # Output — recording 3 (Tel Aviv → Jerusalem)
└── gnss_log_2026_03_22_08_44_21_offline_path.kml   # Output — recording 3
```

> The three `.26o` RINEX input files are stored in the [shared course folder](https://drive.google.com) and are not committed to the repository due to their size.

---

## Installation

**Requirements:** Python 3.9 or newer.

```bash
# 1. Clone the repository
git clone https://github.com/AdhamHamoudy/ex0-gnss-navigation.git
cd ex0-gnss-navigation

# 2. (Recommended) Create a virtual environment
python -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate

# 3. Install dependencies
pip install georinex laika simplekml scipy numpy pandas tqdm
```

> **Note:** `laika` downloads satellite ephemeris data from NASA servers on first run.  
> An internet connection is required. Data is cached locally for subsequent runs.

---

## Run Example

### Auto-detect RINEX file in the current folder

Place your `.26o` file in the same folder as `main.py`, then run:

```bash
python main.py
```

```
[auto] Found RINEX file: gnss_log_2026_03_22_08_44_21.26o
Loading: gnss_log_2026_03_22_08_44_21.26o
  [compat] Rewrote version header 4.0 → 3.03

Raw epochs        : 2184
1 Hz epochs       : 1847
Pseudorange field : C1C
SNR field         : S1C
Constellations    : ('G',)
Min satellites    : 5
Max residual RMS  : 100.0 m
Max speed         : 60.0 m/s  (216 km/h)
CSV → gnss_log_2026_03_22_08_44_21_offline_path.csv
KML → gnss_log_2026_03_22_08_44_21_offline_path.kml

Processing epochs: 100%|████████████| 1847/1847 [03:21<00:00,  9.17it/s]

====================================================
  Done.  1700 / 1847 epochs solved.
  Skipped – not enough satellites   : 89
  Skipped – solver residual too high : 31
  Skipped – altitude out of range   : 12
  Skipped – speed > 60.0 m/s        : 15
  CSV → gnss_log_2026_03_22_08_44_21_offline_path.csv
  KML → gnss_log_2026_03_22_08_44_21_offline_path.kml
====================================================
```

### Specify file paths explicitly

```bash
python main.py path/to/gnss_log.26o --csv output/path.csv --kml output/path.kml
```

### View results in Google Earth

1. Open **Google Earth Pro** (free download from Google)
2. Go to **File → Open** and select any `*_offline_path.kml` file
3. The path appears as a red line with altitude; click any point to see UTC time, satellites used, and speed

---

## Sample Results

Three recordings are included, all processed and ready to open:

### Recording 1 — `gnss_log_2026_03_21_17_14_34`
| Field | Value |
|---|---|
| Date | March 21, 2026 — 17:14 UTC |
| Output | `gnss_log_2026_03_21_17_14_34_offline_path.csv / .kml` |

### Recording 2 — `gnss_log_2026_03_21_17_17_57`
| Field | Value |
|---|---|
| Date | March 21, 2026 — 17:17 UTC |
| Output | `gnss_log_2026_03_21_17_17_57_offline_path.csv / .kml` |

### Recording 3 — `gnss_log_2026_03_22_08_44_21` *(main example)*
| Field | Value |
|---|---|
| Date | March 22, 2026 — 08:44 UTC |
| Route | Tel Aviv → Jerusalem |
| Solved epochs | 1 700 / 1 847 |
| Altitude range | ~15 m → ~800 m (matches the highway climb) |
| Max speed | 27 m/s (97 km/h) |
| Avg satellites | 6.8 per epoch |
| Output | `gnss_log_2026_03_22_08_44_21_offline_path.csv / .kml` |

---

## Output Format

### CSV — `<stem>_offline_path.csv`

| Column | Unit | Description |
|---|---|---|
| `UTC Time` | `YYYY-MM-DD HH:MM:SS` | UTC time (GPS time minus 18 leap seconds) |
| `Latitude` | degrees °N | WGS-84 geodetic latitude |
| `Longitude` | degrees °E | WGS-84 geodetic longitude |
| `Altitude(m)` | metres | WGS-84 ellipsoidal height |
| `Speed(m/s)` | m/s | EMA-smoothed ground speed magnitude |
| `Vx(m/s)` | m/s | EMA-smoothed ECEF X velocity component |
| `Vy(m/s)` | m/s | EMA-smoothed ECEF Y velocity component |
| `Vz(m/s)` | m/s | EMA-smoothed ECEF Z velocity component |
| `Satellites Used` | count | GPS satellites used in this epoch |

### KML — `<stem>_offline_path.kml`

- **Points** — one per epoch, labelled with UTC time; click to see satellites used and speed
- **LineString** — the full path drawn in red at absolute altitude (visible in Google Earth 3D view)

---

## Design Decisions

**GPS only, not GPS + Galileo**  
Mixing constellations introduces an inter-system clock bias — a 5th unknown — which requires at least 5 satellites just to make the system solvable and worsens geometry when only a handful of measurements are available. GPS alone provides enough satellites and keeps the solver simple and correct for a first implementation.

**Soft-L1 loss, not standard least squares**  
Urban environments produce multipath reflections that cause occasional pseudorange errors of hundreds of metres. Standard least squares treats all measurements equally and is pulled badly by these outliers. Soft-L1 automatically assigns lower weight to large residuals without needing to identify and manually remove them.

**Residual RMS gate instead of `result.success`**  
`scipy.least_squares` sets `success = False` when the gradient norm falls below a tolerance — even for a perfectly converged solution. Checking the RMS of the final residuals (< 100 m) correctly separates good solutions from genuinely bad ones.

**EMA velocity smoothing**  
Finite-difference velocity inherits all position noise and produces unrealistic second-to-second speed jumps. A single-parameter EMA filter (α = 0.4) is transparent, easy to tune, and adds no latency beyond ~3 seconds — appropriate for a 1 Hz offline solver.

---

## Known Limitations

- **Altitude noise ±20–40 m** — inherent in single-frequency pseudorange-only positioning. The vertical component always has 2–3× worse accuracy than horizontal due to satellite geometry (all satellites are above, never below the receiver). This matches the phone's own raw GNSS accuracy before its internal Kalman filter runs.

- **No Kalman filter** — a Kalman filter would smooth both position and velocity across epochs and handle data gaps gracefully. This implementation uses warm-start least squares, which is simpler and sufficient for the assignment requirements.

- **Leap seconds hardcoded** — the GPS–UTC offset is currently 18 seconds (unchanged since 2017). If a new leap second is announced, update `GPS_UTC_LEAP_SECONDS` at the top of `main.py`.

- **Internet required on first run** — Laika downloads satellite ephemeris from NASA CDDIS servers and caches it locally. Subsequent runs on data from the same time period work fully offline.
