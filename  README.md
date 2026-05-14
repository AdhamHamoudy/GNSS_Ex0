# Ex0 – Intro to GNSS Navigation

## Overview

This project implements an offline GNSS navigation solution based on an Android RINEX observation file.  
The program reads a RINEX 4.x observation file, processes the raw GNSS measurements, estimates the receiver position over time, computes velocity, converts the solution to geodetic coordinates, and exports the final trajectory as both CSV and KML files. 

The goal of the project is to produce a 1 Hz path containing:
- UTC time
- 3D position
- velocity
- number of satellites used

The output is saved in:
- a `.csv` file for numerical analysis
- a `.kml` file for visualization in Google Earth or similar tools 

---

## Main Features

- Loads Android RINEX observation files (`.26o`, `.obs`, `.rnx`)
- Groups raw epochs into a 1 Hz timeline
- Uses pseudorange observations to estimate receiver ECEF position
- Applies satellite clock correction and Earth rotation (Sagnac) correction
- Solves the navigation equations using robust least squares
- Converts ECEF coordinates to latitude, longitude, and altitude
- Computes velocity from epoch-to-epoch motion
- Applies sanity filters on pseudorange, altitude, and speed
- Exports results to CSV and KML :contentReference[oaicite:3]{index=3}

---

## Algorithm Summary

For each 1 Hz epoch, the program performs the following steps:

1. Load pseudorange and SNR measurements from the RINEX file.
2. Keep only valid GPS satellites that satisfy the measurement quality thresholds.
3. Correct the pseudorange using the satellite clock bias.
4. Apply Earth rotation correction to the satellite ECEF coordinates.
5. Solve the receiver state  
   \[
   [X, Y, Z, \Delta t]
   \]
   using robust nonlinear least squares.
6. Convert the ECEF solution to geodetic coordinates:
   - latitude
   - longitude
   - altitude
7. Estimate receiver velocity from the position difference between consecutive epochs.
8. Smooth the velocity using an exponential moving average.
9. Save the valid solutions to CSV and KML.

---

## Project Requirements

This project uses Python and the following libraries:

- `georinex`
- `laika`
- `simplekml`
- `scipy`
- `numpy`
- `pandas`
- `tqdm` :contentReference[oaicite:5]{index=5}

Install them with:

```bash
pip install georinex laika simplekml scipy numpy pandas tqdm