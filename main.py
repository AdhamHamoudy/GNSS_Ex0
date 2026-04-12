import georinex as gr
import io
import numpy as np
import pandas as pd
import datetime
import csv
import simplekml
from tqdm import tqdm
from scipy.optimize import least_squares
from laika import AstroDog
from laika.gps_time import GPSTime
from laika.lib.coordinates import ecef2geodetic

def load_rinex_data(file_path):
    print(f"Loading RINEX file: {file_path}...")
    with open(file_path, 'r') as file:
        lines = file.readlines()
        
    first_line = lines[0]
    if '4.01' in first_line or '4.0' in first_line:
        lines[0] = first_line.replace('4.01', '3.03').replace('4.00', '3.03').replace('4.0', '3.03')
        
    modified_content = "".join(lines)
    return gr.load(io.StringIO(modified_content))

def gnss_residuals(guess, sat_positions, pseudoranges):
    vectors = sat_positions - guess[:3]
    ranges = np.linalg.norm(vectors, axis=1)
    return pseudoranges - (ranges + guess[3])

if __name__ == "__main__":
    rinex_file = 'gnss_log_2026_03_22_08_44_21.26o' 
    
    try:
        obs_data = load_rinex_data(rinex_file)
        
        # 1. OPTIMIZATION: Extract to pure Numpy for blazing fast processing
        sv_list = obs_data.sv.values
        time_list = obs_data.time.values
        c1c_data = obs_data['C1C'].values
        s1c_data = obs_data['S1C'].values
        
        # We will track both American GPS and European Galileo!
        valid_sat_indices = [i for i, sv in enumerate(sv_list) if str(sv).startswith('G') or str(sv).startswith('E')]
        
        dog = AstroDog()
        C = 299792458.0 
        OMEGA_E = 7.2921151467e-5 
        
        # Output setup
        kml = simplekml.Kml()
        csv_file = open('offline_path.csv', 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['UTC Time', 'Latitude', 'Longitude', 'Altitude(m)', 'Velocity(m/s)', 'Satellites Used'])
        
        prev_ecef = None
        prev_time = None
        prev_clock_bias = 0.0
        
        print(f"\nProcessing {len(time_list)} epochs. Buckle up...")
        
        # 2. THE BIG LOOP: Process every single second in the file
        for t_idx, epoch_np in enumerate(tqdm(time_list)):
            
            epoch_pd = pd.to_datetime(epoch_np).to_pydatetime()
            utc_dt = epoch_pd - datetime.timedelta(seconds=18) # Leap Second Fix
            gps_time = GPSTime.from_datetime(utc_dt)
            
            sat_positions = []
            pseudoranges = []
            
            for s_idx in valid_sat_indices:
                sat = sv_list[s_idx]
                pr = c1c_data[t_idx, s_idx]
                snr = s1c_data[t_idx, s_idx]
                
                if not np.isnan(pr) and 15000000 < pr < 30000000 and not np.isnan(snr) and snr > 20.0:
                    flight_time = pr / C
                    tx_time = gps_time - flight_time
                    
                    sat_info = dog.get_sat_info(sat, tx_time)
                    if sat_info is not None:
                        # Physics Fixes
                        pr_corrected = pr + (sat_info[2] * C)
                        
                        # Sagnac Effect (Earth rotating while signal travels)
                        theta = OMEGA_E * flight_time
                        rot_matrix = np.array([
                            [np.cos(theta),  np.sin(theta), 0],
                            [-np.sin(theta), np.cos(theta), 0],
                            [0,              0,             1]
                        ])
                        corrected_pos = rot_matrix.dot(sat_info[0])
                        
                        sat_positions.append(corrected_pos)
                        pseudoranges.append(pr_corrected)
            
            sat_positions = np.array(sat_positions)
            pseudoranges = np.array(pseudoranges)
            
           # If we found at least 4 strong satellites, solve!
            if len(sat_positions) >= 4:
                
                # If this is the very first second, start with our Israel anchor
                if prev_ecef is None:
                    guess_pos = np.array([4409000.0, 3108000.0, 3383000.0])
                    initial_clock_guess = np.mean(pseudoranges - np.linalg.norm(sat_positions - guess_pos, axis=1))
                    guess_array = np.array([guess_pos[0], guess_pos[1], guess_pos[2], initial_clock_guess])
                else:
                    # SMART TRACKING: Use the EXACT position from 1 second ago as our new guess!
                    # We assume the clock bias roughly carried over too.
                    guess_array = np.array([prev_ecef[0], prev_ecef[1], prev_ecef[2], prev_clock_bias])

                result = least_squares(
                    gnss_residuals, 
                    guess_array, 
                    args=(sat_positions, pseudoranges), 
                    loss='soft_l1'
                )
                
                # Save the clock bias so we can carry it over to the next second
                prev_clock_bias = result.x[3]
                
                curr_ecef = result.x[:3]
                lat, lon, alt = ecef2geodetic(curr_ecef)
                
                # --- THE SANITY CHECK ---
                # If the math puts us >5km underground or >10km in the air, it's a glitch!
                if alt < -5000 or alt > 10000:
                    prev_ecef = None # Reset the tracking loop!
                    continue # Skip this broken second and move to the next one
                
                # Calculate Velocity
                speed = 0.0

                if prev_ecef is not None:
                    dt_seconds = (epoch_pd - prev_time).total_seconds()
                    if dt_seconds > 0:
                        speed = np.linalg.norm(curr_ecef - prev_ecef) / dt_seconds
                
                # Save to CSV
                csv_writer.writerow([epoch_pd.strftime('%Y-%m-%d %H:%M:%S'), lat, lon, alt, speed, len(sat_positions)])
                
                # Save to KML
                kml.newpoint(name="", coords=[(lon, lat, alt)])
                
                prev_ecef = curr_ecef
                prev_time = epoch_pd

        csv_file.close()
        kml.save("offline_path.kml")
        
        print("\n DONE! Data saved to 'offline_path.csv' and 'offline_path.kml' ")
        
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"An error occurred: {e}")