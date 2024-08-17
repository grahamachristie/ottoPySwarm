# Incoming serial connection
gnss_port = '/dev/ttyUSB0'
gnss_baud = 115200

# Input seafloor model file names
vesselB_model_file = 'output/vB_empty.nc'

# Output seafloor model file names
vesselB_output_file = 'output/vB_hhparking_filled.nc'

from datetime import datetime as dt
import serial
import pynmea2
import xarray as xr
import csv
import random
import geopandas
from shapely.geometry import Point, box
import pyproj
import geopandas as gpd
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
from shapely.ops import transform
import pandas as pd
from shapely.geometry import Polygon
import numpy as np
from shapely.prepared import prep
import pandas as pd
pd.options.mode.chained_assignment = None  # default='warn'

import math

# Open the data models for each vessel
vB = xr.open_dataset(vesselB_model_file)

# Pull out attributes for survey as a whole and for each vessel for easy use
home_point = vB.attrs['home_point']
away_point = vB.attrs['away_point']
allow_sigmaVert = vB.attrs['allowable_sigmaVertical']
input_geodetics = 'EPSG:4326'                                
project_geodetics = 'EPSG:32619' 

vB_sigmaVert = vB.attrs['vesselB_sigmaVertical']
vB_sigmaHor = vB.attrs['vesselB_sigmaHorizontal']
vB_min_safe_depth = vB.attrs['vesselB_min_safe_depth']

# For a new value new_value, compute the new count, new mean, the new M2.
# mean accumulates the mean of the entire dataset
# M2 aggregates the squared distance from the mean
# count aggregates the number of samples seen so far
def update(existing_aggregate, new_value):
    (count, mean, M2) = existing_aggregate
    count += 1
    delta = new_value - mean
    mean += delta / count
    
    
    delta2 = new_value - mean
    M2 += delta * delta2
    return (count, mean, M2)

# Retrieve the mean, variance and sample variance from an aggregate
def finalize(existing_aggregate):
    (count, mean, M2) = existing_aggregate
    if count < 2:
        return float("nan")
    else:
        (mean, variance, sample_variance) = (mean, M2 / count, M2 / (count - 1))
        return (mean, variance, sample_variance)
    
wgs84 = pyproj.CRS(input_geodetics)
utm = pyproj.CRS(project_geodetics)

projection = pyproj.Transformer.from_crs(wgs84, utm, always_xy=True).transform

def read_incoming_data(vessel_output_file, vDS, vessel, home, away, projection, gnss_port, gnss_baud):
    ser = serial.Serial(port=gnss_port, baudrate=gnss_baud)
    
    depth = 2.5
    
    count = 0
    
    vB_update_number = 0
    vB_update_number_list = []
    vB_updates_sending_list = []
    
    vA_update_number_list = []
    vA_this_update_list = []
    vA_updates_received_list = []
    
    all_updates = []
    
    prev_coords = (None, None)
    prev_cell = [0, 0, 0, 0, 0, 0, 0]
    vB_recent = []
    
    while count < 2500:
        try:
            line_raw = ser.readline()
            line = bytes.decode(line_raw)
            
            try:
                msg = pynmea2.parse(line)
            except:
                pass
                
            if msg.sentence_type == 'RMC':
                lat = msg.latitude
                long = msg.longitude
                
                coords = transform(projection, Point([long, lat]))
                north = coords.y
                east = coords.x
                
                north = round(north - home[1])
                east = round(east - home[0])
                
                depth = random.uniform(depth + 0.11, depth - 0.1)
                
                
                if north != prev_coords[0] or east != prev_coords[1]:
                    vB_recent.append(prev_cell)
                    
                    
                    
                if np.isnan(vDS['soundings'].loc[dict(north=north, east=east, vessel=vessel)]):
                    vDS['depth'].loc[dict(north=north, east=east, vessel=vessel)] = depth
                    vDS['soundings'].loc[dict(north=north, east=east, vessel=vessel)] = 1
                    vDS['M2'].loc[dict(north=north, east=east, vessel=vessel)] = 0
                    vDS['stdev'].loc[dict(north=north, east=east, vessel=vessel)] = 0
                    
                    if depth > vB_min_safe_depth:
                        safe = True
                        vDS['safe'].loc[dict(north=north, east=east, vessel=vessel)] = safe
                    else:
                        safe = False
                        vDS.loc[dict(north=north, east=east, vessel=vessel)] = safe
                        
                    prev_cell = [north, east, depth, 1, 0, 0, safe]
                    prev_coords = (north, east)
                    
                    
                else:
                    prev_soundings = vDS['soundings'].loc[dict(north=north, east=east, vessel=vessel)]
                    prev_depth = vDS['depth'].loc[dict(north=north, east=east, vessel=vessel)]
                    prev_M2 = vDS['M2'].loc[dict(north=north, east=east, vessel=vessel)]

                    updates = update((prev_soundings, prev_depth, prev_M2), depth)
                    finals = finalize(updates)

                    vDS['depth'].loc[dict(north=north, east=east, vessel=vessel)] = updates[1]
                    vDS['soundings'].loc[dict(north=north, east=east, vessel=vessel)] = updates[0]
                    vDS['M2'].loc[dict(north=north, east=east, vessel=vessel)] = updates[2]
                    vDS['stdev'].loc[dict(north=north, east=east, vessel=vessel)] = math.sqrt(finals[1])
                    
                    if depth > vB_min_safe_depth:
                        safe = True
                        vDS['safe'].loc[dict(north=north, east=east, vessel=vessel)] = safe
                    else:
                        safe = False
                        vDS.loc[dict(north=north, east=east, vessel=vessel)] = safe
                    
                    prev_cell = [north, east, float(updates[1]), float(updates[0]), float(updates[2]), math.sqrt(float(finals[1])), safe]
                    prev_coords = (north, east)
                    
                
                count += 1
                print(count)
                
                interval = count / 75
                if interval.is_integer():
                    vB_recent.append(prev_cell)        
                    all_updates.append(vB_recent)
                    vB.to_netcdf(path=vesselB_output_file)
                    print('NetCDF file Saved')
                    
                    vB_update_number_list.append(vB_update_number)                    
                    vB_updates_sending_list.append(vB_update_number)
                    
                    with open('updates/vB_recent.csv', 'w', newline='') as file:
                        csv_writer = csv.writer(file)       
                        csv_writer.writerow(vB_updates_sending_list)
                        csv_writer.writerow(list(set(vA_updates_received_list)))
                        csv_writer.writerow(vB_update_number_list)                        
                        for update_number in vB_updates_sending_list:                   
                            csv_writer.writerows(all_updates[update_number])
                            print(update_number)
                        print('Update Written')                        
                        
                    with open('updates/vB_recent' + str(vB_update_number) + '.csv', 'w', newline='') as file:
                        csv_writer = csv.writer(file)       
                        csv_writer.writerow(vB_updates_sending_list)
                        csv_writer.writerow(list(set(vA_updates_received_list)))
                        csv_writer.writerow(vB_update_number_list)                        
                        for update_number in vB_updates_sending_list:
                            print('8')
                            csv_writer.writerows(all_updates[update_number])
                            print('9')
                            print(update_number)
                        print('Update Written')
                    
                        import subprocess

                    #arg = 'updates/vB_recent' + str(vB_update_number) + '.csv'
                    arg = 'updates/vB_recent.csv'
                        
                    val = subprocess.check_call("./testbash2.sh '%s'" % arg, shell=True)
                    
                    vB_recent = [] 
                    
                    vB_update_number += 1

                    with open('transfered/vA_recent.csv', 'r', newline='') as csvfile:
                        csv_reader = csv.reader(csvfile)
                        
                        vA_this_update_list = [int(item) for item in next(csv_reader)]
                        print(vA_this_update_list)
                        
                        for item in vA_this_update_list:
                            vA_updates_received_list.append(item)
                        
                        vA_update_number_list = [int(item) for item in next(csv_reader)]
                        print(vA_update_number_list)
                        
                        
                        vB_on_vA_update_number_list = [int(item) for item in next(csv_reader)]
                        print(vB_on_vA_update_number_list)
                        
                        
                        
                        for row in csv_reader:
                            vDS['depth'].loc[dict(north=int(row[0]), east=int(row[1]), vessel='A')] = float(row[2])
                            vDS['soundings'].loc[dict(north=int(row[0]), east=int(row[1]), vessel='A')] = int(float(row[3]))
                            vDS['M2'].loc[dict(north=int(row[0]), east=int(row[1]), vessel='A')] = float(row[4])
                            vDS['stdev'].loc[dict(north=int(row[0]), east=int(row[1]), vessel='A')] = float(row[5])
                        print('Update read')
                    
                    vB_update_number_set = set(vB_update_number_list)
                    vB_on_vA_update_number_set = set(vB_on_vA_update_number_list)
                    
                    vB_updates_sending_list = list(vB_update_number_set.difference(vB_on_vA_update_number_set))
                    
                    print(vB_updates_sending_list)
                    
                    
                        
        except:
            pass
    
    ser.close()
    print(all_updates)
    return vDS

vB = read_incoming_data(vesselB_output_file, vB, 'B', home_point, away_point, projection, gnss_port, gnss_baud)
