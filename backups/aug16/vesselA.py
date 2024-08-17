# Incoming serial connection
gnss_port = '/dev/ttyUSB0'
gnss_baud = 115200

# Input seafloor model file names
vesselA_model_file = 'output/vA_empty.nc'

# Output seafloor model file names
vesselA_output_file = 'output/vA_headhall_filled.nc'

from datetime import datetime as dt
import serial
import pynmea2
import xarray as xr
import time
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
vA = xr.open_dataset(vesselA_model_file)

# Pull out attributes for survey as a whole and for each vessel for easy use
home_point = vA.attrs['home_point']
away_point = vA.attrs['away_point']
allow_sigmaVert = vA.attrs['allowable_sigmaVertical']
input_geodetics = 'EPSG:4326'                                
project_geodetics = 'EPSG:32619' 

vA_sigmaVert = vA.attrs['vesselA_sigmaVertical']
vA_sigmaHor = vA.attrs['vesselA_sigmaHorizontal']
vA_min_safe_depth = vA.attrs['vesselA_min_safe_depth']

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
    
    vA_update_number = 0
    vA_update_number_list = []
    vA_updates_sending_list = []
    
    vB_update_number_list = []
    vB_this_update_list = []
    vB_updates_received_list = []
    
    
    all_updates = []
    
    prev_coords = (None, None)
    prev_cell = [0, 0, 0, 0, 0, 0, 0]
    vA_recent = []
    
    while count < 2555:
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
                    vA_recent.append(prev_cell)
                    
                    
                    
                if np.isnan(vDS['soundings'].loc[dict(north=north, east=east, vessel=vessel)]):
                    vDS['depth'].loc[dict(north=north, east=east, vessel=vessel)] = depth
                    vDS['soundings'].loc[dict(north=north, east=east, vessel=vessel)] = 1
                    vDS['M2'].loc[dict(north=north, east=east, vessel=vessel)] = 0
                    vDS['stdev'].loc[dict(north=north, east=east, vessel=vessel)] = 0
                    
                    if depth > vA_min_safe_depth:
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
                    
                    if depth > vA_min_safe_depth:
                        safe = True
                        vDS['safe'].loc[dict(north=north, east=east, vessel=vessel)] = safe
                    else:
                        safe = False
                        vDS.loc[dict(north=north, east=east, vessel=vessel)] = safe
                    
                    prev_cell = [north, east, float(updates[1]), float(updates[0]), float(updates[2]), math.sqrt(float(finals[1])), safe]
                    prev_coords = (north, east)
                    
                
                count += 1
                print(count)
                
                interval = count / 50
                if interval.is_integer():
                    vA_recent.append(prev_cell)        
                    all_updates.append(vA_recent)
                    vA.to_netcdf(path=vesselA_output_file)
                    print('NetCDF file Saved')
                    
                    vA_update_number_list.append(vA_update_number)                    
                    vA_updates_sending_list.append(vA_update_number)
                                        
                    with open('updates/vA_recent.csv', 'w', newline='') as file:
                        csv_writer = csv.writer(file)                        
                        csv_writer.writerow(vA_updates_sending_list)
                        csv_writer.writerow(vA_update_number_list)
                        csv_writer.writerow(list(set(vB_updates_received_list)))
                        for update_number in vA_updates_sending_list:
                            csv_writer.writerows(all_updates[update_number])
                            print(update_number)
                        print('Update Written')
                        
                    with open('updates/vA_recent' + str(vA_update_number) + '.csv', 'w', newline='') as file:
                        csv_writer = csv.writer(file)                        
                        csv_writer.writerow(vA_updates_sending_list)
                        csv_writer.writerow(vA_update_number_list)
                        csv_writer.writerow(list(set(vB_updates_received_list)))
                        for update_number in vA_updates_sending_list:
                            csv_writer.writerows(all_updates[update_number])
                            print(update_number)
                        print('Update Written')
                        
                    import subprocess

                    #arg = 'updates/vA_recent' + str(vA_update_number) + '.csv'
                    arg = 'updates/vA_recent.csv'
                        
                    val = subprocess.check_call("./testbash2.sh '%s'" % arg, shell=True)
                    
                    vA_recent = [] 
                    
                    vA_update_number += 1

                    with open('transfered/vB_recent.csv', 'r', newline='') as csvfile:
                        csv_reader = csv.reader(csvfile)
                        
                        vB_this_update_list = [int(item) for item in next(csv_reader)]
                        print(vB_this_update_list)
                        
                        for item in vB_this_update_list:
                            vB_updates_received_list.append(item)
                        
                        vA_on_vB_update_number_list = [int(item) for item in next(csv_reader)]
                        print(vA_on_vB_update_number_list)
                        
                        
                        vB_update_number_list = [int(item) for item in next(csv_reader)]
                        print(vB_update_number_list)
                        
                        
                        for row in csv_reader:
                            vDS['depth'].loc[dict(north=int(row[0]), east=int(row[1]), vessel='B')] = float(row[2])
                            vDS['soundings'].loc[dict(north=int(row[0]), east=int(row[1]), vessel='B')] = int(float(row[3]))
                            vDS['M2'].loc[dict(north=int(row[0]), east=int(row[1]), vessel='B')] = float(row[4])
                            vDS['stdev'].loc[dict(north=int(row[0]), east=int(row[1]), vessel='B')] = float(row[5])
                        print('Update read')
                    
                    vA_update_number_set = set(vA_update_number_list)
                    vA_on_vB_update_number_set = set(vA_on_vB_update_number_list)
                    
                    vA_updates_sending_list = list(vA_update_number_set.difference(vA_on_vB_update_number_set))
                    
                    print(vA_updates_sending_list)
                    
                    
                        
        except:
            pass
    
    ser.close()
    print(all_updates)
    return vDS

vA = read_incoming_data(vesselA_output_file, vA, 'A', home_point, away_point, projection, gnss_port, gnss_baud)
