'''
####################################
OttoSwarmPylot Controller - Vessel A
####################################

Graham Christie | Aug 17, 2024

'''


####################################
# Parameters
####################################

# Vessel letter
vessel = 'A'
vessel_other = 'B'

# Control motors
motor_control = False

####################################
# Import Libraries
####################################
import csv
import math
import time
import pyproj
import serial
import random
import pigpio
import pynmea2
import subprocess

if motor_control != False:
    import read_sbus_from_GPIO

import numpy as np
import xarray as xr

from shapely.ops import transform
from shapely.geometry import Point



####################################
# Functions
####################################
def set_geodetics(vDS):
    # Function to set the project geodetics for input and output
    print('Setting up geodetics...')
    wgs84 = pyproj.CRS('EPSG:4326')                                                     # Add to template and change to pull out of metadata
    utm = pyproj.CRS('EPSG:32619')                                                      # Add to template and change to pull out of metadata
    
    projection = pyproj.Transformer.from_crs(wgs84, utm, always_xy=True).transform
    back_projection = pyproj.Transformer.from_crs(utm, wgs84, always_xy=True).transform
    
    print('Geodetics initialized!')
    
    return projection, back_projection

def get_depth():
    # Function to get depth, but for now just make a random depth, change later         # Change later to actually get depth from sonar
    depth = 2.5
    depth = random.uniform(depth + 0.11, depth - 0.1)
    
    return depth

def parse_nmea(line_raw):
    line = bytes.decode(line_raw)
    print(line)
    try:
        msg = pynmea2.parse(line)
        
        if msg.sentence_type == 'RMC':
            lat = msg.latitude
            long = msg.longitude
            
        return lat, long
    
    except:
        pass
    
def transform_to_local(vDS, lat, long, projection):
    # Transform lat/long into UTM
    coords = transform(projection, Point([long, lat]))
    
    # Grab the northing and eastings
    north = coords.y
    east = coords.x
    
    # Convert to the local coordinates relative to the home point
    north_l = round(north - vDS.attrs['home_point'][1])
    east_l = round(east - vDS.attrs['home_point'][0])
    
    #print('Observation local coordinates: ', north_l, east_l)
    
    return north_l, east_l

def update(existing_aggregate, new_value):
    # Function to update the count in a cell
    (count, mean, M2) = existing_aggregate
    count += 1
    delta = new_value - mean
    mean += delta / count
    delta2 = new_value - mean
    M2 += delta * delta2
    return (count, mean, M2)

def finalize(existing_aggregate):
    # Function to retrieve the mean, variance and sample variance from an aggregate
    (count, mean, M2) = existing_aggregate
    if count < 2:
        return float("nan")
    else:
        (mean, variance, sample_variance) = (mean, M2 / count, M2 / (count - 1))
        return (mean, variance, sample_variance)
    
def map_value(min_input,max_input,min_output,max_output,invert_mapping,input):
    #for a given input in a range, output to a normalized value in the output range

    input_delta = max_input-min_input
    output_delta = max_output-min_output
    input_percent = (input-min_input)/input_delta
    new_val = min_output+output_delta*input_percent
    if invert_mapping:
        new_val = max_output+min_output-new_val
    new_val = int(new_val)
    
    return new_val

def read_incoming_data(vDS, vessel, projection, reader):
    # Main function that does a bunch of things                                         # Fix the documentation when done
    
    if reader != False:
        TRANSMITTER_MIN_VAL = 17                                                            # Put all these in a config file of some sort for the vessel
        TRANSMITTER_MAX_VAL = 1811
    
        MOTOR_MIN_VAL = 1000
        MOTOR_MAX_VAL = 2000
        MOTOR_DEFAULT = 1500
    
        STEER_CHANNEL = 1
        THROTTLE_CHANNEL = 2
        MODE_CHANNEL = 8
        ARM_CHANNEL = 7
    
        MOTOR_1_PIN = 26
        MOTOR_2_PIN = 13
        
        reader.pi.set_mode(MOTOR_1_PIN, pigpio.OUTPUT)
        reader.pi.set_mode(MOTOR_2_PIN, pigpio.OUTPUT)
        
        print('Waking up the RC receiver...')
        time.sleep(2)
        reader.display_latest_packet()
        print('RC receiver awake!')
        time.sleep(5)
    
    # Open serial port for GNSS
    print('Opeining GNSS serial port...')
    ser = serial.Serial(port='COM10', baudrate=115200)                           # Add to template and change to pull out of metadata
    print('GNSS serial port opened!')
      
    vX_update_number = 0
    vX_update_number_list = []
    vX_updates_sending_list = []
    
    vO_update_number_list = []
    vO_this_update_list = []
    vO_updates_received_list = []
    
    vX_recent = []
    all_updates = []
    
    prev_coords = (None, None)
    prev_cell = [0, 0, 0, 0, 0, 0, 0]
    
    observation = 0
    
    armed = False
    
    while True:
        try:
            
            if reader != False:
                while True:
                    
                    if armed:
                        break
                        
                    print('Time to arm the motors.')
                    time.sleep(1)
                    
                    if reader.is_connected():
                        latest_channel_data = reader.translate_latest_packet()
                        
                        print(latest_channel_data)
                        if latest_channel_data[ARM_CHANNEL-1] > 1500:
                            print('Arming motors...')
                            time.sleep(1)
                            
                            reader.pi.set_servo_pulsewidth(MOTOR_1_PIN,0)
                            time.sleep(1)
                            reader.pi.set_servo_pulsewidth(MOTOR_1_PIN,2000)
                            time.sleep(1)
                            reader.pi.set_servo_pulsewidth(MOTOR_1_PIN,1000)
                            time.sleep(1)
                            reader.pi.set_servo_pulsewidth(MOTOR_1_PIN,1500)
                            time.sleep(1)
                            print('Motor one armed...')
                            print('Arming motor two...')
                            reader.pi.set_servo_pulsewidth(MOTOR_2_PIN,0)
                            time.sleep(1)
                            reader.pi.set_servo_pulsewidth(MOTOR_2_PIN,2000)
                            time.sleep(1)
                            reader.pi.set_servo_pulsewidth(MOTOR_2_PIN,1000)
                            time.sleep(1)
                            reader.pi.set_servo_pulsewidth(MOTOR_2_PIN,1500)
                            time.sleep(1)
                            
                            armed = True
                            print('All motors armed!')
                            break
                        else:
                            pass
                    else:
                        pass
                
                if reader.is_connected():
                    latest_channel_data = reader.translate_latest_packet()
                    if latest_channel_data[ARM_CHANNEL-1] < 1500:
                        reader.pi.set_servo_pulsewidth(MOTOR_1_PIN, 1500)
                        reader.pi.set_servo_pulsewidth(MOTOR_2_PIN, 1500)
                    else:
                        if latest_channel_data[MODE_CHANNEL-1] > 500:
                            cur_throttle_val = latest_channel_data[THROTTLE_CHANNEL-1]                
                            throttle_pwm = map_value(TRANSMITTER_MIN_VAL,TRANSMITTER_MAX_VAL,MOTOR_MIN_VAL,MOTOR_MAX_VAL, False , cur_throttle_val)
                    
                            cur_steer_val = latest_channel_data[STEER_CHANNEL-1]                
                            steer_pwm = map_value(TRANSMITTER_MIN_VAL,TRANSMITTER_MAX_VAL,MOTOR_MIN_VAL,MOTOR_MAX_VAL, False , cur_steer_val)
                    
                            motor_1_pwm = throttle_pwm + (steer_pwm - MOTOR_DEFAULT)
                            motor_2_pwm = throttle_pwm - (steer_pwm - MOTOR_DEFAULT)
                        
                            reader.pi.set_servo_pulsewidth(MOTOR_1_PIN, motor_1_pwm)
                            reader.pi.set_servo_pulsewidth(MOTOR_2_PIN, motor_2_pwm)
            
            
            # Extract lat and long from the NMEA message
            line_raw = ser.readline()   
            lat, long = parse_nmea(line_raw)
            
            # Transform lat long into local north and east
            north, east = transform_to_local(vDS, lat, long, projection)
            
            # Get the depth at the point
            depth = get_depth()
            
            # Check if the current position is in the same grid cell as before
            # If it isnt, append the previous cell to the recent list for transfer
            if north != prev_coords[0] or east != prev_coords[1]:
                vX_recent.append(prev_cell)
            
            # If there are no previous soundings in this cell, add the current observation
            if np.isnan(vDS['soundings'].loc[dict(north=north, east=east, vessel=vessel)]):
                vDS['depth'].loc[dict(north=north, east=east, vessel=vessel)] = depth
                vDS['soundings'].loc[dict(north=north, east=east, vessel=vessel)] = 1
                vDS['M2'].loc[dict(north=north, east=east, vessel=vessel)] = 0
                vDS['stdev'].loc[dict(north=north, east=east, vessel=vessel)] = 0
                
                # If the depth is deeper than the min safe depth for the vessel
                # mark as safe and if not, mark as unsafe.
                if depth > vDS.attrs['vessel' + vessel + '_min_safe_depth']:
                    safe = True
                    vDS['safe'].loc[dict(north=north, east=east, vessel=vessel)] = safe
                else:
                    safe = False
                    vDS.loc[dict(north=north, east=east, vessel=vessel)] = safe                    # Possibly add in here to stop the vessel and reverse in the future if unsafe
                
                # Note the observation into the previous cell fields for later use
                prev_cell = [north, east, depth, 1, 0, 0, safe]
                prev_coords = (north, east)
            
            # If there is a previous sounding in that cell
            else:
                # Grab the details of the previous sounding
                prev_soundings = vDS['soundings'].loc[dict(north=north, east=east, vessel=vessel)]
                prev_depth = vDS['depth'].loc[dict(north=north, east=east, vessel=vessel)]
                prev_M2 = vDS['M2'].loc[dict(north=north, east=east, vessel=vessel)]

                # Run the update with the new depth  measurement
                updates = update((prev_soundings, prev_depth, prev_M2), depth)
                finals = finalize(updates)
                
                # Record the new information into the cell
                vDS['depth'].loc[dict(north=north, east=east, vessel=vessel)] = updates[1]
                vDS['soundings'].loc[dict(north=north, east=east, vessel=vessel)] = updates[0]
                vDS['M2'].loc[dict(north=north, east=east, vessel=vessel)] = updates[2]
                vDS['stdev'].loc[dict(north=north, east=east, vessel=vessel)] = math.sqrt(finals[1])
                
                # Check on the safe status again with the new depth estimate
                if updates[1] > vDS.attrs['vessel' + vessel + '_min_safe_depth']:
                    safe = True
                    vDS['safe'].loc[dict(north=north, east=east, vessel=vessel)] = safe
                else:
                    safe = False
                    vDS.loc[dict(north=north, east=east, vessel=vessel)] = safe
                
                # Note the new observation into the previous cell fields for later use
                prev_cell = [north, east, float(updates[1]), float(updates[0]), float(updates[2]), math.sqrt(float(finals[1])), safe]
                prev_coords = (north, east)
            
            # Increment obseravation number
            observation += 1
            print('Observation number: ', observation)
            
            # Check if the observation is at the threshold for recording
            interval = observation / 50                                                          # Make this user settable in the metadata of the template
            
            if interval.is_integer():
                
                # If it is, append the most recent observation to the current update list
                vX_recent.append(prev_cell)
                # Append the recent observation list to the all updates list
                all_updates.append(vX_recent)
                
                # Save the NetCDF
                print('Saving NetCDF file...')
                vDS.to_netcdf(path='output/vessel' + vessel + '_output.nc')
                print('NetCDF file Saved!')
                
                # Put the update numbers into the appropriate lists
                vX_update_number_list.append(vX_update_number)                    
                vX_updates_sending_list.append(vX_update_number)
                
                # Write the update file with the header as follows:
                
                '''    
                1 - Updates in this file
                2 - Updates Vessel A has sent or have been received from Vessel A
                3 - Updates Vessel B has sent or have been received from Vessel B
                '''
                
                print('Writing update to file...')
                with open('updates/vessel' + vessel + '_recent.csv', 'w', newline ='') as file:
                    csv_writer = csv.writer(file)
                    csv_writer.writerow(vX_updates_sending_list)
                    
                    if vessel == 'A':
                        csv_writer.writerow(vX_update_number_list)
                    elif vessel == 'B':
                        csv_writer.writerow(list(set(vO_updates_received_list)))
                        
                    if vessel == 'A':
                        csv_writer.writerow(list(set(vO_updates_received_list)))
                    elif vessel == 'B':
                        csv_writer.writerow(vX_update_number_list)
                        
                    for update_number in vX_updates_sending_list:
                        csv_writer.writerows(all_updates[update_number])
                
                # Clear the recent observation list and increment update number
                vX_recent = []
                vX_update_number += 1
                        
                print('Update written to file!')
                
                # Transfer the file to the other vessel (disabled on windows)
                '''
                print('Transferring file to other vessel...')
                arg = 'updates/vessel' + vessel + '_recent.csv'
                val = subprocess.check_call("./testbash2.sh '%s'" % arg, shell=True)                  # Rename the testbash2 script, maybe make it dynamic too
                print('File transfered!')
                '''
                
                # Read in the data from the other vessel
                print('Reading data from other vessel...')
                with open('transfered/vessel' + vessel_other + '_recent.csv', 'r', newline='') as csvfile:
                    csv_reader = csv.reader(csvfile)         
                    vO_this_update_list = [int(item) for item in next(csv_reader)]
                
                    for item in vO_this_update_list:
                        vO_updates_received_list.append(item)
                            
                    if vessel == 'A':
                        vX_on_vO_update_number_list = [int(item) for item in next(csv_reader)]                        
                        vO_update_number_list = [int(item) for item in next(csv_reader)]                        
                    elif vessel == 'B':
                        vO_update_number_list = [int(item) for item in next(csv_reader)]                
                        vX_on_vO_update_number_list = [int(item) for item in next(csv_reader)]
                
                    for row in csv_reader:
                        vDS['depth'].loc[dict(north=int(row[0]), east=int(row[1]), vessel=vessel_other)] = float(row[2])
                        vDS['soundings'].loc[dict(north=int(row[0]), east=int(row[1]), vessel=vessel_other)] = int(float(row[3]))
                        vDS['M2'].loc[dict(north=int(row[0]), east=int(row[1]), vessel=vessel_other)] = float(row[4])
                        vDS['stdev'].loc[dict(north=int(row[0]), east=int(row[1]), vessel=vessel_other)] = float(row[5])
                
                vX_update_number_set = set(vX_update_number_list)
                vX_on_vO_update_number_set = set(vX_on_vO_update_number_list)                
                vX_updates_sending_list = list(vX_update_number_set.difference(vX_on_vO_update_number_set))
                
                print('Data from other vessel read!')
                
        except KeyboardInterrupt:
            print('Shutting down ports...')
            ser.close()
            if reader != False:
                reader.pi.write(MOTOR_1_PIN,0)
                reader.pi.write(MOTOR_2_PIN,0)
                reader.end_listen()
            print('All closed up! Goodbye!')
            
            break
        
        except:
            pass
    
    
####################################
# Main Program
####################################
if __name__ == '__main__':
    
    # Open the data model for the vessel
    vX = xr.open_dataset('input/v' + vessel + '_empty.nc')
    print('Opening dataset for vessel ' + vessel)
    
    # Get the projection objects for use later
    projection, back_projection = set_geodetics(vX)
    
    if motor_control:
        # Initialize the SBUS reader on pin 4
        reader = read_sbus_from_GPIO.SbusReader(4)                                                  # Make the data pin configurable somewhere, maybe template metadata
        reader.begin_listen()
    else:
        reader = False
        
    # Start monitoring incoming data
    read_incoming_data(vX, vessel, projection, reader)

    


















