# Last Known Posistion is the last posistion the person was spotted at
# The Search Datum is the esitmated posistion of the person corrected for drift

import math
import random
import csv

#API imports
from xmlrpc.client import DateTime
from urllib3.util import url
import requests

#Point to other files in project
import Drone_Controller
import Launch_Parameters
import list_converter
import search_leg
import intersect_Calculator


#This function retrieves the water surface speed and direction from DMI's environmental prediction model
def find_drift_for_location(location):

    with open('c:\\users\\api-key.txt','r') as f:
        api_key = f.read().strip()

    url = "https://opendataapi.dmi.dk/v1/forecastedr/collections/dkss_idw/position"

    location_txt = f"POINT({location[1]} {location[0]})"
    input = {"coords": location_txt, "crs": "crs84", "parameter-name": "current-u,current-v"}
    header = {"X-Gravitee-Api-Key": api_key}

    r = requests.get(url, params=input, headers=header)
    r.raise_for_status()
    data = r.json()

    u = data["ranges"]["current-u"]["values"]
    v = data["ranges"]["current-v"]["values"]

    # ChatGPT made the formulas below
    direction_deg = (math.degrees(math.atan2(u[0], v[0])) + 360) % 360
    theta = math.radians(direction_deg)
    u_unit = math.sin(theta)
    v_unit = math.cos(theta)
    speed = u[0] * u_unit + v[0] * v_unit

    return direction_deg, speed

#This function retrieves the wind speed and direction from DMI
def api_wind_vector():
    url = "https://dmi.cma.dk/api/weather/forecast/Ish%C3%B8j?hours=1"
    try:
        data = requests.get(url).json()
    except Exception as e:
        raise RuntimeError(f"Somthing went wrong when fetching data: {e}")

    # The API puts weather values inside forecast[0]
    forecast = data.get("forecast", [{}])[0]

    wind_speed = forecast.get("wind_speed", 0)
    wind_dir   = forecast.get("wind_direction", 0)

    print("wind_dir:", wind_dir, "wind_speed:", wind_speed)


    return wind_speed, wind_dir

#This function creates the "ideal" expanding square pattern
def Expanding_Square_pattern(datum):
    # Sets the size of the value d in Expanding Square Searches
    d = Launch_Parameters.drone_FOV

    #Keeps track of the d_value currently in use
    current_d = d

    #Counts the amount of legs calculated so far
    counter = 1

    #Getting initial bearing from drift estimation
    current_bearing = Launch_Parameters.estimated_drift_bearing

    current_pos = datum

    # List for storing point in search pattern
    search_legs = []

    while counter < Launch_Parameters.expanding_square_count:

        #Calculating the next position to go to
        next_pos = Calc_pos(current_pos,current_bearing, current_d)

        new_leg = search_leg.Search_leg()
        new_leg.start_pos = current_pos
        new_leg.end_pos = next_pos
        new_leg.is_active = True

        search_legs.append(new_leg)
        current_pos = next_pos

        # calculating the new bearing for the next leg of the square
        if  current_bearing > 90:
            current_bearing -= 90
        else:
            current_bearing = 360 + (current_bearing - 90)

        if counter % 2 == 0:
            current_d += d

        counter += 1

    return search_legs

#This function selects the route through the expanding square pattern to modify it
def select_route_expanding_square(search_legs, target_pos, drone):

    # Used to keep track of which search legs have already been modified
    modified_search_legs = []

    # Testing intersections for all legs in pattern
    for leg in search_legs:
        modified_search_leg = intersect_Calculator.calc_intersec(Launch_Parameters.beach_plygon, leg)
        modified_search_legs.append(modified_search_leg)

    # Track unprocessed search legs
    unprocessed_indexes = []


    for i in range(len(modified_search_legs)):
        if modified_search_legs[i].is_active == True:
            unprocessed_indexes.append(i)

    flight_path = []
    current_leg_index = 0

    # Can be 1 or -1, an is used to determine the derection of flight in the pattern
    path_direction = 1

    while True:

        if len(unprocessed_indexes) == 0:
            break

        # Was put in to stop an index out of range error, due to some specific simulation conditions
        if current_leg_index < len(modified_search_legs):
            current_leg = modified_search_legs[current_leg_index]
        else:
            # break out of generation
            break

        # Is used to determine what position from the seach leg that should be appended to the flight_path
        # This only applies to legs that have no intersection
        if current_leg.is_active and current_leg.intersect_point == None:
            if path_direction == 1:
                flight_path.append(current_leg.end_pos)
            else:
                flight_path.append(current_leg.start_pos)

            unprocessed_indexes.remove(current_leg_index)
            current_leg_index += path_direction

        # Checks If the leg is active and have an intersection with the beach
        elif current_leg.is_active and current_leg.intersect_point is not None:

            flight_path.append(current_leg.intersect_point)
            unprocessed_indexes.remove(current_leg_index)

            if len(unprocessed_indexes) == 0:
                break

            next_leg_index = current_leg_index + path_direction
            if next_leg_index > len(modified_search_legs) -1:
                break
            else:
                next_leg = modified_search_legs[next_leg_index]

            if next_leg.is_active == False:

                if (next_leg_index) in unprocessed_indexes:
                    unprocessed_indexes.remove(next_leg_index)

                current_leg_index += 4

                if current_leg_index >= len(modified_search_legs):
                    current_leg_index = len(modified_search_legs) - 1

                    flight_path.append(modified_search_legs[current_leg_index].end_pos)

                path_direction *= -1

            else:
                if  next_leg.intersect_point is None:
                    if path_direction == 1:
                        flight_path.append(current_leg.end_pos)
                    else:
                        flight_path.append(current_leg.start_pos)

                current_leg_index += path_direction

        else:
            # We only get here if a leg is inactive and we cannot continue in the pattern. Then we break
            break

    flight_path.insert(0, drone.drone_base)
    flight_path.insert(1, datum)
    flight_path.append(drone.drone_base)

    return flight_path

#This function converts a list of search legs object to a flight path
def Convert_legs_to_route(legs):

    flight_path = []

    for leg in legs:
        flight_path.append(leg.start_pos)
        flight_path.append(leg.end_pos)

    flight_path.insert(0, drone.drone_base)
    flight_path.insert(1, datum)
    flight_path.append(drone.drone_base)

    return flight_path

#This function calculated the distance between to coordinates
def Calc_dist_to_point(current_pos, target_pos):

    # Math done by ChatGPT
    cur_lat_rad, cur_long_rad = math.radians(current_pos[0]), math.radians(current_pos[1])
    target_lat_rad, target_long_rad = math.radians(target_pos[0]), math.radians(target_pos[1])

    # Differences
    dlon = target_long_rad - cur_long_rad

    # Bearing calculation
    x = math.sin(dlon) * math.cos(target_lat_rad)
    y = math.cos(cur_lat_rad) * math.sin(target_lat_rad) - \
        math.sin(cur_lat_rad) * math.cos(target_lat_rad) * math.cos(dlon)
    bearing_rad = math.atan2(x, y)
    bearing_deg = (math.degrees(bearing_rad) + 360) % 360  # Normalize to 0–360°

    # Distance using the haversine formula
    R = 6371000  # Earth radius in meters
    dlat = target_lat_rad - cur_lat_rad
    a = math.sin(dlat/2)**2 + math.cos(cur_lat_rad) * math.cos(target_lat_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance_m = R * c

    return distance_m

#This function handles the movement of the drone object, and is called continually throughout the simulation loop
def Drone_movement(current_pos, target_pos):

    # Math done by ChatGPT
    cur_lat_rad, cur_long_rad = math.radians(current_pos[0]), math.radians(current_pos[1])
    target_lat_rad, target_long_rad = math.radians(target_pos[0]), math.radians(target_pos[1])

    # Differences
    dlon = target_long_rad - cur_long_rad

    # Bearing calculation
    x = math.sin(dlon) * math.cos(target_lat_rad)
    y = math.cos(cur_lat_rad) * math.sin(target_lat_rad) - \
        math.sin(cur_lat_rad) * math.cos(target_lat_rad) * math.cos(dlon)
    bearing_rad = math.atan2(x, y)
    bearing_deg = (math.degrees(bearing_rad) + 360) % 360  # Normalize to 0–360°

    # Distance using the haversine formula
    R = 6371000  # Earth radius in meters
    dlat = target_lat_rad - cur_lat_rad
    a = math.sin(dlat/2)**2 + math.cos(cur_lat_rad) * math.cos(target_lat_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance_m = R * c


    # Checking if distance to target is higher than speed in m.
    if distance_m >= Launch_Parameters.drone_cruise_speed:
        new_drone_pos = Calc_pos(current_pos, bearing_deg, Launch_Parameters.drone_cruise_speed)

    # if distance is less than speed, set posistion to target
    else:
        new_drone_pos = target_pos

    return new_drone_pos

#This function is used to calculate a new coordinate from an existing coordinate, a bearing and a distance
def Calc_pos(pos, bearing, distance):

    # Haversine Function Setup

    R = 6371000 #Radius of the Earth
    # Calculation from previous project, modified using ChatGPT (Next 6 lines)
    lat_rad = math.radians(pos[0])
    long_rad = math.radians(pos[1])
    bearing_rad = math.radians(bearing)

    new_lat_rad = math.asin(math.sin(lat_rad) * math.cos(distance / R) + math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad))

    new_long_rad = long_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
        math.cos(distance / R) - math.sin(lat_rad) * math.sin(new_lat_rad)
    )

    new_lat = math.degrees(new_lat_rad)
    new_long = math.degrees(new_long_rad)

    new_position = (new_lat,new_long)

    return new_position

#This function call the fin_drift_for_location function and build a drift pattern from the data recieved.
def create_drift_pattern(drift_data, person_pos):
    drift = []
    pos = person_pos
    new_pos = (0,0)
    for i in range(Launch_Parameters.drift_length_seconds):
        new_pos = Calc_pos(pos,drift_data[0], drift_data[1])

        # if the person drifted ashore, they will just lay there for the remainder of the generation.
        if not intersect_Calculator.calc_point_in_poly(Launch_Parameters.beach_plygon, new_pos):
            pos = new_pos

        drift.append(pos)

    return drift

#Generates the sweep pattern
def SweepSearch(target_pos):

    datum = Calc_pos(target_pos, Launch_Parameters.estimated_drift_bearing - 180, Launch_Parameters.drone_FOV * 3)
    flight_path = []

    sweep_angle = Launch_Parameters.sweep_angle

    flight_path.append(datum)
    angle_rad = math.radians(sweep_angle/2)
    dist_to_point = (math.sin(angle_rad) * Launch_Parameters.drone_FOV) / math.sin((math.pi / 2) - angle_rad)

    for i in range(1,70):
        mid_point = Calc_pos(datum, Launch_Parameters.estimated_drift_bearing, Launch_Parameters.drone_FOV * i)

        if intersect_Calculator.calc_point_in_poly(Launch_Parameters.beach_plygon, mid_point):
            break

        left_point = Calc_pos(mid_point, Launch_Parameters.estimated_drift_bearing - 90, dist_to_point * i)
        intersect_point = intersect_Calculator.calc_intersect_from_pos(mid_point, left_point, Launch_Parameters.beach_plygon)

        if intersect_point is not None:
            left_point = intersect_point

        right_point = Calc_pos(mid_point, Launch_Parameters.estimated_drift_bearing + 90, dist_to_point * i)
        intersect_point = intersect_Calculator.calc_intersect_from_pos(mid_point, right_point, Launch_Parameters.beach_plygon)
        if intersect_point is not None:
            right_point = intersect_point

        if i%2 == 0:
            flight_path.append(right_point)
            flight_path.append(left_point)
        else:
            flight_path.append(left_point)
            flight_path.append(right_point)

    flight_path.insert(0, drone.drone_base)
    flight_path.append(drone.drone_base)

    return flight_path

#Generates the sector seach pattern
#(Could take the drift pattern as an input, and use the drift speed to define radius of sector. This was however cut for time, and a fixed value selected)
def SectorSearch(datum, drift_direction):

    d = Launch_Parameters.drone_FOV * Launch_Parameters.sector_diameter_multiplier
    current_pos = datum
    dir = drift_direction
    flight_path = []

    for search in range(3):
        for leg in range(7):
            if leg in [2,4]:
                temp_d = d
            else:
                temp_d = d/2

            next_pos = Calc_pos(current_pos, dir, temp_d)
            flight_path.append(next_pos)
            dir += 120
            if dir > 360:
                dir = dir - 360

            current_pos = next_pos

        dir += 30
        if dir > 360:
            dir -= 360

        # After the first search, a new datum could be calculated and used to move the search pattern in the drift direction.
        # We Chose not to implement it, due to both low drift speeds along coastlines and lack of time.

    flight_path.insert(0, drone.drone_base)
    flight_path.insert(1, datum)
    flight_path.append(drone.drone_base)

    return flight_path

#Generates the line search patten
def LineSearch(datum, drift_direction):

    d = Launch_Parameters.drone_FOV * 5
    current_pos = datum
    dir = drift_direction - 90
    if dir < 0:
        dir = 360 - dir

    flight_path = []


    for leg in range(30):
        if leg == 0:
            temp_d = d/2
        else:
            if leg % 2 == 0:
                temp_d = d
            else:
                temp_d = Launch_Parameters.drone_FOV

        next_pos = Calc_pos(current_pos, dir, temp_d)
        flight_path.append(next_pos)

        if leg % 4 in (0, 1):
            dir += 90
            if dir > 360:
                dir = dir - 360
        else:
            dir -= 90
            if dir < 0:
                dir = 360 - dir

        current_pos = next_pos

    flight_path.insert(0, drone.drone_base)
    flight_path.insert(1, datum)
    flight_path.append(drone.drone_base)

    return flight_path

#The main simulation loop that simulates a single "mission"
def simulation(drone, flight_path, drift_pattern ):

    running = True
    # Keeps track of where in the search pattern the drone is
    search_pattern_step = 0
    path_flown = []
    person_found = False
    while running:

        path_flown.append(drone.position)
        drone_new_pos = Drone_movement(drone.position, flight_path[search_pattern_step])

        if drone_new_pos == drone.position:

            # Penalty for changing direction
            drone.flight_time += 2

            # Increments the counter, keeping track of progress in pattern
            search_pattern_step += 1

            if search_pattern_step >= len(flight_path):
                break

        dist_to_person = Calc_dist_to_point(drone_new_pos, drift_pattern[drone.flight_time.__floor__()])

        if dist_to_person <= Launch_Parameters.drone_FOV:
            print("Person found at: " + str(drift_pattern[drone.flight_time.__floor__()]))
            person_found = True
            break

        dist_flown = Calc_dist_to_point(drone.position, drone_new_pos)
        drone.distance_flown += dist_flown
        time_flown = dist_flown / Launch_Parameters.drone_cruise_speed
        drone.flight_time += time_flown
        drone.battery_Wh_left -= (time_flown/3600) * drone.discharge_rate
        dist_home = Calc_dist_to_point(drone_new_pos, drone.drone_base)
        flight_time_home = dist_home/Launch_Parameters.drone_cruise_speed
        wh_home = (flight_time_home/3600) * drone.discharge_rate

        if drone.battery_Wh_left - wh_home <= (drone.battery_full_capacity / 10):
            search_pattern_step = len(flight_path) - 1

        drone.position = drone_new_pos

    print(drone.flight_time)
    print(drone.distance_flown)
    #list_converter.save_kml(path_flown, "C:\\users\\bena3\\downloads\\Path_Flown.kml", "Path_Flown")

    return drone.flight_time, drone.distance_flown, person_found


with open('data.csv', 'w', newline='') as csvfile:
    data_writer = csv.writer(csvfile, delimiter=',', quotechar='"')
    data_writer.writerow(["Simulation ID","Pattern Type", "Flight Time", "Distance Flown", "Person Found", "Estimated Position Lat",
                          "Estimated Position Lon", "Actual Position Lat", "Actual Position Lon",
                          "Deviation Direction", "Deviation Distance", "Drift Direction", "Drift Speed", "Time Since Contact", "Distance To Shore"]
                         )

# Default = 360
max_dev_dir = 360
#Default = 50
max_dev_dist = 50

person_pos = Launch_Parameters.last_known_position

for sim_id in range(400):

    print("SimID: " + str(sim_id))

    # generating a new last_known_position for use in next set of simulations
    while True:

        rand_pos = (random.uniform(55.587897 ,55.598510), random.uniform(12.375741, 12.419301))

        # checks if rand_pos is on beach, and if not, saves position for use in simulation
        if not intersect_Calculator.calc_point_in_poly(Launch_Parameters.beach_plygon, rand_pos):
            Launch_Parameters.last_known_position = rand_pos
            break

    while True:
        deviation_dir = random.randrange(0,max_dev_dir)
        Launch_Parameters.estimated_drift_bearing = deviation_dir
        deviation_dist = random.randrange(0, max_dev_dist)

        person_pos = Calc_pos(Launch_Parameters.last_known_position, deviation_dir, deviation_dist)

        # checks if the shifted position is on the beach, and retries until the shifted point is in water
        if not intersect_Calculator.calc_point_in_poly(Launch_Parameters.beach_plygon, person_pos):
            break

    Launch_Parameters.time_since_contact = random.randrange(0, 600)

    #drift_data = find_drift_for_location(person_pos) #Remove comment to run with API calls
    drift_data = (random.randrange(0,360), random.random())
    #drift_data = (Launch_Parameters.estimated_drift_bearing, random.random())
    drift_pattern = create_drift_pattern(drift_data, person_pos)

    # Target_pos is the Search Datum the first time it runs.
    datum = Calc_pos(Launch_Parameters.last_known_position, drift_data[0], drift_data[1] * Launch_Parameters.time_since_contact)

    list_converter.save_kml(drift_pattern,  "C:\\users\\bena3\\downloads\\drift.kml", "drift")
    distance_to_shore = intersect_Calculator.calc_dist_to_poly(Launch_Parameters.beach_plygon, person_pos)

    for pattern in range(5):

        # Creating Drone object
        drone =  Drone_Controller.Drone_Controller()
        drone.position = drone.drone_base


        if pattern == 0:
            path_type = "Expanding Square"
            search_legs = Expanding_Square_pattern(datum)
            flight_path = Convert_legs_to_route(search_legs)
            list_converter.save_kml(flight_path,  "C:\\users\\bena3\\downloads\\Expanding_Square.kml", "Expanding Square")

        elif pattern == 1:
            path_type = "Expanding Square Adaptive"
            search_legs = Expanding_Square_pattern(datum)
            flight_path = select_route_expanding_square(search_legs, datum, drone)
            list_converter.save_kml(flight_path,  "C:\\users\\bena3\\downloads\\Expanding_Square_Adaptive.kml", "Expanding Square Adaptive")

        elif pattern == 2:
            path_type = "Line Search"
            flight_path = LineSearch(datum, drift_data[0])
            list_converter.save_kml(flight_path,  "C:\\users\\bena3\\downloads\\Line_Search.kml", "Line Search")

        elif pattern == 3:
            path_type = "Sweep Adaptive"
            flight_path = SweepSearch(datum)
            list_converter.save_kml(flight_path,  "C:\\users\\bena3\\downloads\\Sweep_Adaptive.kml", "Sweep Adaptive")

        elif pattern == 4:
            path_type = "Sector Search"
            flight_path = SectorSearch(datum, drift_data[0])
            list_converter.save_kml(flight_path,  "C:\\users\\bena3\\downloads\\Sector_Search.kml", "Sector Search")


        flight_time, distance_flown, person_found = simulation(drone, flight_path, drift_pattern)


        with open('data.csv', 'a', newline='') as csvfile:
            data_writer = csv.writer(csvfile, delimiter=',', quotechar='"')
            data_writer.writerow([sim_id,path_type, flight_time,
                                  distance_flown, person_found,
                                  Launch_Parameters.last_known_position[0], Launch_Parameters.last_known_position[1],
                                  person_pos[0], person_pos[1], deviation_dir,
                                  deviation_dist, drift_data[0],
                                  drift_data[1], Launch_Parameters.time_since_contact, distance_to_shore]
                                 )

    #list_converter.save_kml(flight_path,  "C:\\users\\bena3\\downloads\\Line_s.kml", "Line_S")






