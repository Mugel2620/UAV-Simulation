# This program is to simulate the drone controller, so it can be isolated from the simulation

class Drone_Controller:
    #Static Variables
    drone_base = (55.607124, 12.393114)

    battery_full_capacity = 100
    discharge_rate = 200 # in watt

    def __init__(self):
        # Variables kept at a pr. instance level
        self.velocity = 0
        self.bearing = 0
        self.altitude = 0
        self.position = (0,0)
        self.flight_time = 0
        self.battery_Wh_left = 100 # in watt hours
        self.distance_flown = 0