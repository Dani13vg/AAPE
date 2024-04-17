import math
import random
import asyncio
import Sensors
import time


def calculate_distance(point_a, point_b):
    distance = math.sqrt((point_b['x'] - point_a['x']) ** 2 +
                         (point_b['y'] - point_a['y']) ** 2 +
                         (point_b['z'] - point_a['z']) ** 2)
    return distance


class DoNothing:
    """
    Does nothing
    """
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state

    async def run(self):
        print("Doing nothing")
        await asyncio.sleep(1)
        return True


class ForwardDist:
    """
        Moves forward a certain distance specified in the parameter "dist".
        If "dist" is -1, selects a random distance between the initial
        parameters of the class "d_min" and "d_max"
    """
    STOPPED = 0
    MOVING = 1
    END = 2

    def __init__(self, a_agent, dist, d_min, d_max):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.original_dist = dist
        self.target_dist = dist
        self.d_min = d_min
        self.d_max = d_max
        self.starting_pos = a_agent.i_state.position
        self.state = self.STOPPED

    async def run(self):
        try:
            while True:
                if self.state == self.STOPPED:
                    # starting position before moving
                    self.starting_pos = self.a_agent.i_state.position
                    # Before start moving, calculate the distance we want to move
                    if self.original_dist < 0:
                        self.target_dist = random.randint(self.d_min, self.d_max)
                    else:
                        self.target_dist = self.original_dist
                    # Start moving
                    await self.a_agent.send_message("action", "mf")
                    self.state = self.MOVING
                    # print("TARGET DISTANCE: " + str(self.target_dist))
                    # print("MOVING ")
                elif self.state == self.MOVING:
                    # If we are moving, check if we already have covered the required distance
                    current_dist = calculate_distance(self.starting_pos, self.i_state.position)
                    if current_dist >= self.target_dist:
                        await self.a_agent.send_message("action", "stop")
                        self.state = self.STOPPED
                        return True
                    else:
                        await asyncio.sleep(0)
                else:
                    print("Unknown state: " + str(self.state))
                    return False
        except asyncio.CancelledError:
            print("***** TASK Forward CANCELLED")
            await self.a_agent.send_message("action", "stop")
            self.state = self.STOPPED


class Turn:
    """
    Repeats the action of turning a random number of degrees in a random
    direction (right or left)
    """
    LEFT = -1
    RIGHT = 1

    SELECTING = 0
    TURNING = 1

    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.rc_sensor = a_agent.rc_sensor
        self.i_state = a_agent.i_state
        self.rotation_amount = 45
        self.prev_rotation = 0
        self.accumulated_rotation = 0
        self.direction = self.RIGHT
        self.state = self.SELECTING

    async def run(self):
        try:
            while True:
                if self.state == self.SELECTING:
                    self.rotation_amount = random.randint(10, 90)
                    print("Degrees: " + str(self.rotation_amount))
                    self.direction = random.choice([self.LEFT, self.RIGHT])
                    if self.direction == self.RIGHT:
                        await self.a_agent.send_message("action", "tr")
                        # print("Direction: RIGHT")
                    else:
                        await self.a_agent.send_message("action", "tl")
                        # print("Direction: LEFT")
                    self.prev_rotation = self.i_state.rotation["y"]
                    self.accumulated_rotation = 0
                    self.state = self.TURNING
                    # print("TURNING...")
                elif self.state == self.TURNING:
                    # check if we have finished the rotation
                    current_rotation = self.i_state.rotation["y"]
                    if self.direction == self.RIGHT:
                        if self.prev_rotation > current_rotation: # complete 360 turn clockwise
                            self.accumulated_rotation += 360 - self.prev_rotation + current_rotation
                        else:
                            self.accumulated_rotation += current_rotation - self.prev_rotation
                    else:
                        if self.prev_rotation < current_rotation: # complete 260 turn counter-clockwise
                            self.accumulated_rotation += 360 - current_rotation + self.prev_rotation
                        else:
                            self.accumulated_rotation += self.prev_rotation - current_rotation
                    self.prev_rotation = current_rotation

                    if self.accumulated_rotation >= self.rotation_amount:
                        # We are there
                        # print("TURNING DONE.")
                        await self.a_agent.send_message("action", "nt")
                        self.accumulated_rotation = 0
                        self.direction = self.RIGHT
                        self.state = self.SELECTING
                        return True
                await asyncio.sleep(0)
        except asyncio.CancelledError:
            print("***** TASK Turn CANCELLED")
            await self.a_agent.send_message("action", "nt")


class Avoid:
    """
    obstacle avoidance class that uses both ForwardDist and Turn classes for movement and turning.
    """
    FORWARD = 0
    AVOIDING = 1

    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.state = self.FORWARD
        self.rc_sensor = a_agent.rc_sensor
        self.forward_task = ForwardDist(a_agent, -1, 1, 5)  # Parameters can be adjusted
        self.turn_task = None

    async def run(self):
        try:
            while True:
                sensor_hit = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.HIT]
                sensor_distances = self.rc_sensor.sensor_rays[Sensors.RayCastSensor.DISTANCE]

                if self.state == self.FORWARD:
                    if any(sensor_hit):
                        # If there are obstacles, initiate avoiding state and start turning
                        self.state = self.AVOIDING
                        left_proximity = sum([sensor_distances[i] for i in range(len(sensor_distances)//2) if sensor_hit[i]])
                        right_proximity = sum([sensor_distances[i] for i in range(len(sensor_distances)//2 + 1, len(sensor_distances)) if sensor_hit[i]])

                        if left_proximity < right_proximity:
                            self.turn_task = Turn(self.a_agent)
                            self.turn_task.direction = Turn.LEFT
                        else:
                            self.turn_task = Turn(self.a_agent)
                            self.turn_task.direction = Turn.RIGHT

                        asyncio.create_task(self.turn_task.run())
                    else:
                        # If no obstacles, continue moving forward
                        if not self.forward_task.state == ForwardDist.MOVING:
                            asyncio.create_task(self.forward_task.run())

                elif self.state == self.AVOIDING:
                    if self.turn_task and not self.turn_task.state == Turn.SELECTING:
                        result = True
                        await asyncio.sleep(0.1)  # Wait for turn to complete
                        
                    else:
                        # Turn completed, go back to forward state
                        self.state = self.FORWARD if random.random() < 0.8 else self.AVOIDING

                await asyncio.sleep(0.2)  # Reacting time.

        except asyncio.CancelledError:
            print("***** TASK Avoid CANCELLED")
            if self.forward_task:
                self.forward_task.cancel()  # Ensure forward task is also cancelled if needed
            if self.turn_task:
                self.turn_task.cancel()
            await self.a_agent.send_message("action", "stop")

class EatFlower:
    def __init__(self, a_agent):
        self.a_agent = a_agent
        self.i_state = a_agent.i_state

    async def run(self):
    
        print("Eating...")
        await asyncio.sleep(5)  # Stop when flower detected for 5 secs.
        self.a_agent.i_state.update_hunger(False)
        self.a_agent.i_state.update_lunch_time(time.time())
        print("Flower eaten!")
        return True
      


      






