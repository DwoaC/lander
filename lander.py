'''
Solution to the codingame.com puzzle [Mars Lander](https://www.codingame.com/training/medium/mars-lander-episode-2)

The goal here is land a moon lander on a terrain at a particular spot.

The game handles the physics of the lander and feeds the lander's velocity and location
into the code each tick.

The code can instruct the lander to rotate clockwise or anit-clockwise and to
fire the thrusters at one of 3 settings (off, low and high).

An optimal solution could be to solve the equations of motion for the lander
and pilot a parabolic path to the landing spot.  Given we don't know the
mass and the rotation of the lander is not instantaneous this would involve
a great deal of testing to develop a model for the lander.  Besides codeingame
puzzles often like to change parameters on you to ensure your solution is
as general as possible.  So...this seems like a great chance to build to use
the state design pattern from the Gang of Four.

So the solution is to implement a Lander class.

The class will have several states and move between the states depending on
the current micro-goal in the solution.

For instance one state is to drive the lander to the landing spot.  Once
the lander is over the stop we transition to a desent state who's goal
is to reduce the lander's height as quickly as possible while not making
it impossible to slow down to land.  Finally a landing state would reduce
the descent to a safe speed to land.
'''

import sys

# Auto-generated code below aims at helping you parse
# the standard input according to the problem statement.
import collections

Point = collections.namedtuple('Point', ['x', 'y'])


class LandingSpot:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.left = start.x
        self.right = end.x
        self.height = start.y

    def __str__(self):
        return 'Safe Spot from {} to {}'.format(self.start, self.end)


class Surface(collections.MutableSequence):
    '''
    List like class to hold the data about the surface.

    Only special method is find_safe_spot that returns a LandingSpot
    object which is used as a goal during landing.
    '''
    def __init__(self, *args):
        self.list = list()
        self.extend(list(args))

    def __len__(self):
        return len(self.list)

    def __getitem__(self, i):
        return self.list[i]

    def __delitem__(self, i):
        del self.list[i]

    def __setitem__(self, i, v):
        self.list[i] = v

    def insert(self, i, v):
        self.list.insert(i, v)

    def __str__(self):
        return str(self.list)

    def find_safe_spot(self):
        old_spot = self.list[0]
        for new_spot in self.list[1:]:
            print(old_spot, new_spot, file=sys.stderr)
            if new_spot.y == old_spot.y and new_spot.x - old_spot.x >= 1000:
                return LandingSpot(old_spot, new_spot)
            old_spot = new_spot
        raise Exception('Failed to find safe spot')


class Lander:
    '''
    Control model of the Lander.

    state is the current state of the lander.  The initial state is Hover.

    State transitinos are handled by the state it self.  The lander's responsibility
    is in holding the current state and parameters of the lander and communicating
    it the state's control outputs to the game.
    '''
    def __init__(self, surface):
        self.surface = surface
        self.x = None
        self.y = None
        self.speed_h = None
        self.speed_v = None
        self.fuel = None
        self.rotate = None
        self.power = None
        self.state = Hover(self, None)
        self.landing_spot = self.surface.find_safe_spot()

    @property
    def is_over_landing_spot(self):
        return self.landing_spot.left <= self.x <= self.landing_spot.right

    @property
    def altitude(self):
        return self.y - self.landing_spot.height

    def update_state(self, x, y, speed_h, speed_v, fuel, rotate, power):
        '''Takes the lander state from the game and updates the landers internal parameters.'''
        self.x_old = self.x
        self.y_old = self.y
        self.speed_h_old = self.speed_h
        self.speed_v_old = self.speed_v
        self.fuel_old = self.fuel
        self.rotate_old = self.rotate
        self.power_old = self.power

        self.x = x
        self.y = y
        self.speed_h = speed_h
        self.speed_v = speed_v
        self.fuel = fuel
        self.rotate = rotate
        self.power = power

    def control(self):
        '''Communicates the control instructions to the game.'''
        print('{} {}'.format(self.control_rotate, self.control_power))

    @property
    def control_rotate(self):
        return self.state.control_rotate

    @property
    def control_power(self):
        return self.state.control_power

    @property
    def is_left_of_landing_spot(self):
        return self.x < self.landing_spot.left

    @property
    def is_right_of_landing_spot(self):
        return self.x > self.landing_spot.right

    @property
    def distance_to_landing_spot(self):
        if self.is_left_of_landing_spot:
            return self.x - self.landing_spot.left
        elif self.is_right_of_landing_spot:
            return self.x - self.landing_spot.right

class State:

    MAX_ROTATION = 90
    ROTATION_RATE_OF_CHANGE = 15
    MIN_POWER = 0
    MAX_POWER = 4

    def __init__(self, lander, next_state=None):
        '''
        Base class for all states.

        To modify the behaviour of the state override update_rotation and update_power.  These
        methods are called each tick of the simulation and are expected to return the goals
        for the rotation and power control methods.

        control_rotate and control_power return the desired outputs the lander should feed
        to the simulation to achieve the goals provided by the update_rotation and
        update_power methods.  It should not be neccessary to override these methods.

        MAX_ROTATION - the lander won't try to rotate passed this angle.  90 degrees
        corresponds to horizontal.

        ROTATION_RATE_OF_CHANGE - The lander will change its rotation by this much
        if its current rotation does not equal its desired rotation.

        :param lander: (Lander) object to control
        :param next_state: (State) the state to transition to next if
        goal is reached.
        '''
        self.lander = lander
        self.rotate_limit = self.MAX_ROTATION
        self.next_state = next_state

    @property
    def control_rotate(self):
        '''
        Calculate a control signal to send to the game.

        Limit the signal to the max rotation
        :return:
        '''
        desired_rotation = round(self.update_rotation())

        if desired_rotation > self.rotate_limit:
            return self.rotate_limit
        elif desired_rotation < -self.rotate_limit:
            return -self.rotate_limit
        else:
            return desired_rotation

    @property
    def rotate_left_limit(self):
        return self.lander.rotate - self.ROTATION_RATE_OF_CHANGE

    @property
    def rotate_right_limit(self):
        return self.lander.rotate + self.ROTATION_RATE_OF_CHANGE

    @property
    def control_power(self):
        '''
        Calculate the power signal to send to the game.

        There are 4 power levels.  0 is trust off.  4 is max trust.

        The power is rounded to integer values.

        :return: int
        '''
        desired_power = round(self.update_power())

        if desired_power < self.MIN_POWER:
            return self.MIN_POWER
        elif desired_power > self.MAX_POWER:
            return self.MAX_POWER
        else:
            return desired_power

    def update_rotation(self):
        return 0

    def update_power(self):
        return 4


class Hover(State):
    def __init__(self, lander, next_state=None):
        '''
        Hover steadily in horizontal and vertical axis.

        Rotation is negitive feedback loop using speed_h as the control

        Power is 3 if speed_v is any positive value. Otherwise its 4.

        There is no goal and no automatic switch to a new state.
        '''
        super().__init__(lander, next_state=next_state)

    def update_rotation(self):
        return self.lander.speed_h

    def update_power(self):
        if self.lander.speed_v > 0:
            return 3
        else:
            return 4


class StopOverLandingSpot(Hover):
    MAX_SPEED = 20
    MAX_ANGLE = 45
    HOVER_ANGLE = 0

    def __init__(self, lander, next_state):
        '''
        Move in the direction of the landing spot then stop and hover.

        To prevent overshot we don't let the speed go above MAX_SPEED and we don't
        use an angle greater than MAX_ANGLE.  Definite room to optimize these
        values.

        The goal is to get over the landing_spot and be at a safe speed.

        This state will transition to next_state when the goal is reached.
        '''
        super().__init__(lander, next_state=next_state)
        self.scale = 0.04
        self.next_state = Descend

    def update_rotation(self):
        if self.is_too_fast:
            return self.transition_to_stop()
        elif self.lander.is_over_landing_spot:
            return self.stop_and_transition_to_next_state()
        else:
            return self.move_towards_landing_spot()

    def stop_and_transition_to_next_state(self):
        if self.has_safe_speed:
            self.lander.state = self.next_state(self.lander)
            return self.lander.state.update_rotation()
        else:
            return self.transition_to_stop()

    def move_towards_landing_spot(self):
        if self.lander.is_left_of_landing_spot:
            if self.lander.speed_h < self.MAX_SPEED:
                return self.lander.distance_to_landing_spot * self.scale
            elif self.lander.speed_h > self.MAX_SPEED:
                return self.MAX_ANGLE
            else:
                return self.HOVER_ANGLE

        if self.lander.is_right_of_landing_spot:
            if self.lander.speed_h > -self.MAX_SPEED:
                return self.lander.distance_to_landing_spot * self.scale
            elif self.lander.speed_h < -self.MAX_SPEED:
                return -self.MAX_ANGLE
            else:
                return self.HOVER_ANGLE

    @property
    def is_too_fast(self):
        return not (-self.MAX_SPEED <= self.lander.speed_h <= self.MAX_SPEED)

    @property
    def has_safe_speed(self):
        return -self.MAX_SPEED <= self.lander.speed_h <= self.MAX_SPEED

    def transition_to_stop(self):
        self.lander.state = Stop(lander, StopOverLandingSpot)
        return self.lander.state.update_rotation()

    def update_power(self):
        if self.lander.altitude < 100 and not (self.lander.is_over_landing_spot):
            return 4
        if self.lander.speed_v > -18:
            return 3
        else:
            return 4


class Descend(Hover):
    SAFE_DESCENT_SPEED = -18
    LANDING_ALTITUDE = 100

    def __init__(self, lander, next_state=None):
        '''
        Descend at a safe speed and stop horizontal movement.

        This is a final state.
        '''
        super().__init__(lander, next_state=next_state)
        self.scale = 0.01

    def update_power(self):
        if self.is_not_safe_to_land:
            return 4
        if self.descent_is_too_fast:
            return 4
        else:
            return 3

    def update_rotation(self):
        if self.lander.altitude > self.LANDING_ALTITUDE:
            return self.lander.speed_h
        else:
            return 0

    @property
    def is_not_safe_to_land(self):
        return (self.lander.altitude < self.LANDING_ALTITUDE and
                not self.lander.is_over_landing_spot)

    @property
    def descent_is_too_fast(self):
        return self.lander.speed_v < self.SAFE_DESCENT_SPEED

class Stop(Hover):
    def __init__(self, lander, next_state):
        '''
        Stop horizontal moverment as fast as possible attempt to hold altitude.
        '''
        super().__init__(lander, next_state)
        self.scale = 0.5
        self.cutoff = 20

    def update_rotation(self):
        if -self.cutoff <= self.lander.speed_h <= self.cutoff:
            self.lander.state = self.next_state(self.lander, None)
            return self.lander.state.update_rotation()
        return self.lander.speed_h * self.scale


def codingame_initilisation():
    '''Parses the inputs provide by codeingame.com'''

    surface_n = int(input())  # the number of points used to draw the surface of Mars.
    surface = Surface()
    for i in range(surface_n):
        land_x, land_y = [int(j) for j in input().split()]
        surface.append(Point(land_x, land_y))

    print(surface.find_safe_spot(), file=sys.stderr)
    return surface

if __name__=='__main__':
    surface = codingame_initilisation()
    lander = Lander(surface)
    lander.state = StopOverLandingSpot(lander)
    previous_state = lander.state.__class__
    # game loop
    while True:
        # h_speed: the horizontal speed (in m/s), can be negative.
        # v_speed: the vertical speed (in m/s), can be negative.
        # fuel: the quantity of remaining fuel in liters.
        # rotate: the rotation angle in degrees (-90 to 90).
        # power: the thrust power (0 to 4).
        # x, y, h_speed, v_speed, fuel, rotate, power = [int(i) for i in input().split()]
        args = [int(i) for i in input().split()]
        lander.update_state(*args)
        lander.control()
        print(lander.state.__class__, file=sys.stderr)
