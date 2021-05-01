'''
Declarations of Follower States that uses the main State framework.
Written by Svena Yu, April 2021 
'''
import rospy

class State:
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """
    def __init__(self):
        self.min_camera_distance = rospy.get_param('/min_camera_distance')
        self.max_buffer_len_when_lost = 15
        # print 'Processing current state:', str(self)

    def on_event(self, ):
        """
        Handle events that are delegated to this State.
        """
        pass

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__
        
'''
Start of our states. wp represents current waypoint goal pose (Pose)
dist represents current distance reading (Float32)
'''

# At rest before new waypoints are received
class F_STATE_0(State):
    def on_event(self, bffer, dist):
        if dist == -1:
            return F_STATE_2()
        elif len(bffer) >= 1:
            return F_STATE_1()
        return self

# Moving to new waypoint goals, leader is still in view
class F_STATE_1(State):
    def on_event(self, bffer, dist):
        if len(bffer) == 0:
            return F_STATE_0()
        elif len(bffer) < self.max_buffer_len_when_lost and dist == -1.0:
            return F_STATE_2()
        elif dist == -1.0:
            return F_STATE_3() # leader too far, has to stop
        return self

# Lost sight of leader
# Continuing on accumulated waypoint path and if sees leader, goes back to STATE 1
class F_STATE_2(State):
    def on_event(self, bffer, dist):
        if len(bffer) >= 1 and dist != -1.0:
            return F_STATE_1()
        elif len(bffer) == 0 and dist == -1.0:
            return F_STATE_3()
        return self

# Leader stopping, robot rotating to find leader
class F_STATE_3(State):
    def on_event(self, bffer, dist):
        if len(bffer) >= 1 and dist != -1.0:
            return F_STATE_1()
        elif dist != -1.0:
            return F_STATE_4()
        return self

# Leader still stopped, follower gets new waypoints when it is also stopped
class F_STATE_4(State):
    def on_event(self, bffer, dist):
        if len(bffer) >= 1:
            return F_STATE_1()
        return self