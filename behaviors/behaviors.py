""" A module containing all the behaviors the robot can use """
from enum import IntEnum
import numpy as np
from behaviors.common_behaviors import Behavior, ActionBehavior
import behaviors.common_behaviors
import py_trees as pt


def get_node(node_descriptor, world_interface, verbose = False):
    """ Returns a node object given the descriptor string """
    return behaviors.common_behaviors.get_node(node_descriptor, world_interface, verbose)

def compatible(_condition1, _condition2):
    """ TODO this is just temp to get it to run, needs to be fixed if we want priorities to work"""
    return True

class AtPos(Behavior):
    """
    Check if object is at position
    """
    def __init__(self, name, parameters, world_interface, _verbose=False):
        name = AtPos.to_string(parameters)
        super().__init__(name, parameters, world_interface)

    @staticmethod
    def check_string_match(node_string, parameters):
        """ Check if the string description of the node matches this node """
        for relation in parameters["relation"].list_of_values:
            if " " + relation + " " in node_string:
                return True
        return False

    @staticmethod
    def to_string(parameters):
        """ Creates a string """
        node_string = parameters["target_object"]
        node_string += " " + parameters["relation"]
        node_string += " " + parameters["relative_object"]
        node_string += "?"
        return Behavior.common_string_rules(node_string, parameters)

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = {}
        n_marks = 4
        marks = [0] * n_marks
        marks[0] = node_descriptor.find('"')
        for i in range(1, n_marks):
            if marks[i - 1] >= 0:
                marks[i] = node_descriptor.find('"', marks[i - 1] + 1)
            else:
                print("Error, parameter parsing failed")
                return None

        parameters["target_object"] = node_descriptor[marks[0]: marks[1] + 1]
        parameters["relation"] = node_descriptor[marks[1] + 2: marks[2] - 1]
        parameters["relative_object"] = node_descriptor[marks[2]: marks[3] + 1]
        parameters["not"] = node_descriptor[0] == "~"

        return parameters

    def __eq__(self, other) -> bool:
        if not isinstance(other, AtPos):
            # don't attempt to compare against unrelated types
            return False
        return super().__eq__(other)

    def update(self):
        target_object = self.parameters["target_object"]
        relation = self.parameters["relation"]
        relative_object = self.parameters["relative_object"]
        if target_object == '"any object"':
            object_at = False
            for movable_object in self.world_interface.movable_objects:
                if self.world_interface.object_at(movable_object, relation, relative_object):
                    object_at = True
                    break
            return self.check_negated(object_at)
        else:
            return self.check_negated(self.world_interface.object_at(target_object, relation, relative_object))

class Grasped(Behavior):
    """
    Check if object is grasped
    """
    def __init__(self, name, parameters, world_interface, _verbose=False):
        name = Grasped.to_string(parameters)
        super().__init__(name, parameters, world_interface)

    def __eq__(self, other) -> bool:
        if not isinstance(other, Grasped):
            # don't attempt to compare against unrelated types
            return False
        return super().__eq__(other)

    @staticmethod
    def to_string(parameters):
        """ Creates a string """
        node_string = 'grasped ' + parameters["target_object"]
        node_string += "?"
        return Behavior.common_string_rules(node_string, parameters)

    def update(self):
        target_object = self.parameters["target_object"]
        if target_object == '"any object"':
            return self.check_negated(self.world_interface.get_grasped_object() is not None)
        else:
            return self.check_negated(self.world_interface.get_grasped_object() == target_object)

class LocationKnown(Behavior):
    """
    Check if object location is known
    """
    def __init__(self, name, parameters, world_interface, _verbose=False):
        name = LocationKnown.to_string(parameters)
        super().__init__(name, parameters, world_interface)

    def __eq__(self, other) -> bool:
        if not isinstance(other, LocationKnown):
            # don't attempt to compare against unrelated types
            return False
        return super().__eq__(other)

    @staticmethod
    def to_string(parameters):
        """ Creates a string """
        node_string = parameters["target_object"] + " location known?"
        return Behavior.common_string_rules(node_string, parameters)

    def update(self):
        return self.check_negated(self.world_interface.object_position_known[self.parameters["target_object"]])

class Upright(Behavior):
    """
    Check if object is standing upright
    """
    def __init__(self, name, parameters, world_interface, _verbose=False):
        name = Upright.to_string(parameters)
        super().__init__(name, parameters, world_interface)

    def __eq__(self, other) -> bool:
        if not isinstance(other, Upright):
            # don't attempt to compare against unrelated types
            return False
        return super().__eq__(other)

    @staticmethod
    def to_string(parameters):
        """ Creates a string """
        node_string = parameters["target_object"] + " upright?"
        return Behavior.common_string_rules(node_string, parameters)

    def update(self):
        return self.check_negated(self.world_interface.is_object_upright(self.parameters["target_object"]))

class NearRobot(Behavior):
    """
    Check if object is within reach
    """
    def __init__(self, name, parameters, world_interface, _verbose=False):
        name = NearRobot.to_string(parameters)
        super().__init__(name, parameters, world_interface)

    def __eq__(self, other) -> bool:
        if not isinstance(other, NearRobot):
            # don't attempt to compare against unrelated types
            return False
        return super().__eq__(other)

    @staticmethod
    def to_string(parameters):
        """ Creates a string """
        node_string = parameters["target_object"] + " near robot?"
        return Behavior.common_string_rules(node_string, parameters)

    def update(self):
        return self.check_negated(self.world_interface.is_near_robot(self.parameters["target_object"], distance=0.6))

class Opened(Behavior):
    """
    Check if object is open
    """
    def __init__(self, name, parameters, world_interface, _verbose=False):
        name = Opened.to_string(parameters)
        super().__init__(name, parameters, world_interface)

    def __eq__(self, other) -> bool:
        if not isinstance(other, Opened):
            # don't attempt to compare against unrelated types
            return False
        return super().__eq__(other)

    @staticmethod
    def to_string(parameters):
        """ Creates a string """
        node_string = parameters["target_object"] + " opened?"
        return Behavior.common_string_rules(node_string, parameters)

    def update(self):
        return self.check_negated(self.world_interface.is_opened(self.parameters["target_object"]))

class Unlocked(Behavior):
    """
    Check if object is unlocked
    """
    def __init__(self, name, parameters, world_interface, _verbose=False):
        name = Unlocked.to_string(parameters)
        super().__init__(name, parameters, world_interface)

    def __eq__(self, other) -> bool:
        if not isinstance(other, Unlocked):
            # don't attempt to compare against unrelated types
            return False
        return super().__eq__(other)

    @staticmethod
    def to_string(parameters):
        """ Creates a string """
        node_string = parameters["target_object"] + " unlocked?"
        return Behavior.common_string_rules(node_string, parameters)

    def update(self):
        return self.check_negated(self.world_interface.is_unlocked(self.parameters["target_object"]))

class Grasp(ActionBehavior):
    """
    Grasp an object
    """
    class GraspStates(IntEnum):
        """Define the internal states during execution."""
        INIT = 1
        WAITING_FOR_STOP = 2
        WAITING_FOR_START = 3
        RUNNING = 4

    def __init__(self, name, parameters, world_interface, verbose=False):
        name = Grasp.to_string(parameters)
        self.target_object = None
        self.grasp_position = None
        self.approach_position = None
        self.orientation = None
        self.internal_state = self.GraspStates.INIT
        self.full_grasping_program = ''
        preconditions = [Grasped('', {"not": True, "target_object": '"any object"'}, world_interface)]
        postconditions = []
        if world_interface.is_graspable(parameters["target_object"]) or parameters["target_object"] == '"any object"':
            postconditions = [Grasped('', {"target_object": parameters["target_object"]}, world_interface)]
            relation = parameters.get("relation")
            relative_object = parameters.get("relative_object")
            if relation is not None and relative_object is not None:
                postconditions += [AtPos('', {"not": True,
                                              "target_object": parameters["target_object"],
                                              "relation": relation,
                                              "relative_object": relative_object}, world_interface)]
        ActionBehavior.__init__(self, name, parameters, world_interface, preconditions, postconditions, max_ticks=500, verbose=verbose)

    @staticmethod
    def to_string(parameters):
        """ Creates a string """
        node_string = "grasp " + parameters["target_object"]
        relation = parameters.get("relation")
        relative_object = parameters.get("relative_object")
        if relation is not None and relative_object is not None:
            node_string += " from " + relation
            node_string += " " + relative_object
        node_string += "!"
        return node_string

    @staticmethod
    def check_string_match(node_string, _parameters):
        """ Check if the string description of the node matches this node """
        if "grasp " in node_string:
            return True
        return False

    def initialise(self):
        self.internal_state = self.GraspStates.INIT
        self.target_object = self.find_target_object()
        ActionBehavior.initialise(self)

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = {}
        n_marks = 4
        marks = []
        marks.append(node_descriptor.find('"'))
        for i in range(1, n_marks):
            if marks[i-1] >= 0:
                marks.append(node_descriptor.find('"', marks[i-1] + 1))
            else:
                break

        if len(marks) < 2:
            print("Error, parameter parsing failed")
            return None
        
        parameters["target_object"] = node_descriptor[marks[0]: marks[1] + 1]
        if len(marks) >= 4:
            parameters["relation"] = node_descriptor[marks[1] + 7: marks[2] - 1]
            parameters["relative_object"] = node_descriptor[marks[2]: marks[3] + 1]

        return parameters

    def find_target_object(self):
        """ Finds first target object from possible objects in world """
        if self.parameters["target_object"] == '"any object"':
            if "relation" in self.parameters and "relative_object" in self.parameters:
                for target_object in self.world_interface.movable_objects:
                    if self.world_interface.object_at(target_object, self.parameters["relation"], self.parameters["relative_object"]):
                        return target_object
            return None
        else:
            return self.parameters["target_object"]

    def check_for_success(self):
        """Check if object is grasped."""
        if self.world_interface.get_grasped_object() == self.target_object:
            self.success()

    def check_for_failure(self):
        """Fail if some other object is grasped."""
        grasped_object = self.world_interface.get_grasped_object()
        return grasped_object not in (self.target_object , None)
    
    def update(self):
        """Executes behavior """
        self.check_for_success()
        ActionBehavior.update(self)

        if self.state is pt.common.Status.RUNNING:
            if self.check_for_failure():
                return self.failure()
            if self.internal_state == self.GraspStates.INIT:
                self.world_interface.stop()
                self.internal_state = self.GraspStates.WAITING_FOR_STOP
                self.calc_grasp_position()
                if self.grasp_position is None:
                    return self.failure()
                self.calc_approach_position()

                open_gripper_program = self.world_interface.get_open_gripper_program(no_wait=True)
                approach_program = self.world_interface.move_cfree(self.approach_position, self.orientation)
                if approach_program is None:
                    return self.failure()
                positioning_program = self.world_interface.move_linear(self.grasp_position, self.orientation, self.target_object)
                if positioning_program is None:
                    return self.failure()
                gripper_program = self.world_interface.get_close_gripper_program()

                lift_program = self.world_interface.move_linear(self.approach_position, self.orientation, self.target_object)

                self.full_grasping_program = self.world_interface.finalize_program(open_gripper_program +
                                                                              approach_program +
                                                                              positioning_program +
                                                                              gripper_program +
                                                                              lift_program)
            if self.internal_state == self.GraspStates.WAITING_FOR_STOP:
                if self.world_interface.has_stopped():
                    if not self.world_interface.run_program(self.full_grasping_program):
                        return self.failure()
                    self.world_interface.set_manipulation_target(self.target_object)
                    self.internal_state = self.GraspStates.WAITING_FOR_START
            if self.internal_state == self.GraspStates.WAITING_FOR_START:
                if self.world_interface.is_running():
                    self.internal_state = self.GraspStates.RUNNING
            if self.internal_state == self.GraspStates.RUNNING:
                if self.world_interface.has_stopped():
                    self.world_interface.set_grasped_object(self.target_object)
                    self.success()

        return self.state

    def calc_grasp_position(self):
        """Gets grasp position of object"""
        self.grasp_position = self.world_interface.get_position(self.target_object)

    def calc_approach_position(self):
        """Gets approach position of object"""
        self.approach_position = self.grasp_position + np.array([0.0, 0.0, 0.05]) #TODO move numbers to world_interface
        if "cap" in self.target_object:
            self.approach_position[2] += 0.02

class Place(ActionBehavior):
    """
    Place object on position
    """
    class PlaceStates(IntEnum):
        """Define the internal states during execution."""
        INIT = 1
        WAITING_FOR_STOP = 2
        WAITING_FOR_START = 3
        RUNNING = 4

    def __init__(self, name, parameters, world_interface, verbose=False):
        self.release_position = None
        self.approach_position = None
        self.orientation = None
        self.internal_state = self.PlaceStates.INIT
        self.full_placing_program = ''
        self.target_object = parameters["target_object"]
        preconditions = []
        postconditions = []
        if world_interface.is_graspable(self.target_object):
            preconditions = [Grasped('', {"target_object": self.target_object}, world_interface)]
            postconditions = [AtPos('', {"target_object": self.target_object,
                                         "relation": parameters["relation"],
                                         "relative_object": parameters["relative_object"]},
                                    world_interface)]
        elif not "relation" in parameters and not "relative_object" in parameters:
            parameters = parameters.copy() # Make sure not to change incoming
            parameters["target_object"] = '"grasped object"'
            parameters["relation"] = "on"
            parameters["relative_object"] = '"table"'
            postconditions = [Grasped('', {"not": True, "target_object": '"any object"'}, world_interface)]
        elif self.target_object == '"grasped object"':
            postconditions = [Grasped('', {"not": True, "target_object": '"any object"'}, world_interface)]
        name = Place.to_string(parameters)
        ActionBehavior.__init__(self, name, parameters, world_interface, preconditions, postconditions, max_ticks=500, verbose=verbose)

    @staticmethod
    def to_string(parameters):
        """ Creates a string """
        node_string = "place " + parameters["target_object"]
        node_string += " " + parameters["relation"]
        node_string += " " + parameters["relative_object"]
        node_string += "!"
        return node_string

    def initialise(self):
        self.internal_state = self.PlaceStates.INIT
        ActionBehavior.initialise(self)
        if self.parameters["target_object"] == '"grasped object"':
            self.target_object = self.world_interface.get_grasped_object()
            if self.target_object is None:
                self.success()

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        parameters = {}
        n_marks = 4
        marks = [0] * n_marks
        marks[0] = node_descriptor.find('"')
        for i in range(1, n_marks):
            if marks[i-1] >= 0:
                marks[i] = node_descriptor.find('"', marks[i-1] + 1)
            else:
                print("Error, parameter parsing failed")
                return None

        parameters["target_object"] = node_descriptor[marks[0]: marks[1] + 1]
        parameters["relation"] = node_descriptor[marks[1] + 2: marks[2] - 1]
        parameters["relative_object"] = node_descriptor[marks[2]: marks[3] + 1]

        return parameters

    def check_for_success(self):
        """Check if object is at target position."""
        if self.state != pt.common.Status.SUCCESS:
            if self.world_interface.object_at(self.target_object, self.parameters["relation"], self.parameters["relative_object"]) and \
                self.world_interface.get_grasped_object() != self.target_object:
                self.success()

    def check_for_failure(self):
        """Fail if object is not grasped."""
        return self.world_interface.get_grasped_object() != self.target_object

    def update(self):
        """Executes behavior """
        self.check_for_success()
        ActionBehavior.update(self)

        if self.state is pt.common.Status.RUNNING:
            if self.check_for_failure():
                return self.failure()
            if self.internal_state == self.PlaceStates.INIT:
                self.world_interface.stop()
                self.internal_state = self.PlaceStates.WAITING_FOR_STOP
                self.calc_release_position()
                if self.release_position is None:
                    return self.failure()
                self.calc_place_approach_position()

                approach_program = self.world_interface.move_cfree(self.approach_position, self.orientation)
                if approach_program is None:
                    return self.failure()
                positioning_program = self.world_interface.move_linear(self.release_position, self.orientation,
                                                                       self.target_object)
                if positioning_program is None:
                    return self.failure()
                gripper_program = self.world_interface.get_open_gripper_program()

                lift_program = self.world_interface.move_linear(self.approach_position, self.orientation, self.target_object)

                self.full_placing_program = self.world_interface.finalize_program(approach_program +
                                                                              positioning_program +
                                                                              gripper_program +
                                                                              lift_program)
            if self.internal_state == self.PlaceStates.WAITING_FOR_STOP:
                if self.world_interface.has_stopped():
                    if not self.world_interface.run_program(self.full_placing_program):
                        return self.failure()
                    self.internal_state = self.PlaceStates.WAITING_FOR_START
            if self.internal_state == self.PlaceStates.WAITING_FOR_START:
                if self.world_interface.is_running():
                    self.internal_state = self.PlaceStates.RUNNING
            if self.internal_state == self.PlaceStates.RUNNING:
                if self.world_interface.has_stopped():
                    self.world_interface.set_grasped_object(None)
                    if self.parameters["relation"] == "in":
                        self.world_interface.set_object_position(self.target_object,
                                                                 self.release_position - np.array([0.0, 0.0, self.world_interface.CUP_HEIGHT + 0.02])) #TODO move numbers to world_interface
                    else:
                        self.world_interface.set_object_position(self.target_object,
                                                                 self.release_position - np.array([0.0, 0.0, 0.003])) #TODO move numbers to world_interface
                    self.success()

        return self.state

    def calc_release_position(self):
        """Gets release position of object"""
        if self.parameters["relation"] == "on":
            relative_object_position = self.world_interface.get_position(self.parameters["relative_object"])
            if self.parameters["relative_object"] == '"table"':
                self.release_position = relative_object_position + np.array([0.0, 0.0, self.world_interface.CUBE_SIZE / 2 + 0.003])
            else:
                self.release_position = relative_object_position + np.array([0.0, 0.0, self.world_interface.CUBE_SIZE + 0.003])#TODO move numbers to world_interface
        elif self.parameters["relation"] == "in":
            if self.parameters["relative_object"] == '"centrifuge"':
                self.release_position = [0.5, 0.0876, 0.19]
                self.orientation = np.array([0.1227878, -0.6963642, 0.6963642, 0.1227878])
            else:
                relative_object_position = self.world_interface.get_position(self.parameters["relative_object"])
                if relative_object_position is not None:
                    self.release_position = relative_object_position + np.array([0.0, 0.0, self.world_interface.CUP_HEIGHT + 0.02])#TODO move numbers to world_interface
        elif self.parameters["relation"] == "at" and isinstance(self.parameters["relative_object"], np.ndarray):
            self.release_position = self.parameters["relative_object"]

    def calc_place_approach_position(self):
        """Gets place approach position of object"""
        if self.parameters["relation"] == "in" and self.parameters["relative_object"] == '"centrifuge"':
            self.approach_position = self.release_position + np.array([0.0, -0.036, 0.1])
        else:
            self.approach_position = self.release_position + np.array([0.0, 0.0, 0.05])#TODO move numbers to world_interface

class MoveHome(ActionBehavior):
    """
    Moves arm to home position
    """
    class MoveHomeStates(IntEnum):
        """Define the internal states during execution."""
        INIT = 1
        WAITING_FOR_STOP = 2
        WAITING_FOR_START = 3
        RUNNING = 4
    def __init__(self, name, parameters, world_interface, verbose=False):
        name = MoveHome.to_string(parameters)
        self.home_position = [0.45, -0.2, 0.2]
        self.internal_state = self.MoveHomeStates.INIT
        self.full_homing_program = ''
        if "target_object" in parameters:
            postconditions = [LocationKnown('', {"not": False, "target_object": parameters.get("target_object")}, world_interface)]
        else:
            postconditions = []
        super().__init__(name, parameters, world_interface, [], postconditions, max_ticks=500, verbose=verbose)

    @staticmethod
    def to_string(_parameters):
        """ Creates a string """
        return "move home!"

    def initialise(self):
        self.internal_state = self.MoveHomeStates.INIT
        super().initialise()

    @staticmethod
    def parse_parameters(_node_descriptor):
        """ Parse behavior parameters from string """
        return {}
    
    def has_postcondition_check(self):
        """
        This behavior does not check postconditions before running
        """
        return False

    def update(self):
        """Executes behavior """
        super().update()

        if self.state is pt.common.Status.RUNNING:
            if self.internal_state == self.MoveHomeStates.INIT:
                self.world_interface.stop()
                self.internal_state = self.MoveHomeStates.WAITING_FOR_STOP
                movement_program = self.world_interface.move_cfree(self.home_position)
                if movement_program is None:
                    return self.failure()

                self.full_homing_program = self.world_interface.finalize_program(movement_program)
            if self.internal_state == self.MoveHomeStates.WAITING_FOR_STOP:
                if self.world_interface.has_stopped():
                    if not self.world_interface.run_program(self.full_homing_program):
                        return self.failure()
                    self.internal_state = self.MoveHomeStates.WAITING_FOR_START
            if self.internal_state == self.MoveHomeStates.WAITING_FOR_START:
                if self.world_interface.is_running():
                    self.internal_state = self.MoveHomeStates.RUNNING
            if self.internal_state == self.MoveHomeStates.RUNNING:
                if self.world_interface.has_stopped():
                    self.success()

        return self.state

class Flip(Grasp, Place):
    """
    Flips upside upside down
    """
    class FlipStates(IntEnum):
        """Define the internal states during execution."""
        GRASPING = 1
        PLACING = 2
    def __init__(self, name, parameters, world_interface, verbose=False): # pylint: disable=super-init-not-called
        name = Flip.to_string(parameters)
        self.target_object = None
        self.grasp_position = None
        self.release_position = None
        self.approach_position = None
        self.orientation = None
        self.flip_state = self.FlipStates.GRASPING
        self.state = None
        self.counter = 0
        self.max_ticks = 500
        self.preconditions = []
        self.postconditions = [Upright('', {"not": True, "target_object": parameters["target_object"]}, world_interface),
                               Upright('', {"not": False, "target_object": parameters["target_object"]}, world_interface)]

        Behavior.__init__(self, name, parameters, world_interface, verbose=verbose) # pylint: disable=non-parent-init-called

    @staticmethod
    def to_string(parameters):
        """ Creates a string """
        node_string = "flip " + parameters["target_object"]
        node_string += "!"
        return node_string
    
    @staticmethod
    def check_string_match(node_string, _parameters):
        """ Check if the string description of the node matches this node """
        if "flip " in node_string:
            return True
        return False

    def initialise(self):
        self.flip_state = self.FlipStates.GRASPING
        self.orientation = np.array([0.707107, -0.707107, 0, 0])
        self.parameters["relation"] = "at"
        self.parameters["relative_object"] = self.world_interface.get_position(self.parameters["target_object"])
        Grasp.initialise(self)

    def has_postcondition_check(self):
        """
        This behavior doesn't check the postcondition
        """
        return False
    
    def check_for_success(self):
        """Check if current subbehavior has succeeded."""
        if self.flip_state == self.FlipStates.GRASPING:
            Grasp.check_for_success(self)
        elif self.flip_state == self.FlipStates.PLACING:
            Place.check_for_success(self)

    def update(self):
        """Executes behavior """
        ActionBehavior.update(self)

        if self.state is pt.common.Status.RUNNING:
            if self.flip_state == self.FlipStates.GRASPING:
                self.state = Grasp.update(self)
                if self.state == pt.common.Status.SUCCESS:
                    self.flip_state = self.FlipStates.PLACING
                    self.orientation = np.array([0.0, 0.0, -0.707107, -0.707107])
                    Place.initialise(self)
            if self.flip_state == self.FlipStates.PLACING:
                self.state = Place.update(self)
            if self.state == pt.common.Status.SUCCESS:
                self.world_interface.set_object_upright(self.parameters["target_object"], not self.world_interface.is_object_upright(self.parameters["target_object"]))

        return self.state
    
    def calc_grasp_position(self):
        """Override the grasp position of object"""
        Grasp.calc_grasp_position(self)
        self.grasp_position[2] = 0.118 # The lowest we can go without hitting the table
    

class OpenCentrifuge(ActionBehavior):
    """
    Open the centrifuge
    """
    class OpenStates(IntEnum):
        """Define the internal states during execution."""
        INIT = 1
        WAITING_FOR_STOP = 2
        WAITING_FOR_START = 3
        RUNNING = 4

    def __init__(self, name, parameters, world_interface, verbose=False):
        name = OpenCentrifuge.to_string(parameters)
        
        preconditions = []
        postconditions = [Opened('', {"target_object": '"centrifuge"'}, world_interface)]
        
        ActionBehavior.__init__(self, name, parameters, world_interface, preconditions, postconditions, max_ticks=500, verbose=verbose)

    @staticmethod
    def to_string(parameters):
        """ Creates a string """
        return "open centrifuge!"
        
    @staticmethod
    def check_string_match(node_string, _parameters):
        """ Check if the string description of the node matches this node """
        if "open centrifuge" in node_string:
            return True
        return False

    def initialise(self):
        self.internal_state = self.OpenStates.INIT
        ActionBehavior.initialise(self)

    @staticmethod
    def parse_parameters(node_descriptor):
        """ Parse behavior parameters from string """
        return {}
    
    def has_postcondition_check(self):
        """
        This behavior doesn't check the postcondition
        """
        return False

    def update(self):
        """Executes behavior """
        ActionBehavior.update(self)

        if self.state is pt.common.Status.RUNNING:
            if self.internal_state == self.OpenStates.INIT:
                self.world_interface.stop()
                self.internal_state = self.OpenStates.WAITING_FOR_STOP
                open_gripper_program = self.world_interface.get_open_gripper_program(no_wait=True)
                motionsup_off = self.world_interface.motionsupervision_off()
                
                approach_program = self.world_interface.move_joint(
                    [0.5, -0.03829, 0.205], np.array([0.368725, -0.641516, 0.585942, 0.33041]), None, True)
                point1_program = self.world_interface.move_joint(
                    [0.5, -0.00071, 0.182], np.array([0.425542, -0.606928, 0.535537, 0.404664]), None, True)
                softservo_on = self.world_interface.softservo_on()
                point2_program = self.world_interface.move_joint(
                    [0.5, 0.01071, 0.204], np.array([0.342386, -0.637357, 0.612253, 0.318895]), None, True)
                point3_program = self.world_interface.move_joint(
                    [0.5, 0.03471, 0.255], np.array([0.296268, -0.677995, 0.619056, 0.263283]), None, True)
                point4_program = self.world_interface.move_joint(
                    [0.5, 0.07471, 0.303], np.array([0.235313, -0.698104, 0.645344, 0.202015]), None, True)
                point5_program = self.world_interface.move_joint(
                    [0.5, 0.12371, 0.329], np.array([0.179468, -0.701869, 0.665314, 0.180359]), None, True)
                point6_program = self.world_interface.move_joint(
                    [0.5, 0.18071, 0.329], np.array([0.191354, -0.705907, 0.677261, 0.0799752]), None, True)
                retract_program = self.world_interface.move_joint(
                    [0.5, 0.12771, 0.377], np.array([0.245167, -0.695143, 0.649404, 0.186936]), None, True)
                home_program = self.world_interface.move_cfree(
                    [0.5, -0.2, 0.35], np.array([0, 0.707107, -0.707107, 0]))

                self.full_placing_program = self.world_interface.finalize_program(open_gripper_program +
                                                                                  motionsup_off + 
                                                                                  approach_program +
                                                                                  point1_program +
                                                                                  softservo_on + 
                                                                                  point2_program + 
                                                                                  point3_program + 
                                                                                  point4_program + 
                                                                                  point5_program + 
                                                                                  point6_program + 
                                                                                  retract_program +
                                                                                  home_program)
            if self.internal_state == self.OpenStates.WAITING_FOR_STOP:
                if self.world_interface.has_stopped():
                    if not self.world_interface.run_program(self.full_placing_program):
                        return self.failure()
                    self.world_interface.set_manipulation_target('"centrifuge"')
                    self.internal_state = self.OpenStates.WAITING_FOR_START
            if self.internal_state == self.OpenStates.WAITING_FOR_START:
                if self.world_interface.is_running():
                    self.internal_state = self.OpenStates.RUNNING
            if self.internal_state == self.OpenStates.RUNNING:
                if self.world_interface.has_stopped():
                    self.world_interface.set_object_opened('"centrifuge"', True)
                    self.success()

        return self.state


def get_condition_nodes():
    """ Returns a list of all action nodes available for planning """
    return [AtPos, Grasped, LocationKnown, Upright, NearRobot, Opened, Unlocked]


def get_action_nodes():
    """ Returns a list of all action nodes available for planning """
    return [Grasp, Place, MoveHome, Flip, OpenCentrifuge]
