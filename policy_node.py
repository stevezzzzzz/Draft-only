#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ai4r_interfaces.msg import EscAndSteering
#from ai4r_interfaces.msg import IntArray
from ai4r_interfaces.msg import ImagePointsArray
from std_msgs.msg import String, UInt16, Float32
from rclpy.parameter import Parameter


# Import any python modules or libraries here
#from ai4r_pkg.module_to_import import function


FSM_STATE_NOT_PUBLISHING_ACTIONS   = 1
FSM_STATE_PUBLISHING_ZERO_ACTIONS  = 2
FSM_STATE_PUBLISHING_POLICY_ACTION = 3

LIST_OF_FSM_STATES = [FSM_STATE_NOT_PUBLISHING_ACTIONS, FSM_STATE_PUBLISHING_ZERO_ACTIONS, FSM_STATE_PUBLISHING_POLICY_ACTION]

TIME_WITHOUT_CV_THRESHOLD_IN_SECONDS = 2


# Create a class which inherits from the rclpy Node class (hence a superset of “rclpy.node.Node”).
class PolicyNode(Node):
    
    # CONSTRUCTOR FUNCTION
    def __init__(self):
        # Initialise the node object (passing the node name as an argument)
        super().__init__('policy_node')

        # Log the namespace
        self.get_logger().info("[POLICY NODE] starting __init__ with node namespace = " + self.get_namespace())

        # Initialise the FSM state
        self.fsm_state = FSM_STATE_PUBLISHING_ZERO_ACTIONS

        # Initialise a counter for timing a lack of CV data
        self.counts_of_timer_without_cv_data_ = 0

        # Declare the parameters of this node
        self.declare_parameters(
            namespace='',
            parameters=[
                ("timer_period", rclpy.Parameter.Type.DOUBLE),
                ("u_optical_centre", rclpy.Parameter.Type.DOUBLE),
                ("p_gain", rclpy.Parameter.Type.DOUBLE),
            ])
        self.timer_period = self.get_parameter_or("timer_period",Parameter("default_timer_period",Parameter.Type.DOUBLE,0.5)).value
        self.u_optical_centre = self.get_parameter_or("u_optical_centre",Parameter("default_u_optical_centre",Parameter.Type.DOUBLE,640.0)).value
        self.p_gain = self.get_parameter_or("p_gain",Parameter("default_p_gain",Parameter.Type.DOUBLE,0.1)).value

        self.baseline_esc_action = 0.0

        # Create ROS2 publishers
        # > For publishing the FSM state
        self.fsm_state_publisher_ = self.create_publisher(String, 'policy_fsm_state', 10)
        # > For publishing the policy actions
        self.policy_action_publisher_ = self.create_publisher(EscAndSteering, 'esc_and_steering_set_point_percent', 10)

        # Create ROS2 subscribers
        # > For subscribing to the CV line detection data
        #self.cv_subscription = self.create_subscription(IntArray, 'Image_Point', self.cv_line_dectection_callback, 10)
        self.cv_subscription = self.create_subscription(ImagePointsArray, 'image_points', self.cv_line_dectection_callback, 10)
        # > For subscribing to requests to transition the state
        self.fsm_transition_request_subscription = self.create_subscription(UInt16, 'policy_fsm_transition_request', self.fsm_transition_request_callback, 10)
        # > For subscribing to requests for esc setpoint
        self.esc_setpoint_request_subscription = self.create_subscription(Float32, 'policy_esc_setpoint_request', self.esc_setpoint_request_callback, 10)
        # > Prevent unused variable warning
        self.cv_subscription
        self.fsm_transition_request_subscription
        self.esc_setpoint_request_subscription

        # Create a timer that is used for continually publishing the FSM state
        # > First argument is the duration between 2 callbacks (in seconds).
        # > Second argument is the callback function.
        self.create_timer(float(self.timer_period), self.timer_callback)



    # CALLBACK FUNCTION: for the CV line detection subscription 
    def cv_line_dectection_callback(self, msg):
        # Log the data received for debugging purposes:
        #self.get_logger().info("[POLICY NODE] Line detection points: \"%s\"" % msg.points)

        # Set the counter to zero
        self.counts_of_timer_without_cv_data_ = 0

        # Return if in the "not publishing actions" state
        if (self.fsm_state == FSM_STATE_NOT_PUBLISHING_ACTIONS):
            return

        # Default the actions to zero
        esc_action = 0.0
        steering_action = 0.0

        # Run the policy, if in the "publishing policy" state
        if (self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION):

            # List of image coordinates
            list_of_points = msg.points

            # =======================================
            # START OF: INSERT POLICY CODE BELOW HERE
            # =======================================

            # OBSERVATIONS:
            # > The "list_of_points" variable is:
            #   - A list with length equal to the number of points detected.
            #   - Each element of the list has properties ".u" and ".v", which
            #     give the pixel coordinates of the respective point.
            #   - IMPORTANT: the (u,v) coordinates are relative to the top left
            #     corner of the image, i.e., they are NOT relative to the optical
            #     center of camera.

            # ACTIONS:
            # > The "esc_action" is:
            #   - The action for driving the main motor (esc := electronic speed contrller).
            #   - In units of signed percent, with valid range [-100,100]
            # > The "steering_action" is:
            #   - The action for changing the position of the steering servo.
            #   - In units of signed percent, with valid range [-100,100]

            # ACRONYMS:
            # "esc" := Electronic Speed Controller
            #   - This device on the Traxxas car does NOT control the speed.
            #   - The "esc" device set the voltage to the motor within the
            #     range [-(max voltage),+(max voltage)]
            
            # PERFORM POLICY COMPUTATIONS
            # > Set the ESC to a fixed value
            esc_action = self.baseline_esc_action
            # > Compute the average x of the line detection points
            steering_action = 0.0
            if (len(list_of_points) > 0):
                # Extract the first elements 
                u_coords = [tup.u for tup in list_of_points]
                # Calculate the average 
                u_average = sum(u_coords) / len(u_coords)
                # Set the steering action. Possibly needs to be negative depending on steering convention.
                steering_action = self.p_gain*(u_average - self.u_optical_centre) 
            
            # =====================================
            # END OF: INSERT POLICY CODE ABOVE HERE
            # =====================================


        # Prepare the message to send
        msg = EscAndSteering()
        msg.esc_percent = esc_action
        msg.steering_percent = steering_action

        # Publish the message
        self.policy_action_publisher_.publish(msg)

        # Log the string published for debugging purposes:
        #self.get_logger().info("[POLICY NODE] Published esc action = " + str(esc_action) + ", steering action = " + str(steering_action))



    # CALLBACK FUNCTION: for the FSM state transition request
    def fsm_transition_request_callback(self, msg):
        # Extract the requests FSM state from the message
        requested_state = msg.data

        # Check that the requested state is valid
        if requested_state not in LIST_OF_FSM_STATES:
            # Log that this occurred
            self.get_logger().info("[POLICY NODE] Received request to transition to an invalid state, requested_state = " + str(requested_state))

        # Transition the state
        self.fsm_state = requested_state

        # Log the data received for debugging purposes:
        self.get_logger().info("[POLICY NODE] Received request to transition to FSM state: " + str(msg.data))

    # CALLBACK FUNCTION: for the ESC setpoint request
    def esc_setpoint_request_callback(self, msg):
        # Extract the ESC request from the message
        requested_setpoint = msg.data

        # Clip the setpoint
        if (requested_setpoint < -100.0):
            requested_setpoint = -100.0
        if (requested_setpoint >  100.0):
            requested_setpoint =  100.0

        # Update the setpoint
        self.baseline_esc_action = requested_setpoint

        # Log the data received for debugging purposes:
        self.get_logger().info("[POLICY NODE] Received request update ESC setpoint to " + str(msg.data) + " %")




    # CALLBACK FUNCTION: for the timer
    def timer_callback(self):
        # Compute the time without CV data, in seconds
        time_since_last_cv_data = self.counts_of_timer_without_cv_data_ * self.timer_period

        # Transition to zero action state if "too" long since last CV data
        if (time_since_last_cv_data > TIME_WITHOUT_CV_THRESHOLD_IN_SECONDS) and (self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION):
            self.fsm_state = FSM_STATE_PUBLISHING_ZERO_ACTIONS

        # Convert the FSM state to a string
        state_as_string = "Unknown state"

        if (self.fsm_state == FSM_STATE_NOT_PUBLISHING_ACTIONS):
            state_as_string = "Not publishing any actions"
        elif (self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION):
            state_as_string = "Publishing policy actions"
        elif (self.fsm_state == FSM_STATE_PUBLISHING_ZERO_ACTIONS):
            state_as_string = "Publishing zero actions"

        # Append to the string if "too" long since last CV data
        if (time_since_last_cv_data > TIME_WITHOUT_CV_THRESHOLD_IN_SECONDS):
            state_as_string = state_as_string + " (" + str(time_since_last_cv_data) + " seconds since last CV line detection data)"

        # Publish actions if "too" long since last CV data
        if (time_since_last_cv_data > TIME_WITHOUT_CV_THRESHOLD_IN_SECONDS) and ((self.fsm_state == FSM_STATE_PUBLISHING_POLICY_ACTION) or (self.fsm_state == FSM_STATE_PUBLISHING_ZERO_ACTIONS)):
            # Prepare the message to send
            msg = EscAndSteering()
            msg.esc_percent = 0.0
            msg.steering_percent = 0.0
            # Publish the message
            self.policy_action_publisher_.publish(msg)


        # Prepare the message to send
        msg = String()
        msg.data = state_as_string

        # Publish the message
        self.fsm_state_publisher_.publish(msg)

        # Log the string published for debugging purposes:
        #self.get_logger().info("[POLICY NODE] Published FSM state = " + state_as_string)

        # Increment the counter
        self.counts_of_timer_without_cv_data_ = self.counts_of_timer_without_cv_data_ + 1



def main(args=None):
    # Initialise ROS2 for this script
    rclpy.init(args=args)
    # Start as instance of the PolicyNode class
    node = PolicyNode()
    # Enter a ROS2 spin
    rclpy.spin(node)
    # Shutdown the nodes in this script
    rclpy.shutdown()

if __name__ == '__main__':
    main()
