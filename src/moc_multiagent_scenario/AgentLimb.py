#!/usr/bin/env python3

from moc_multiagent_scenario.Agent import Agent
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from moc_android.msg import MoveLimbsToPoseAction, MoveLimbsToPoseGoal # movement realisations
from moc_android.msg import PlayPoseAnimationAction, PlayPoseAnimationGoal  # movement realisations
import yaml


class AgentLimb(Agent):
    """Agent class for limb control with advanced conflict resolution.
    
    Implements priority-based operation handling with wait-for-completion strategy
    for lower priority operations to maintain natural behavior.
    """
    
    def __init__(self, agent_name, limb_name, default_action_delay=0.0, filler_activation_time=0.0):
        """Initialize limb agent with specified parameters.
        
        Args:
            agent_name (str): Name of the agent
            limb_name (str): Name of the controlled limb
            default_action_delay (float): Default delay before executing actions
            filler_activation_time (float): Activation time for filler operations
        """
        super(AgentLimb, self).__init__(agent_name, default_action_delay, filler_activation_time)
        
        self._init_action_poses(limb_name, 0.3)
        self._init_action_animations(limb_name)
        
        self.register_operation('action_pose', self._do_action_pose, 
                               priority=2, cancel_function=self._cancel_action_pose)
        self.register_operation('action_animation', self._do_action_animation,
                               priority=3, cancel_function=self._cancel_action_animation)
        
        self.init_operation_default('action_pose', pose_name='default')
        
        joints_path = rospy.get_param('~joints_path', '')
        self.animation_durations = {}
        self._load_animation_durations(joints_path, limb_name)
        self.elapsed_time = 0
        self.animation_duration = 0

    def set_stopped_state(self, state):
        """Set agent's stopped state with proper handling of default operations.
        
        Args:
            state (bool): True to stop the agent, False to activate
        """
        if not state:
            self.is_stopped = False
            
        if not self.children:
            if state:
                if self.default_operation:
                    kwargs = {'pose_name': 'default'}
                    self.do_operation(self.default_operation, override_priority=float('inf'), **kwargs)
                self.is_stopped = True
        else:
            super().set_stopped_state(state)

    def do_operation(self, operation_name, override_priority=None, **kwargs):
        """Execute operation with priority-based conflict resolution.
        
        Args:
            operation_name (str): Name of the operation to execute
            override_priority (float, optional): Optional priority override
            **kwargs: Additional operation-specific arguments
        """
        if self.is_stopped:
            rospy.logerr(f"[{self.name}] Agent is stopped. Ignoring operation {operation_name}.")
            self.current_operation = None
            self.current_operation_priority = -1
            return

        local_priority = override_priority or self.agent_operations[operation_name]['priority']

        while True:
            # Parent priority check
            parent_priority = self.get_parent_max_priority()
            if parent_priority <= local_priority and parent_priority > 0:
                parent = self.parent
                while parent:
                    if isinstance(parent, AgentLimb):
                        if parent.get_current_priority() <= local_priority and parent.get_current_priority() > 0:
                            if parent._is_in_last_quarter_of_animation():
                                self._wait_for_parent_animation(parent)
                                return self.do_operation(operation_name, local_priority, **kwargs)
                            else:
                                break
                    parent = parent.parent

            # Children priority check
            children_priority = self.get_children_max_priority()
            if children_priority <= local_priority and children_priority > 0:
                for child in self.children:
                    if isinstance(child, AgentLimb):
                        child_priority = child.get_current_priority()
                        if child_priority <= local_priority and child_priority > 0:
                            if child._is_in_last_quarter_of_animation():
                                self._wait_for_child_animation(child)
                                return self.do_operation(operation_name, local_priority, **kwargs)
                            else:
                                break

            # Self-priority check
            current_priority = self.get_current_priority()
            if current_priority > local_priority and current_priority > 0:
                rospy.logerr(f"[{self.name}] Current operation has higher priority. Skipping '{operation_name}'")
                return

            if self._is_in_last_quarter_of_animation():
                self._wait_for_self_animation()
                return self.do_operation(operation_name, local_priority, **kwargs)
            else:
                break

        super().do_operation(operation_name, override_priority=local_priority, **kwargs)

    # region Action Pose Methods
    def _init_action_poses(self, limb, speed):
        """Initialize pose action parameters.
        
        Args:
            limb (str): Limb name to control
            speed (float): Default movement speed
        """
        self.action_pose_limb = limb
        self.action_pose_speed = speed
        self.pose_action_client = actionlib.SimpleActionClient('move_limbs_to_pose', MoveLimbsToPoseAction)

    def _do_action_pose(self, pose_name, delay=2.0):
        """Execute pose action.
        
        Args:
            pose_name (str): Name of the pose to execute
            delay (float): Timeout for server connection
        """
        if not self.pose_action_client.wait_for_server(timeout=rospy.Duration(delay)):
            rospy.logerr("Can't make pose - server connection timeout")
            return
            
        goal = MoveLimbsToPoseGoal([self.action_pose_limb], [pose_name], self.action_pose_speed)
        
        def goal_done_callback(state, result):
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"[{self.action_pose_limb}] Pose executed successfully")
                self.update_last_operation_time()
            else:
                rospy.logwarn(f"Pose execution failed with status: {state}")
            self._reset_operation_state()
            
        self.pose_action_client.send_goal(goal, done_cb=goal_done_callback)

    def _cancel_action_pose(self):
        """Cancel current pose action."""
        if self.pose_action_client.get_state() == GoalStatus.ACTIVE:
            self.pose_action_client.cancel_goal()
            self.elapsed_time = 0
            self.animation_duration = 0
    # endregion

    # region Action Animation Methods
    def _load_animation_durations(self, joints_path, limb_name):
        """Load animation durations from YAML configuration.
        
        Args:
            joints_path (str): Path to joints configuration file
            limb_name (str): Name of the limb to load animations for
        """
        try:
            with open(joints_path, 'r') as f:
                data = yaml.safe_load(f)
            
            for limb in data.get('limbs', []):
                if limb.get('name') == limb_name:
                    for anim in limb.get('animations', []):
                        self.animation_durations[anim['id']] = anim['sequence'][-1]['time']
                    break
        except Exception as e:
            rospy.logerr(f"Error loading animation durations: {str(e)}")

    def _is_in_last_quarter_of_animation(self):
        """Check if current animation is in last 25% of its duration.
        
        Returns:
            bool: True if in last quarter, False otherwise
        """
        if self.current_operation != 'action_animation' or not self.elapsed_time:
            return False
        return self.elapsed_time >= 0.75 * self.animation_duration

    def _init_action_animations(self, limb):
        """Initialize animation action parameters.
        
        Args:
            limb (str): Limb name to control
        """
        self.action_anim_limb = limb
        self.anim_action_client = actionlib.SimpleActionClient('play_pose_animation', PlayPoseAnimationAction)

    def _do_action_animation(self, number_of_times, animation_name, delay=2.0):
        """Execute animation action.
        
        Args:
            number_of_times (int): Number of repetitions
            animation_name (str): Name of the animation to play
            delay (float): Timeout for server connection
        """
        self.animation_duration = self.animation_durations.get(animation_name, 0)
        
        if not self.anim_action_client.wait_for_server(timeout=rospy.Duration(delay)):
            rospy.logerr("Animation server connection timeout")
            return
            
        goal = PlayPoseAnimationGoal(
            limb=self.action_anim_limb,
            animation=animation_name,
            number_of_times=number_of_times
        )

        def feedback_cb(feedback):
            self.elapsed_time = feedback.elapsed_time

        def done_cb(state, result):
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"[{self.name}] Animation {animation_name} completed")
                self.update_last_operation_time()
            else:
                rospy.logwarn(f"Animation failed with status: {state}")
            self._reset_operation_state()

        self.anim_action_client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)

    def _cancel_action_animation(self):
        """Cancel current animation action."""
        if self.anim_action_client.get_state() == GoalStatus.ACTIVE:
            self.anim_action_client.cancel_goal()
            self.elapsed_time = 0
    # endregion

    # region Helper Methods
    def _wait_for_parent_animation(self, parent):
        """Wait for parent animation to complete.
        
        Args:
            parent (AgentLimb): Parent agent instance
        """
        rospy.loginfo(f"[{self.name}] Waiting for parent {parent.name} animation completion...")
        remaining = parent.animation_duration - parent.elapsed_time
        while remaining > 0.1:
            rospy.sleep(0.1)
            remaining = parent.animation_duration - parent.elapsed_time
            self.update_last_operation_time()

    def _wait_for_child_animation(self, child):
        """Wait for child animation to complete.
        
        Args:
            child (AgentLimb): Child agent instance
        """
        rospy.loginfo(f"[{self.name}] Waiting for child {child.name} animation completion...")
        remaining = child.animation_duration - child.elapsed_time
        while remaining > 0.1:
            rospy.sleep(0.1)
            remaining = child.animation_duration - child.elapsed_time
            self.update_last_operation_time()

    def _wait_for_self_animation(self):
        """Wait for current animation to complete."""
        rospy.loginfo(f"[{self.name}] Waiting for current animation completion...")
        remaining = self.animation_duration - self.elapsed_time
        while remaining > 0.1:
            rospy.sleep(0.1)
            remaining = self.animation_duration - self.elapsed_time
            self.update_last_operation_time()

    def _reset_operation_state(self):
        """Reset current operation state."""
        if self.get_current_priority() > 0:
            self.current_operation = None
            self.current_operation_priority = -1
            self.is_default_operation_running = False
    # endregion