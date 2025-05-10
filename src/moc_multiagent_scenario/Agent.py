#! /usr/bin/env python3 
import rospy
import actionlib
import time
import random
from actionlib_msgs.msg import GoalStatus


class Agent:
    """Abstract class representing a decentralized agent in the robotic control system.
    
    Attributes:
        name (str): Unique identifier for the agent
        children (list): Child agents in hierarchy
        parent (Agent): Parent agent in hierarchy
        current_operation (str): Name of currently executing operation
        current_operation_priority (int): Priority of current operation
    """

    def __init__(self, agent_name, default_action_delay=0.0, filler_activation_time=0.0):
        """Initialize agent with basic configuration.
        
        Args:
            agent_name (str): Unique name for the agent
            default_action_delay (float): Delay before executing default operation
            filler_activation_time (float): Time before activating filler operations
        """
        self.name = agent_name
        self.current_operation = None
        self.current_operation_priority = -1
        self.agent_operations = {}
        self.parent = None
        self.children = []

        self.last_operation_time = time.time()
        self.default_action_delay = default_action_delay
        self.is_default_operation_running = False
        self.default_operation = None
        self.default_operation_start = None
        self.is_stopped = False
        self.force_default_now = False

        self.fillers = {}
        self.filler_activation_time = filler_activation_time

    def set_stopped_state(self, state):
        """Set agent's stopped state.
        
        Args:
            state (bool): True to stop the agent, False to activate
        """
        self.is_stopped = state

    def find_child_by_name(self, name):
        """Recursively find child agent by name.
        
        Args:
            name (str): Name of agent to find
            
        Returns:
            Agent or None: Found agent or None if not exists
        """
        for child in self.children:
            if child.name == name:
                return child
            
            found = child.find_child_by_name(name)
            if found is not None:
                return found
        return None

    def update_last_operation_time(self):
        """Update last operation timestamp for agent and all children."""
        self.last_operation_time = time.time()
        for child in self.children:
            child.update_last_operation_time()

    def add_child(self, agent):
        """Add single child agent."""
        self.children.append(agent)

    def add_children(self, agent_list):
        """Add multiple child agents."""
        for agent in agent_list:
            self.children.append(agent)
            agent._set_parent(self)

    def _set_parent(self, parent_agent):
        """Set parent agent (internal use)."""
        self.parent = parent_agent

    def get_current_priority(self):
        """Get priority of current operation."""
        return self.current_operation_priority if self.current_operation else -1

    def get_parent_max_priority(self):
        """Get maximum priority from all parent agents."""
        priorities = [-1]
        parent = self.parent
        while parent is not None:
            priorities.append(parent.get_current_priority())
            parent = parent.parent
        return max(priorities)

    def cancel_all_parent_operations(self):
        """Cancel operations for all parent agents."""
        parent = self.parent
        while parent is not None:
            parent.cancel_current_operation()
            parent = parent.parent

    def get_children_max_priority(self):
        """Get maximum priority from all children agents."""
        def recursive_max_priority(children):
            max_priority = -1
            for child in children:
                current = child.get_current_priority()
                max_priority = max(max_priority, current)
                max_priority = max(max_priority, recursive_max_priority(child.children))
            return max_priority
        return recursive_max_priority(self.children)

    def cancel_all_children_operations(self):
        """Cancel operations for all children agents."""
        for child in self.children:
            child.cancel_current_operation()
            child.cancel_all_children_operations()

    def do_operation(self, operation_name, override_priority=None, from_parent=False, **kwargs):
        """Main decision-making method for conflict resolution.
        
        Args:
            operation_name (str): Name of operation to execute
            override_priority (int, optional): Force priority level
            from_parent (bool): Indicates if called by parent agent
        """
        if self.is_stopped:
            rospy.logwarn(f"[{self.name}] Agent is stopped. Ignoring {operation_name}.")
            self.current_operation = None
            self.current_operation_priority = -1
            return

        local_priority = override_priority or self.agent_operations[operation_name]['priority']

        rospy.logdebug(f"[{self.name}] Trying {operation_name} with priority {local_priority}")

        if self.parent is not None and not from_parent:
            parent_priority = self.get_parent_max_priority()
            if parent_priority < local_priority and parent_priority != 0:
                rospy.logerr(f"Canceling parents! {local_priority} > {parent_priority}")
                self.cancel_all_parent_operations()
            elif parent_priority >= local_priority:
                return

        if self.get_current_priority() < local_priority:
            if self.get_children_max_priority() < local_priority:
                self.cancel_all_children_operations()

            self.cancel_current_operation()
            self.current_operation = operation_name
            self.current_operation_priority = local_priority

            if local_priority == 0:
                self.is_default_operation_running = True
                self.default_operation_start = time.time() if self.filler_activation_time > 0.0 else None
            else:
                self.is_default_operation_running = False

            self.agent_operations[operation_name]['do'](**kwargs)
        else:
            rospy.logwarn(f"[{self.name}] Priority {self.get_current_priority()} >= {local_priority}. Skip.")

    def register_operation(self, operation_name, function_to_do, priority=0, cancel_function=lambda: None):
        """Register new operation type for the agent.
        
        Args:
            operation_name (str): Unique operation identifier
            function_to_do (callable): Operation execution function
            priority (int): Base priority level
            cancel_function (callable): Operation cancellation function
        """
        if operation_name in self.agent_operations:
            raise ValueError(f"Operation {operation_name} already registered")

        self.agent_operations[operation_name] = {
            'do': function_to_do,
            'priority': priority,
            'cancel': cancel_function
        }

    def cancel_current_operation(self):
        """Cancel currently executing operation."""
        rospy.loginfo(f'[{self.name}] Canceling {self.current_operation}')
        if self.current_operation:
            self.agent_operations[self.current_operation]['cancel']()
        
        if self.is_default_operation_running:
            self.update_last_operation_time()
            self.is_default_operation_running = False
            
        self.current_operation = None
        self.current_operation_priority = -1

    def init_operation_default(self, operation_name, **operation_kwargs):
        """Initialize default idle operation."""
        if self.default_action_delay > 0:
            self.default_operation = operation_name
            self.default_operation_kwargs = operation_kwargs
            if not self.is_stopped:
                self.default_timer = rospy.Timer(rospy.Duration(2), self._check_default_action)

    def _check_default_action(self, event):
        """Internal method for default operation management."""
        if self.force_default_now and not self.is_stopped:
            self.force_default_now = False
            if not self.is_default_operation_running and not self.current_operation:
                rospy.loginfo(f"[{self.name}] Forcing default operation")
                self.do_operation(
                    self.default_operation,
                    override_priority=0,
                    **self.default_operation_kwargs
                )
            return

        if not self.is_stopped and (time.time() - self.last_operation_time) > self.default_action_delay:
            if not self.is_default_operation_running:
                rospy.loginfo(f"[{self.name}] Executing default action")
                self.do_operation(
                    self.default_operation,
                    override_priority=0,
                    **self.default_operation_kwargs
                )
            elif self.default_operation_start and self.fillers:
                elapsed = time.time() - self.default_operation_start
                if elapsed > self.filler_activation_time:
                    filler = random.choice(list(self.fillers.keys()))
                    rospy.loginfo(f"[{self.name}] Executing filler: {filler}")
                    params = {k: v for k, v in self.fillers[filler].items() if k != 'do'}
                    self.do_operation(filler, override_priority=1, **params)

    def add_filler(self, operation_name, **operation_kwargs):
        """Register filler operation for idle time.
        
        Args:
            operation_name (str): Registered operation name
            operation_kwargs: Parameters for the operation
        """
        if operation_name in self.agent_operations:
            self.fillers[operation_name] = operation_kwargs
        else:
            rospy.logwarn(f"[{self.name}] Operation {operation_name} not registered")