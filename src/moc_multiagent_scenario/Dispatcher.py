#!/usr/bin/env python3
import os
import re
import rospy
import actionlib
from speak_out_loud.msg import SpeakGoal, SpeakAction, Priority, SpeakFeedback
from actionlib_msgs.msg import GoalStatus
from moc_multiagent_scenario.msg import MultiAgentScenarioExecutorAction
from moc_multiagent_scenario.Agents import AgentLimb


class Dispatcher(object):
    """The main dispatcher class for managing multi-agent scenarios."""
    
    def __init__(self):
        """Initializing the dispatcher, creating agents and configuring ROS components."""
        self.agents = {}

        # Setting up ROS components
        self.sol_cli = actionlib.SimpleActionClient('/sol', SpeakAction)
        self.sol_feednack_sub = rospy.Subscriber(
            "/sol/feedback",
            SpeakFeedback,
            self.sol_feedack_cb
        )

        # Initializing Action Server
        self.action_server = actionlib.SimpleActionServer(
            'multiagent_dispatcher',
            MultiAgentScenarioExecutorAction,
            execute_cb=self.goal_cb,
            auto_start=False
        )
        self.action_server.start()
        self.action_server.register_preempt_callback(self.cancel_cb)
        self.init_robot_agents()

    def cancel_cb(self):
        """callback to cancel current targets on interruption."""
        if self.sol_cli.get_state() == GoalStatus.ACTIVE:
            self.sol_cli.cancel_all_goals()

    def init_robot_agents(self):
        """Initialization and configuration of all agents for specific robot"""
        rospy.logwarn("Initializing agents...")
        
        # Creating and configuring agents for hands (example of creating hierarchy and class instances)
        agent_right_arm = AgentLimb('right_arm', 'right_arm', 10.0, 25.0)
        agent_right_arm.setStoppedState(True)
        agent_right_arm.addFiller('action_pose', pose_name='right')
        agent_right_arm.addFiller(
            'action_animation',
            number_of_times=1,
            animation_name='slow_splash'
        )

        agent_left_arm = AgentLimb('left_arm', 'left_arm', 10.0, 18.0)
        agent_left_arm.setStoppedState(True)
        agent_left_arm.addFiller('action_pose', pose_name='attention')
        agent_left_arm.addFiller(
            'action_animation',
            number_of_times=1,
            animation_name='scratch_myself'
        )

        agent_both_arms = AgentLimb('both_arms', 'both_arms')
        agent_both_arms.setStoppedState(True)
        agent_both_arms.addChildren([agent_right_arm, agent_left_arm])

        # Register all agents
        self.register_agents([
            agent_right_arm,
            agent_left_arm,
            agent_both_arms,
        ])

    def read_file_content(self, file_path):
        """Reading the contents of a script file."""
        if os.path.isfile(file_path):
            with open(file_path, 'r') as file:
                return file.read()
        else:
            rospy.logerr(f"File not found: {file_path}")
            return None

    def get_latest_scenario_file(self, directory):
        """Search for the latest script file in the specified directory."""
        if not os.path.exists(directory):
            rospy.logerr(f"Directory does not exist: {directory}")
            return None

        if not os.path.isdir(directory):
            rospy.logerr(f"The specified path is not a directory: {directory}")
            return None

        files = [
            f for f in os.listdir(directory)
            if re.match(r'scenario_\d+\.txt', f)
        ]

        if not files:
            rospy.logerr(f"Script files not found in directory: {directory}")
            return None

        latest_file = max(
            files,
            key=lambda f: int(re.search(r'\d+', f).group())
        )

        return os.path.join(directory, latest_file)

    def speak(self, text):
        """Producing text using a speech synthesis system."""
        rospy.logwarn(text)
        if not self.sol_cli.wait_for_server(timeout=rospy.Duration(4.0)):
            rospy.logerr("Speech synthesis error - server timeout exceeded")
            return
        
        rospy.logerr('Speech synthesizer activated')
        goal = SpeakGoal()
        goal.sender_node = rospy.get_name()
        goal.priority = Priority.TEXT
        goal.text = text
        goal.use_ssml = True
        self.sol_cli.send_goal(goal)

    def register_agent(self, agent):
        """Registration of one agent in the system."""
        self.agents[agent.name] = agent

    def register_agents(self, agent_list):
        """Registering a list of agents in the system."""
        for agent in agent_list:
            self.register_agent(agent)

    def parse_scenario(self, full_scenario_text):
        """Parsing and executing the full script."""
        self.speak(full_scenario_text)

    def sol_feedack_cb(self, feedback):
        """Processing feedback from a speech synthesis system."""
        mark_raw = feedback.feedback.mark
        rospy.logerr(f"Label received: {mark_raw}")
        parsed_mark = self.parse_mark(mark_raw)

        if parsed_mark[0] is None:
            rospy.logerr("Error parsing label")
            return
            
        # Handling different types of labels
        # your agent classes must implement these methods (see AgentLimb)
        if parsed_mark[0] == "pose":
            self.agents[parsed_mark[1][0]].doOperation(
                'action_pose',
                pose_name=parsed_mark[1][1]
            )
        elif parsed_mark[0] == "anim":
            self.agents[parsed_mark[1][0]].doOperation(
                'action_animation',
                number_of_times=int(parsed_mark[1][2]),
                animation_name=parsed_mark[1][1]
            )
        else:
            rospy.logerr("Unknown action type")

    def parse_mark(self, mark):
        """Parsing a label from the <type:parameter_1;parameter_n> format into structured data."""
        mark = mark.replace("<", "").replace(">", "")
        parse = mark.split(":")
        if parse[0] == "exh":
            return (parse[0], parse[1])
        return (parse[0], parse[1].split(";") if len(parse) > 1 else None)

    def goal_cb(self, goal):
        """Processing incoming targets to execute the scenario."""
        self.start_all_agents()
        # here you can specify a flag for some agents so that when receiving a ueli they immediately switch to the default behavior
        self.agents['agent_left_arm'].force_default_now = True
        self.agents['agent_right_arm'].force_default_now = True

        # Processing a target with a script file
        if str(goal.text).strip()[-1] == '/':
            rospy.logerr("New target with script file received")
            base_scenario_dir = rospy.get_param('~scenario_dir', '')
            full_scenario_path = os.path.join(base_scenario_dir, goal.text)
            latest_file_path = self.get_latest_scenario_file(full_scenario_path)
            
            if latest_file_path is not None:
                file_content = self.read_file_content(latest_file_path)
                if file_content is not None:
                    goal.text = file_content
                else:
                    self.action_server.set_aborted()
                    self.stop_all_agents()
                    return
            else:
                self.action_server.set_aborted()
                self.stop_all_agents()
                return
        else:
            rospy.logerr("New text target received")

        self.speak(goal.text)

        # Monitoring Script Execution
        while not rospy.is_shutdown():
            if self.sol_cli.get_state() == GoalStatus.SUCCEEDED:
                rospy.logerr("Speech synthesis complete")
                all_agents_ready = all(
                    (agent.getCurrentPriority() <= 1) or 
                    (agent.getCurrentPriority() > 1 and agent.is_default_operation_running)
                    for agent in self.agents.values()
                )
                
                if all_agents_ready:
                    rospy.logerr("All agents have completed their work.")
                    self.stop_all_agents()
                    self.action_server.set_succeeded()
                    return

                rospy.logerr("Not all agents are ready (priority > 0). Waiting...")
                for agent in self.agents.values():
                    if agent.getCurrentPriority() >= 1:
                        print(f"{agent.name}: {agent.getCurrentPriority()}")

            elif self.action_server.is_preempt_requested():
                rospy.logerr("Goal interrupted")
                self.cancel()
                self.stop_all_agents()
                self.action_server.set_preempted()
                return

            rospy.sleep(0.5)

        self.action_server.set_aborted()

    def cancel(self, exceptions=[]):
        """Cancel execution for all agents except exceptions."""
        self.cancel_goals_for_every_client(exceptions)

    def cancel_goals_for_every_client(self, exceptions=[]):
        """Cancel targets for all action clients."""
        if self.sol_cli.get_state() == GoalStatus.ACTIVE:
            self.sol_cli.cancel_goal()
        for agent in self.agents.values():
            if agent.name not in exceptions:
                agent.cancelCurrentOperation()

    def stop_all_agents(self, exceptions=[]):
        """Stop all agents except exceptions."""
        for agent in self.agents.values():
            if agent.name not in exceptions:
                agent.setStoppedState(True)

    def start_all_agents(self):
        """Activate all agents to execute the scenario."""
        for agent in self.agents.values():
            agent.updateLastOperationTime()
            agent.is_default_operation_running = False
            agent.setStoppedState(False)

    def run(self):
        """The main cycle of the dispatcher's work."""
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('multiagent_dispatcher')
        ma_d = Dispatcher()
        ma_d.run()
    except rospy.exceptions.ROSInterruptException:
        pass