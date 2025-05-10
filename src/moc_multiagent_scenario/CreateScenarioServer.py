#!/usr/bin/env python3

import rospy
import actionlib
import json
import os
import requests
import time
from moc_multiagent_scenario.msg import CreateScenarioAction


class CreateScenarioServer:
    """Action server for generating interactive museum exhibit scenarios using GPT."""
    
    def __init__(self):
        """Initialize action server and start it."""
        self._action_server = actionlib.SimpleActionServer(
            'create_scenario',
            CreateScenarioAction,
            execute_cb=self.goal_cb,
            auto_start=False
        )
        self._action_server.start()
        rospy.loginfo("Create Scenario Action Server Started")

    def run(self):
        """Keep the node running until shutdown."""
        rospy.spin()

    def ask_gpt(self, token, model, content, sentence):
        """
        Send request to OpenAI GPT API with error handling and retries.
        
        Args:
            token: OpenAI API authentication token
            model: GPT model version to use
            content: System prompt content
            sentence: User input text to process
            
        Returns:
            dict: API response or None if failed
        """
        api_url = 'https://api.openai.com/v1/chat/completions'
        proxies = {
            "http": "socks5h://127.0.0.1:1080/",
            "https": "socks5h://127.0.0.1:1080/"
        }

        retries = 0
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {token.strip()}"
        }

        messages = [
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": content.replace('TARGET', sentence)}
        ]

        data = {
            "model": model,
            "temperature": 0.3,
            "messages": messages
        }

        while retries <= 5:
            rospy.loginfo(f"Sending request to {api_url}, attempt {retries + 1}...")
            try:
                response = requests.post(
                    api_url,
                    headers=headers,
                    json=data,
                    proxies=proxies,
                    timeout=60
                )

                if response.status_code == 200:
                    rospy.loginfo("Request successful.")
                    break
                elif response.status_code == 503:
                    rospy.logwarn("Service unavailable (503), retrying...")
                    retries += 1
                    time.sleep(0.25)
                else:
                    rospy.logerr(f"Unexpected response code: {response.status_code}")
                    break

            except requests.exceptions.Timeout:
                rospy.logwarn("Request timed out.")
                retries += 1
                time.sleep(0.25)

            except requests.exceptions.RequestException as e:
                rospy.logerr(f"Request error: {e}")
                break

        if retries > 5:
            rospy.logerr("Exceeded maximum number of attempts.")
            return None

        try:
            return {
                "status": f"Code: {response.status_code}, Status: {response.reason}",
                "message": response.json()
            }
        except (KeyError, json.JSONDecodeError):
            return {
                "status": f"Code: {response.status_code}, Status: {response.reason}",
                "message": {}
            }

    def send_to_gpt(self, secret_path, model, prompt, text):
        """
        Handle GPT API communication with credentials management.
        
        Args:
            secret_path: Path to file containing API token
            model: GPT model version to use
            prompt: System prompt template
            text: User input text
            
        Returns:
            str: Processed response from GPT or None if failed
        """
        with open(secret_path, 'r') as file:
            token = file.read()

        response = self.ask_gpt(token, model, prompt, text)
        
        if (response and 
            'choices' in response['message'] and 
            len(response['message']['choices']) > 0):
            text_response = response['message']['choices'][0]['message']['content']
            return ' '.join(text_response.replace('\n', '').split())
        
        rospy.logerr("ERROR: No valid response from GPT")
        return None

    def create_scenario(self, secret_path, model, exhibit_name, description_dir,
                        reduce_prompt_path, action_labels_prompt_path, scenario_dir,
                        num_symbols, tour_style, jokes_flag=True):
        """
        Main scenario generation workflow.
        
        Args:
            secret_path: Path to API credentials
            model: GPT model version
            exhibit_name: Name of exhibit to process
            description_dir: Directory with full descriptions
            reduce_prompt_path: Path to text reduction prompt template
            action_labels_prompt_path: Path to action labeling prompt template
            scenario_dir: Output directory for generated scenarios
            num_symbols: Target length for generated text
            tour_style: Tour style specification
            jokes_flag: Whether to include humor elements
            
        Returns:
            bool: True if scenario generated successfully, False otherwise
        """
        rospy.logdebug(f"Target text length: {num_symbols} symbols")
        
        # Read full exhibit description
        full_description = self.read_txt(
            os.path.join(description_dir, exhibit_name)
        )
        
        # Prepare reduction prompt
        reduce_prompt = self.read_txt(reduce_prompt_path)
        modified_prompt = self.modify_reduce_text(
            reduce_prompt,
            num_symbols,
            tour_style,
            jokes_flag
        )
        
        # Generate condensed description
        condensed_desc = self.send_to_gpt(
            secret_path,
            model,
            modified_prompt + full_description,
            ""
        )
        
        if not condensed_desc:
            return False

        # Generate action-labeled scenario
        action_prompt = self.read_txt(action_labels_prompt_path)
        scenario = self.send_to_gpt(
            secret_path,
            model,
            action_prompt + condensed_desc,
            ""
        )

        if scenario:
            # Save generated scenario
            output_dir = os.path.join(scenario_dir, exhibit_name)
            os.makedirs(output_dir, exist_ok=True)
            
            existing_files = len(os.listdir(output_dir))
            scenario_path = os.path.join(
                output_dir,
                f'scenario_{existing_files}.txt'
            )
            
            with open(scenario_path, 'w', encoding='utf-8') as f:
                f.write(scenario)
            
            rospy.loginfo(f"Scenario saved to: {scenario_path}")
            return True
        
        return False

    @staticmethod
    def read_txt(file_path):
        """
        Read contents of a text file.
        
        Args:
            file_path: Path to text file
            
        Returns:
            str: File contents
        """
        with open(file_path, 'r') as f:
            return f.read()

    @staticmethod
    def modify_reduce_text(base_prompt, num_symbols, tour_style, jokes_flag):
        """
        Adapt prompt template based on generation parameters.
        
        Args:
            base_prompt: Template prompt text
            num_symbols: Target text length
            tour_style: Audience/technical level
            jokes_flag: Humor inclusion flag
            
        Returns:
            str: Modified prompt text
        """
        # Adjust length parameters
        len_variance = 100 if (0.15 * num_symbols) > 100 else int(0.15 * num_symbols)
        modified = base_prompt.replace(
            "650 символов, можно допускать разницу, но не больше чем в 100 символов!",
            f"{num_symbols} символов, можно допускать разницу, но не больше чем в {len_variance} символов!"
        )

        # Adapt to target audience
        style_modifications = {
            "tech_adults": (
                "людям без специального технического образования",
                "людям с техническим образованием, но не углубленным в тематику экспоната"
            ),
            "schoolchild": (
                "людьми без специального технического образования",
                "школьникам старших классов"
            )
        }
        if tour_style in style_modifications:
            modified = modified.replace(*style_modifications[tour_style])

        # Handle humor inclusion
        if not jokes_flag:
            modified = modified.replace(
                "шуточные или даже саркастические вставки",
                "Объяснение сложного принципа работы на упрощенном примере. Не испольуй шутки"
            )

        return ' '.join(modified.split())

    def goal_cb(self, goal):
        """Handle incoming action server goals."""
        default_params = {
            "secret_path": rospy.get_param("~secret_path"),
            "model": rospy.get_param("~model"),
            "description_dir": rospy.get_param("~description_dir"),
            "reduce_prompt_path": rospy.get_param("~reduce_prompt_path"),
            "action_labels_prompt_path": rospy.get_param("~action_labels_prompt_path"),
            "scenario_dir": rospy.get_param("~scenario_dir"),
            "num_symbols": int(rospy.get_param("~num_symbols")),
            "tour_style": rospy.get_param("~tour_style"),
            "jokes_flag": rospy.get_param("~jokes_flag"),
        }

        # Use goal parameters or fallback to defaults
        params = {
            field: getattr(goal, field) or default_params[field]
            for field in default_params
        }

        success = self.create_scenario(
            secret_path=params["secret_path"],
            model=params["model"],
            exhibit_name=goal.exhibit_name,
            description_dir=params["description_dir"],
            reduce_prompt_path=params["reduce_prompt_path"],
            action_labels_prompt_path=params["action_labels_prompt_path"],
            scenario_dir=params["scenario_dir"],
            num_symbols=params["num_symbols"],
            tour_style=params["tour_style"],
            jokes_flag=params["jokes_flag"]
        )

        if success:
            self._action_server.set_succeeded()
        else:
            self._action_server.set_aborted()


if __name__ == '__main__':
    try:
        rospy.init_node('create_scenario_action_server')
        server = CreateScenarioServer()
        server.run()
    except rospy.ROSInterruptException:
        rospy.logerr("Server interrupted")