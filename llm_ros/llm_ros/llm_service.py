import rclpy
from rclpy.node import Node
from llm_ros_interfaces.srv import LLM

import requests
import json
import os

class LLMService(Node):

    def __init__(self):
        super().__init__('llm_service')
        self.srv = self.create_service(LLM, 'llm', self.llm_callback)
        self.get_logger().info('LLM service has been started')

    def llm_callback(self, request, response):
        self.get_logger().info(f"LLM request: {request.prompt}")
        api_key = os.environ.get("API_KEY")
        if not api_key:
            self.get_logger().error("API_KEY environment variable is not set")
            response.response = "API_KEY environment variable is not set"
            return response

        self.get_logger().info(f"Using API_KEY: {api_key}")

        api_response = requests.post(
            url="https://openrouter.ai/api/v1/chat/completions",
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json"
            },
            data=json.dumps({
                "model": "deepseek/deepseek-r1:free",
                "messages": [
                    {
                        "role": "system",
                        "content": "You are a helpful assistant that always responds in a friendly and concise manner."
                    },
                    {
                        "role": "user",
                        "content": request.prompt
                    }
                ],
                "top_p": 1,
                "temperature": 1,
                "frequency_penalty": 0,
                "presence_penalty": 0,
                "repetition_penalty": 1,
                "top_k": 0
            })
        )

        self.get_logger().info(f"LLM response status: {api_response.status_code}")
        self.get_logger().info(f"LLM response content: {api_response.text}")

        if api_response.status_code == 200:
            try:
                response_data = api_response.json()
                response.response = response_data.get('choices', [{}])[0].get('message', {}).get('content', '')
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to decode JSON response: {e}")
                response.response = "Failed to decode JSON response"
        else:
            response.response = f"Error: {api_response.status_code}"

        self.get_logger().info(f"LLM response: {response.response}")
        return response
    
def main(args=None):
    rclpy.init(args=args)
    llm_service = LLMService()
    rclpy.spin(llm_service)
    llm_service.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()