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

    def llm_callback(self, request, response):
        api_key = os.environ.get("API_KEY")
        response.response = requests.post(
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
                ]
            })
        )
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
    
def main(args=None):
    rclpy.init(args=args)
    llm_service = LLMService()
    rclpy.spin(llm_service)
    llm_service.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()