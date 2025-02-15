import types
import rclpy
from rclpy.node import Node
from llm_ros_interfaces.srv import LLM

from google import genai
from google.genai import types
import os

class LLMService(Node):

    def __init__(self):
        super().__init__('llm_service')
        self.srv = self.create_service(LLM, 'llm', self.llm_callback)
        self.api_key = os.environ.get("GEMINI_API_KEY")
        self.client = genai.Client(api_key=self.api_key)
        self.get_logger().info('LLM service has been started')

    def llm_callback(self, request, response):
        self.get_logger().info(f"LLM request: {request.prompt}")
        
        if not self.api_key:
            self.get_logger().error("GEMINI_API_KEY environment variable is not set")
            response.response = "GEMINI_API_KEY environment variable is not set"
            return response

        api_response = self.client.models.generate_content(
            model="gemini-2.0-flash",
            contents=f"{request.prompt}",
            config=types.GenerateContentConfig(
                system_instruction="You are a helpful assistant and you answer in a friendly and concise manner."
            )
        )

        response.response = api_response.text

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