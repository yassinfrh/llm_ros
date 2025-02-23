import types
import rclpy
from rclpy.node import Node
from llm_ros_interfaces.srv import LLM
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from google import genai
from google.genai import types
import os
import PIL.Image

class LLMService(Node):

    def __init__(self):
        super().__init__('llm_service')
        self.srv_text = self.create_service(LLM, 'llm_text', self.llm_callback_text)
        self.srv_image = self.create_service(LLM, 'llm_image', self.llm_callback_image)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.api_key = os.environ.get("GEMINI_API_KEY")
        self.client = genai.Client(api_key=self.api_key)
        self.cv_bridge = CvBridge()
        self.image = None
        self.get_logger().info('LLM service has been started')

    def llm_callback_text(self, request, response):
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
    
    def llm_callback_image(self, request, response):
        self.get_logger().info(f"LLM image request")
        
        if not self.api_key:
            self.get_logger().error("GEMINI_API_KEY environment variable is not set")
            response.response = "GEMINI_API_KEY environment variable is not set"
            return response

        api_response = self.client.models.generate_content(
            model="gemini-2.0-flash",
            contents=[f"{request.prompt}", self.image],
            config=types.GenerateContentConfig(
                system_instruction="You are a helpful assistant and you answer in a friendly and concise manner."
            )
        )

        response.response = api_response.text

        self.get_logger().info(f"LLM response: {response.response}")
        return response
    
    def image_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.image = PIL.Image.fromarray(cv_image)
    
def main(args=None):
    rclpy.init(args=args)
    llm_service = LLMService()
    rclpy.spin(llm_service)
    llm_service.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()