#!/usr/bin/env python3

import os
import base64
import io
import yaml
import json
from pathlib import Path
from datetime import datetime
from openai import OpenAI

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


class SimpleVLMNode(Node):
    def __init__(self):
        super().__init__('simple_vlm_node')
        
        # Declare config file parameter first
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value
        
        # Load configuration
        config = self.load_config(config_file)
        
        # Initialize OpenAI client for DashScope
        self.client = OpenAI(
            api_key=config['api_key'],
            base_url=config['base_url'],
        )
        
        # CV Bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # Declare parameters with defaults from config
        self.declare_parameter('image_topic', config.get('image_topic', '/camera/camera/color/image_raw'))
        self.declare_parameter('model_name', config.get('model_name', 'qwen3-vl-plus'))
        self.declare_parameter('prompt', config.get('prompt', '图中描绘的是什么景象?'))
        self.declare_parameter('sequential_mode', config.get('sequential_mode', False))
        self.declare_parameter('process_interval', config.get('process_interval', 2.0))
        self.declare_parameter('save_results', config.get('save_results', False))
        self.declare_parameter('save_directory', config.get('save_directory', '~/vlm_results'))
        
        # Get parameters (ROS params override config file)
        image_topic = self.get_parameter('image_topic').value
        self.model_name = self.get_parameter('model_name').value
        self.prompt = self.get_parameter('prompt').value
        self.sequential_mode = self.get_parameter('sequential_mode').value
        self.process_interval = self.get_parameter('process_interval').value
        self.save_results = self.get_parameter('save_results').value
        self.save_directory = os.path.expanduser(self.get_parameter('save_directory').value)
        
        # Create save directory if saving is enabled
        if self.save_results:
            os.makedirs(self.save_directory, exist_ok=True)
            self.get_logger().info(f'Saving results to: {self.save_directory}')
        
        # Subscribe to image topic
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        # Publisher for VLM output
        self.result_pub = self.create_publisher(String, 'vlm_result', 10)
        
        # Store latest image
        self.latest_image = None
        self.processing = False
        
        # Setup processing mode
        if self.sequential_mode:
            # Sequential mode: process continuously, one after another
            # Use a fast timer to check if we can process (when not busy)
            self.timer = self.create_timer(0.01, self.process_image)  # Check every 10ms
            self.get_logger().info(f'SimpleVLM Node started in SEQUENTIAL mode')
            self.get_logger().info('Processing images continuously (as fast as possible)')
        else:
            # Timer mode: process at regular intervals
            self.timer = self.create_timer(self.process_interval, self.process_image)
            self.get_logger().info(f'SimpleVLM Node started in TIMER mode')
            self.get_logger().info(f'Processing interval: {self.process_interval}s')
        
        self.get_logger().info(f'Subscribing to {image_topic}')
        self.get_logger().info(f'Using model: {self.model_name}')
    
    def load_config(self, config_file):
        """Load configuration from config file or use defaults"""
        config = {
            'api_key': None,
            'base_url': "https://dashscope.aliyuncs.com/compatible-mode/v1",
            'image_topic': '/camera/camera/color/image_raw',
            'model_name': 'qwen3-vl-plus',
            'prompt': '图中描绘的是什么景象?',
            'sequential_mode': False,
            'process_interval': 2.0,
            'save_results': False,
            'save_directory': '~/vlm_results'
        }
        
        # Try to load from config file
        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    file_config = yaml.safe_load(f)
                    if file_config:
                        config.update(file_config)
                        self.get_logger().info(f'Loaded config from {config_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load config file: {e}')
        
        # Fallback to environment variable for API key if not in config
        if not config['api_key']:
            config['api_key'] = os.getenv("DASHSCOPE_API_KEY")
            if config['api_key']:
                self.get_logger().info('Using API key from DASHSCOPE_API_KEY environment variable')
            else:
                self.get_logger().error('No API key found! Please set DASHSCOPE_API_KEY or provide config_file')
                raise ValueError('API key not configured')
        
        return config
    
    def image_callback(self, msg):
        """Store the latest image"""
        self.latest_image = msg
    
    def image_to_base64(self, cv_image):
        """Convert OpenCV image to base64 string"""
        # Encode image as JPEG
        _, buffer = cv2.imencode('.jpg', cv_image)
        # Convert to base64
        image_base64 = base64.b64encode(buffer).decode('utf-8')
        return f"data:image/jpeg;base64,{image_base64}"
    
    def save_result_to_disk(self, cv_image, result):
        """Save image and VLM result to disk for visualization"""
        try:
            # Generate timestamp-based filename
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]  # milliseconds
            
            # Save image
            image_filename = f"image_{timestamp}.jpg"
            image_path = os.path.join(self.save_directory, image_filename)
            cv2.imwrite(image_path, cv_image)
            
            # Save result as JSON with metadata
            result_filename = f"result_{timestamp}.json"
            result_path = os.path.join(self.save_directory, result_filename)
            
            result_data = {
                'timestamp': timestamp,
                'image_file': image_filename,
                'prompt': self.prompt,
                'model': self.model_name,
                'result': result
            }
            
            with open(result_path, 'w', encoding='utf-8') as f:
                json.dump(result_data, f, ensure_ascii=False, indent=2)
            
            self.get_logger().info(f'Saved to {image_filename} and {result_filename}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save results: {str(e)}')
    
    def process_image(self):
        """Process the latest image with VLM"""
        if self.latest_image is None:
            if not self.sequential_mode:
                self.get_logger().warn('No image received yet', throttle_duration_sec=5.0)
            return
        
        if self.processing:
            if not self.sequential_mode:
                self.get_logger().warn('Still processing previous image, skipping...', throttle_duration_sec=5.0)
            return
        
        try:
            self.processing = True
            
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            
            # Convert to base64
            image_base64 = self.image_to_base64(cv_image)
            
            self.get_logger().info('Sending image to VLM...')
            
            # Call VLM API
            completion = self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": image_base64
                                },
                            },
                            {"type": "text", "text": self.prompt},
                        ],
                    },
                ],
            )
            
            # Get result
            result = completion.choices[0].message.content
            
            # Log and publish result
            self.get_logger().info(f'VLM Result: {result}')
            
            msg = String()
            msg.data = result
            self.result_pub.publish(msg)
            
            # Save results if enabled
            if self.save_results:
                self.save_result_to_disk(cv_image, result)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
        
        finally:
            self.processing = False


def main(args=None):
    rclpy.init(args=args)
    node = SimpleVLMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
