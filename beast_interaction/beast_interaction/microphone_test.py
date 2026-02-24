#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
from datetime import datetime

class MicrophoneTest(Node):
    def __init__(self):
        super().__init__('microphone_test')
        
        self.get_logger().info('Microphone Test Node Started')
        self.get_logger().info('Recording 3 seconds from camera microphone...')
        
        # Test recording using arecord
        self.test_microphone()
    
    def test_microphone(self):
        """Record a short audio clip using arecord"""
        filename = f'/tmp/mic_test_{datetime.now().strftime("%Y%m%d_%H%M%S")}.wav'
        
        # Use arecord to record from camera mic (hw:0,0) at 48kHz
        cmd = [
            'arecord',
            '-D', 'hw:0,0',      # Camera microphone
            '-f', 'S16_LE',      # 16-bit PCM
            '-c', '1',           # Mono
            '-r', '48000',       # 48kHz
            '-d', '3',           # 3 seconds
            filename
        ]
        
        try:
            subprocess.run(cmd, check=True)
            self.get_logger().info(f'✓ Recording saved to: {filename}')
            self.get_logger().info(f'✓ Play it back with: aplay {filename}')
            self.get_logger().info('✓ Microphone test successful!')
        except Exception as e:
            self.get_logger().error(f'Recording failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MicrophoneTest()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()