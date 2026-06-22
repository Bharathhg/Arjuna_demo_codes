#!/usr/bin/env python3

import http.server
import socketserver
import webbrowser
import os
import threading
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from functools import partial

# Configuration
PORT = 8000

class WebGuiNode(Node):
    def __init__(self):
        super().__init__('web_gui_launcher')
        
        # Locate the webpage folder inside the installed package share directory
        try:
            package_share = get_package_share_directory('arjuna')
            self.webpage_dir = os.path.join(package_share, 'webpage')
        except Exception as e:
            self.get_logger().error(f"Failed to find package share directory: {e}")
            # Fallback to local source directory relative to this script
            self.webpage_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../webpage'))
            
        self.get_logger().info(f"Serving webpage from directory: {self.webpage_dir}")

        # Start the web server in a separate thread
        self.server_thread = threading.Thread(target=self.start_server)
        self.server_thread.daemon = True
        self.server_thread.start()

        # Open the webpage in the default browser
        self.get_logger().info("Opening Arjuna Web Control Dashboard in default browser...")
        webbrowser.open(f"http://localhost:{PORT}/Arjuna.html")

    def start_server(self):
        # Create a request handler configured to serve the webpage directory
        handler = partial(http.server.SimpleHTTPRequestHandler, directory=self.webpage_dir)
        try:
            socketserver.TCPServer.allow_reuse_address = True
            with socketserver.TCPServer(("", PORT), handler) as httpd:
                self.get_logger().info(f"Web GUI server running at http://localhost:{PORT}/Arjuna.html")
                httpd.serve_forever()
        except OSError as e:
            self.get_logger().warn(f"Port {PORT} is busy: {e}. Assuming server is already running.")

def main(args=None):
    rclpy.init(args=args)
    node = WebGuiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
