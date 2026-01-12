import os
import subprocess
import signal
import yaml
import tempfile
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from custom_interfaces.srv import SetDepthMode

class ZedManager(Node):
    def __init__(self):
        super().__init__('zed_manager_node')
        
        self.camera_model = 'zed2i'
        self.zed_pkg = 'zed_wrapper'
        
        self.share_dir = get_package_share_directory(self.zed_pkg)
        self.template_config = os.path.join(self.share_dir, 'config', 'common_stereo.yaml')
        self.generated_config = os.path.join(tempfile.gettempdir(), 'generated_common_stereo.yaml')

        self.process = None
        
        self.srv = self.create_service(SetDepthMode, 'set_zed_status', self.handle_request)
        self.get_logger().info('ZED Manager Ready.')

    def handle_request(self, request, response):
        new_mode = request.mode.upper()

        # 1. STOP COMMAND (Wrapper only)
        if new_mode == 'STOP':
            self.stop_zed()
            response.success = True
            response.message = "ZED Wrapper stopped. Manager is still running."
            return response

        # 2. STANDARD MODE SWITCHING
        valid_modes = ['NONE', 'PERFORMANCE', 'QUALITY', 'ULTRA', 'NEURAL', 'NEURAL_LIGHT']

        if new_mode not in valid_modes:
            response.success = False
            response.message = f"Invalid Mode. Options: {valid_modes} or STOP"
            return response

        # Stop existing process before starting new one
        self.stop_zed()

        try:
            self.generate_config(new_mode)
        except Exception as e:
            response.success = False
            response.message = f"Failed to write config: {str(e)}"
            return response

        self.start_zed()

        response.success = True
        response.message = f"ZED started with depth mode: {new_mode}"
        return response

    def stop_zed(self):
        if self.process:
            self.get_logger().info("Stopping ZED Wrapper...")
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.process = None
        else:
            self.get_logger().info("ZED Wrapper is not running.")

    def generate_config(self, mode):
            self.get_logger().info(f"Generating config in {self.generated_config} with mode: {mode}")
            
            # 1. Read original file as text lines
            try:
                with open(self.template_config, 'r') as f:
                    lines = f.readlines()
            except IOError as e:
                self.get_logger().error(f"Template config file could not be read: {e}")
                return

            new_lines = []
            found = False

            # 2. Iterate and replace only the specific line
            for line in lines:
                # Check if line starts with 'depth_mode:' (ignoring whitespace)
                if line.strip().startswith('depth_mode:'):
                    # Calculate original indentation
                    indentation = line[:line.find('depth_mode:')]
                    
                    # Preserve existing comments if any
                    comment = ""
                    if '#' in line:
                        comment = line[line.find('#'):].strip()
                    
                    # Write new line: Indentation + Key + Value + Comment
                    new_line = f"{indentation}depth_mode: '{mode}' {comment}\n"
                    new_lines.append(new_line)
                    
                    self.get_logger().info(f"Changed depth_mode to '{mode}'")
                    found = True
                else:
                    new_lines.append(line)

            if not found:
                self.get_logger().error("Could not find 'depth_mode' parameter in the template!")

            # 3. Write exact text back to new file
            with open(self.generated_config, 'w') as f:
                f.writelines(new_lines)
                
            if os.path.exists(self.generated_config):
                self.get_logger().info(f"File successfully written to: {self.generated_config}")
            else:
                self.get_logger().error("File write failed!")
                
    def start_zed(self):
            self.get_logger().info(f"Launching ZED with override config: {self.generated_config}")
            cmd = [
                'ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py',
                f'camera_model:={self.camera_model}',
                # This argument appends your file to the end of the param list
                f'ros_params_override_path:={self.generated_config}' 
            ]
            self.get_logger().info(f"Executing command: {' '.join(cmd)}")
            self.process = subprocess.Popen(cmd, start_new_session=True)

    def destroy_node(self):
        self.stop_zed()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZedManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()