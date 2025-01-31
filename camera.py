### This should be updated for the ZED2I camera ###

import pyrealsense2 as rs
import numpy as np
# import open3d as o3d

def initialize_camera():
    try:
        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()
        
        # Get device list
        ctx = rs.context()
        devices = ctx.query_devices()
        print(f"Found {len(list(devices))} connected RealSense devices")
        
        if len(list(devices)) == 0:
            raise RuntimeError("No RealSense devices found!")
        
        # Get first device & name
        first_dev = devices[0]
        print(f"Using device: {first_dev.get_info(rs.camera_info.name)}")
        
        # Enable both depth and color streams
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        print("Starting pipeline...")
        # Start streaming
        pipeline_profile = pipeline.start(config)
        print("Pipeline started successfully")
        
        return pipeline
    except Exception as e:
        print(f"Error initializing camera: {str(e)}")
        raise

def get_aligned_frames(pipeline):
    try:
        # Wait for a coherent pair of frames
        print("Waiting for frames...")
        frames = pipeline.wait_for_frames(timeout_ms=5000)  # 5 second timeout
        print("Frames received")
        
        # Align depth frame to color frame
        align = rs.align(rs.stream.color)
        aligned_frames = align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            raise RuntimeError("Could not get both depth and color frames")
            
        return depth_frame, color_frame
    except Exception as e:
        print(f"Error getting frames: {str(e)}")
        raise

def create_point_cloud(depth_frame, color_frame):
    try:
        # Create point cloud from depth image
        pc = rs.pointcloud()
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        
        # Convert to numpy array
        vertices = np.asanyarray(points.get_vertices())
        texcoords = np.asanyarray(points.get_texture_coordinates())
        
        # Get color image
        color_image = np.asanyarray(color_frame.get_data())
        
        print(f"Generated point cloud with {len(vertices)} points")
        return vertices, color_image, texcoords
    except Exception as e:
        print(f"Error creating point cloud: {str(e)}")
        raise

def main():
    pipeline = None
    try:
        # Initialize camera
        pipeline = initialize_camera()
        
        # Warm-up camera
        print("Warming up camera...")
        for i in range(30):
            pipeline.wait_for_frames()
            if i % 10 == 0:
                print(f"Warmup frame {i}/30")
        
        print("Capturing point cloud...")
        # Get frames
        depth_frame, color_frame = get_aligned_frames(pipeline)
        
        # Create point cloud
        vertices, color_image, texcoords = create_point_cloud(depth_frame, color_frame)
        
        print("Point cloud captured successfully!")
        print(f"Point cloud contains {len(vertices)} points")
        
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        if pipeline:
            print("Stopping pipeline...")
            pipeline.stop()
            print("Pipeline stopped")

if __name__ == "__main__":
    main()