import os
import re
import pyrealsense2 as rs
import cv2
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageTk
import time

def get_camera_labels(serial_numbers):
    """
    Maps serial numbers to their corresponding camera labels.
    """
    camera_labels = {
        '021222072398': 'Robot camera SN=021222072398',
        '021222070941': 'Participant camera SN=021222070941'
    }
    
    labeled_cameras = {}
    
    for sn in serial_numbers:
        if sn in camera_labels:
            labeled_cameras[sn] = camera_labels[sn]
        else:
            labeled_cameras[sn] = 'Unknown camera'
    
    return labeled_cameras

def capture_frames(serial_number, frame_dict, lock, stop_event):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial_number)
    
    retry_attempts = 3  # Retry up to 3 times

    while retry_attempts > 0:
        try:
            print(f"[INFO] Attempting to start video stream for {serial_number} (Retries left: {retry_attempts})")

            # Reset the camera before starting
            ctx = rs.context()
            devices = ctx.query_devices()
            for device in devices:
                if device.get_info(rs.camera_info.serial_number) == serial_number:
                    print(f"[DEBUG] Resetting RealSense device: {serial_number}")
                    device.hardware_reset()
                    time.sleep(5)  # Give it time to restart

            pipeline.start(config)
            print(f"[INFO] Started video stream for Serial Number: {serial_number}")

            while not stop_event.is_set():
                frames = pipeline.wait_for_frames(5000)  # 5-second timeout
                color_frame = frames.get_color_frame()

                if color_frame:
                    color_image = np.asanyarray(color_frame.get_data())
                    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                    with lock:
                        frame_dict[serial_number] = color_image
                else:
                    print(f"[WARNING] No color frame received from {serial_number}")

        except RuntimeError as e:
            print(f"[ERROR] Failed to capture frames for Serial Number {serial_number}: {e}")
            retry_attempts -= 1  # Reduce retries on failure
            print(f"[DEBUG] Retrying camera {serial_number} in 3 seconds...")
            time.sleep(3)
            continue  # Try again

        finally:
            if "pipeline.start" in locals():
                print(f"[DEBUG] Stopping pipeline for Serial Number: {serial_number}")
                pipeline.stop()

        if retry_attempts == 0:
            print(f"[CRITICAL] Camera {serial_number} failed after multiple attempts. Skipping...")
            break  # Stop retrying if attempts are exhausted


class VideoDisplayApp:
    def __init__(self, root, serial_numbers, frame_dict, lock, stop_event):
        self.root = root
        self.serial_numbers = serial_numbers
        self.frame_dict = frame_dict
        self.lock = lock
        self.stop_event = stop_event
        self.recording = False
        self.recorders = {}
        self.experimentFilesFolder = "experimentsData"
        self.next_experiment_no = self._find_next_experiment_number()
        print(f"[INFO] Next experiment number: {self.next_experiment_no}")

        # Initialize recording path variable
        self.recording_path = f"ExperimentVideo/Experiment_{self.next_experiment_no}_Data"

        self.root.title("Video Display")

        # Keep the window on top
        self.root.attributes('-topmost', True)

        # Determine the next experiment number
        self.next_experiment_no = self._find_next_experiment_number()

        # Create and configure frames
        self._setup_frames()
        self._setup_widgets()

        # Start the video update loop
        self.update_video_streams()

        # Handle window close
        root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _find_next_experiment_number(self, base_dir=""):
        """
        Find the next available experiment number based on existing folders.
        If base_dir is empty, use the current working directory.
        """
        # Use current working directory if base_dir is empty
        if not base_dir:
            base_dir = self.experimentFilesFolder


        # If the directory doesn't exist, start with Experiment 1
        if not os.path.exists(base_dir):
            return 1

        # Find all folders matching the pattern Experiment_XXX_Data
        experiment_folders = [f for f in os.listdir(base_dir) if re.match(r"Experiment_\d+_Data", f)]

        if not experiment_folders:
            return 1  # If no matching folders, start with Experiment 1

        # Extract the numeric part from the folder names
        experiment_numbers = [
            int(re.search(r"Experiment_(\d+)_Data", folder).group(1))
            for folder in experiment_folders
        ]

        # Return the next available experiment number
        return max(experiment_numbers, default=0) + 1
    
    def _setup_frames(self):
        """Setup frames for the application layout."""
        self.top_left_frame = ttk.Frame(self.root)
        self.top_right_frame = ttk.Frame(self.root)
        self.bottom_frame = ttk.Frame(self.root)

        self.top_left_frame.grid(row=0, column=0, sticky="nsew")
        self.top_right_frame.grid(row=0, column=1, sticky="nsew")
        self.bottom_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")

        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=0)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)

    def _setup_widgets(self):
        """Create widgets for the application."""
        # Create canvases for video streams
        self.canvas1 = tk.Canvas(self.top_left_frame)
        self.canvas2 = tk.Canvas(self.top_right_frame)
        self.canvas1.pack(fill="both", expand=True)
        self.canvas2.pack(fill="both", expand=True)

        # Recording path input
        path_frame = ttk.Frame(self.bottom_frame)
        path_frame.pack(side="top", fill="x", padx=5, pady=5)

        ttk.Label(path_frame, text="Experiment No.:").pack(side="left", padx=5)
        self.recording_path_field = tk.Entry(path_frame, width=15)
        self.recording_path_field.pack(side="left", padx=5)
        self.recording_path_field.insert(0, str(self.next_experiment_no))

        # Control buttons
        self.buttons = []
        start_recording_button = ttk.Button(self.bottom_frame, text="Start Recording", command=self.toggle_recording)
        start_recording_button.pack(side="left", expand=True, fill="x")
        self.buttons.append(start_recording_button)

        for i in range(1, 5):
            btn = ttk.Button(self.bottom_frame, text=f"Button {i+1}")
            btn.pack(side="left", expand=True, fill="x")
            self.buttons.append(btn)


    def toggle_recording(self):
        """Toggle the recording state and handle video file saving."""
        self.recording = not self.recording

        if self.recording:
            # Get the current experiment number
            experiment_no = self.next_experiment_no

            # Construct the recording path
            save_path = os.path.join(self.experimentFilesFolder, f"Experiment_{experiment_no}_Data")
            print(f"[INFO] Recording path: {save_path}")

            try:
                # Create the directory if it doesn't exist
                os.makedirs(save_path, exist_ok=True)
                print(f"[INFO] Path '{save_path}' is ready.")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to create directory '{save_path}': {e}")
                self.recording = False
                return

            self.buttons[0].config(text="Stop Recording")
            timestamp = time.strftime("%Y%m%d-%H%M%S")

            # Initialize video writers for each camera
            for sn in self.serial_numbers:
                try:
                    fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    if sn == '021222072398':
                        file_name = os.path.join(save_path, f"KobiThesis_RobotCameraRecord_{timestamp}.avi")
                    elif sn == '021222070941':
                        file_name = os.path.join(save_path, f"KobiThesis_ParticipantCameraRecord_{timestamp}.avi")
                    else:
                        continue
                    print(f"[INFO] Creating video writer for {sn}: {file_name}")
                    self.recorders[sn] = cv2.VideoWriter(file_name, fourcc, 20.0, (640, 480))

                    # Check if the video writer is opened successfully
                    if not self.recorders[sn].isOpened():
                        raise IOError(f"VideoWriter failed to open for {file_name}")
                except Exception as e:
                    messagebox.showerror("Error", f"Failed to initialize recorder for {sn}: {e}")
                    self.recording = False
                    return
        else:
            self.buttons[0].config(text="Start Recording")
            # Release all video writers
            for sn, out in self.recorders.items():
                print(f"[INFO] Releasing recorder for {sn}")
                out.release()
            self.recorders.clear()
            messagebox.showinfo("Recording Stopped", "Recording has been stopped.")

            # Update the experiment number
            self.next_experiment_no = self._find_next_experiment_number()
            self.recording_path_field.delete(0, tk.END)  # Clear the current field
            self.recording_path_field.insert(0, str(self.next_experiment_no))  # Set the updated number
            print(f"[INFO] Updated experiment number: {self.next_experiment_no}")


    def _initialize_recording(self, save_path):
        """Initialize the recording setup."""
        try:
            # Create the directory if it doesn't exist
            os.makedirs(save_path, exist_ok=True)
            print(f"[INFO] Recording path: {save_path} created.")

            self.buttons[0].config(text="Stop Recording")
            timestamp = time.strftime("%Y%m%d-%H%M%S")

            # Initialize video writers for each camera
            for sn in self.serial_numbers:
                try:
                    fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    if sn == '021222072398':
                        file_name = os.path.join(save_path, f"KobiThesis_RobotCameraRecord_{timestamp}.avi")
                    elif sn == '021222070941':
                        file_name = os.path.join(save_path, f"KobiThesis_ParticipantCameraRecord_{timestamp}.avi")
                    else:
                        continue
                    self._create_video_writer(sn, file_name, fourcc)
                except Exception as e:
                    messagebox.showerror("Error", f"Failed to initialize recorder for {sn}: {e}")
                    self.recording = False
                    return
        except Exception as e:
            messagebox.showerror("Error", f"Failed to create directory '{save_path}': {e}")
            self.recording = False

    def _create_video_writer(self, serial_number, file_name, fourcc):
        """Create a video writer for the given serial number."""
        writer = cv2.VideoWriter(file_name, fourcc, 20.0, (640, 480))
        if not writer.isOpened():
            raise IOError(f"VideoWriter failed to open for {file_name}")
        self.recorders[serial_number] = writer
        print(f"[INFO] Video writer created for {serial_number}: {file_name}")

    def _stop_recording(self):
        """Stop the recording and release resources."""
        self.buttons[0].config(text="Start Recording")
        for sn, out in self.recorders.items():
            print(f"[INFO] Releasing recorder for {sn}")
            out.release()
        self.recorders.clear()
        messagebox.showinfo("Recording Stopped", "Recording has been stopped.")

    def update_video_streams(self):
        """Update video streams and save frames if recording."""
        frames = []
        with self.lock:
            for sn in self.serial_numbers:
                frame = self.frame_dict.get(sn)
                if frame is not None:
                    frames.append((sn, frame))

        if len(frames) == len(self.serial_numbers):
            self._update_canvases(frames)
            if self.recording:
                self._save_frames(frames)

        self.root.after(10, self.update_video_streams)

    def _update_canvases(self, frames):
        """Update canvases with the latest frames."""
        self._show_frame(frames[0][1], self.canvas1)
        self._show_frame(frames[1][1], self.canvas2)

    def _show_frame(self, frame, canvas):
        """Display a frame on the given canvas."""
        frame = cv2.resize(frame, (canvas.winfo_width(), canvas.winfo_height()))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = ImageTk.PhotoImage(image=Image.fromarray(frame))
        canvas.create_image(0, 0, anchor="nw", image=image)
        canvas.image = image

    def _save_frames(self, frames):
        """Save frames to video files."""
        for sn, frame in frames:
            if sn in self.recorders:
                self.recorders[sn].write(frame)

    def on_closing(self):
        """Ensure the camera process stops when closing the form."""
        print("[INFO] Closing Video Display...")
        
        self.stop_event.set()  # Signal threads to stop
        time.sleep(1)  # Give some time for cleanup

        self.root.quit()  # Stop the Tkinter main loop
        self.root.destroy()  # Destroy the window
        print("[INFO] Video form closed successfully.")


def main():
    # Initialize the RealSense context
    ctx = rs.context()
    devices = ctx.query_devices()
    
    # Get the serial numbers of the connected devices
    serial_numbers = [device.get_info(rs.camera_info.serial_number) for device in devices]

    if len(serial_numbers) < 2:
        print("[ERROR] At least two RealSense devices are required.")
        return
    
    # Shared dictionary to hold frames from each camera
    frame_dict = {}
    lock = threading.Lock()
    
    # Create a stop event to signal threads to stop
    stop_event = threading.Event()

    # Start a thread for each camera
    threads = []
    for sn in serial_numbers:
        thread = threading.Thread(target=capture_frames, args=(sn, frame_dict, lock, stop_event))
        threads.append(thread)
        thread.start()
    
    # Start the GUI application
    root = tk.Tk()
    app = VideoDisplayApp(root, serial_numbers, frame_dict, lock, stop_event)
    root.mainloop()
    
    # Wait for all threads to finish
    for thread in threads:
        thread.join()

if __name__ == "__main__":
    main()
