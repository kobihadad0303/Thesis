import tkinter as tk
from tkinter import simpledialog, messagebox, filedialog
from tkinter import ttk
import json
import time
import tkinter
import urx
import math
import os
import sys
import subprocess
import re
import psutil  # New import to kill any stuck processes
import winsound

# Realsense
from PIL import Image, ImageTk
import pyrealsense2 as rs
import cv2
import numpy as np
import threading

class URRobotApp:

    CLICK_DELAY = 250  # Class attribute for click delay in milliseconds

    def __init__(self, master):

        self.robot = None
        self.robot_ip = "10.0.0.204"
        self.waypoints = []  # Store positions as joint angles
        self.descriptions = []  # Store descriptions as strings
        self.sequence = []  # List to store indices of waypoints in the sequence
        self.tasks_sequences = []  # List to store indices of waypoints in the sequence
        self.basic_sequences_list = []  # Store sequences as list of tuples (sequence name, sequence data)
        self.tasks_sequences_list = []  # Store sequences as list of tuples (sequence name, sequence data)
        self.single_button_pressed = False
        self.delete_mode = False  # Track whether we are in delete mode
        self.update_mode = False  # Track whether we are in update waypoint mode
        self.update_sequence_mode = False  # Track whether we are in update sequence mode
        self.delete_sequence_mode = False  # Track whether we are in delete sequence mode
        self.update_tasks_sequences_mode = False
        self.delete_tasks_sequences_mode = False  
        self.append_mode = False  # Initialize the append mode as False
        self.waypoint_frames = []  # Store frames that hold the buttons and entries
        self.waypoint_description_entries = []  # Store Entry widgets for descriptions
        self.current_index = -1  # Initialize current_index to start before the first item
        self.suppress_messages = False  # Add this flag to suppress messages
        self.mark_sequences = False  # Add this flag to disable marking sequences
        self.time_counts = []  # List to store time counts for each step
        self.start_time = None  # Variable to store the start time for the current step
        self.button_counters = {}
        self.master = master
        self.master.title("UR Robot Control")
        self.master.state('zoomed')  # Make the window full screen
        
        # Create experiment number entry field
        self.experimenter_no_entry = tk.Entry(self.master, width=20)
        self.experimenter_no_entry.pack(pady=5)  # Place it appropriately in your layout

        # Load waypoints, basic sequences and task sequences
        self.txtFilesFolder = "txtFiles"
        self.experimentsFilesFolder = "experimentsData"
        
        # Determine the next experiment number
        self.experiment_no = self._find_next_experiment_number()
        
        # Populate the entry field with the experiment number
        self.experimenter_no_entry.insert(0, str(self.experiment_no))   



        # Top frame---------------------------------------------------------------------------------------------------------------------------------
        self.messages_frame = tk.Frame(master)#, borderwidth=1, relief=tk.SOLID)
        self.messages_frame.pack(fill=tk.X, pady=5)
        # Top buttons
        self.connect_button = tk.Button(self.messages_frame, text="Connect to Robot", command=self.connect_robot)
        self.connect_button.pack(side=tk.LEFT, padx=5)
        self.disconnect_button = tk.Button(self.messages_frame, text="Disconnect from Robot", command=self.disconnect_robot, state=tk.DISABLED)
        self.disconnect_button.pack(side=tk.LEFT, padx=5)
        self.move_up_button = tk.Button(self.messages_frame, text="Move up", command=self.move_item_up)
        self.move_up_button.pack(side=tk.LEFT, padx=5)
        self.move_down_button = tk.Button(self.messages_frame, text="Move down", command=self.move_item_down)
        self.move_down_button.pack(side=tk.LEFT, padx=5)
        speed_label = tk.Label(self.messages_frame, text="Speed Control:")
        speed_label.pack(side=tk.LEFT, padx=10)
        self.speed_value_label = tk.Label(self.messages_frame, text="6")
        self.speed_value_label.pack(side=tk.LEFT)
        self.speed_scale = tk.Scale(self.messages_frame, from_=0.1, to=10.0, resolution=0.01, orient=tk.HORIZONTAL, length=300, showvalue=0)
        self.speed_scale.set(6)  # Default speed
        self.speed_scale.pack(side=tk.LEFT, padx=10)
        self.speed_scale.config(command=self.update_speed_value)
        self.acc_value_label = tk.Label(self.messages_frame, text="5")
        self.acc_value_label.pack(side=tk.LEFT)
        self.acc_scale = tk.Scale(self.messages_frame, from_=0.1, to=10.0, resolution=0.01, orient=tk.HORIZONTAL, length=300, showvalue=0)
        self.acc_scale.set(5)  # Default acc
        self.acc_scale.pack(side=tk.LEFT, padx=10)
        self.acc_scale.config(command=self.update_acc_value)
        self.exit_button = tk.Button(self.messages_frame, text="Exit", command=self.exit_app)
        self.exit_button.pack(side=tk.RIGHT, padx=10)
        # Top frame---------------------------------------------------------------------------------------------------------------------------------


        # Main frame---------------------------------------------------------------------------------------------------------------------------------
        self.main_frame = tk.Frame(master)
        self.main_frame.pack(side=tk.LEFT, fill=tk.Y, pady=5)

        # Waypoints frame
        self.waypoints_frame = tk.Frame(self.main_frame, borderwidth=1, relief=tk.SOLID)
        self.waypoints_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        self.waypoints_frame_title = tk.Label(self.waypoints_frame, text="Waypoints", font=("Arial", 12))
        self.waypoints_frame_title.pack(side=tk.TOP)
        # Waypoints listbox
        self.waypoints_listbox = tk.Listbox(self.waypoints_frame)
        self.waypoints_listbox.pack(fill=tk.BOTH, expand=True,padx=5)
        self.waypoints_listbox.bind("<<ListboxSelect>>", self.on_waypoint_click)  # Single-click with delay check
        self.waypoints_listbox.bind("<Double-1>", self.edit_waypoint_description)  # Double-click to edit description

        # Waypoints buttons frame
        waypoints_button_frame = tk.Frame(self.waypoints_frame)
        waypoints_button_frame.pack(side=tk.BOTTOM, pady=5)
        # Waypoints buttons
        self.seq_open_tool_button = tk.Button(waypoints_button_frame, text="Seq. Open Tool", command=self.add_seq_open_tool)
        self.seq_open_tool_button.pack(side=tk.LEFT)
        self.seq_close_tool_button = tk.Button(waypoints_button_frame, text="Seq. Close Tool", command=self.add_seq_close_tool)
        self.seq_close_tool_button.pack(side=tk.LEFT)
        self.add_waypoint_button = tk.Button(waypoints_button_frame, text="Add Waypoint", command=self.add_waypoint, state=tk.DISABLED)
        self.add_waypoint_button.pack(side=tk.LEFT)
        self.delete_waypoint_button = tk.Button(waypoints_button_frame, text="Delete Waypoint", command=self.enter_delete_mode, state=tk.DISABLED)
        self.delete_waypoint_button.pack(side=tk.LEFT)
        self.edit_waypoint_button = tk.Button(waypoints_button_frame, text="Update Waypoint", command=self.edit_selected_waypoint, state=tk.DISABLED)
        self.edit_waypoint_button.pack(side=tk.LEFT)
        # Main frame---------------------------------------------------------------------------------------------------------------------------------


        # Basic sequences frame---------------------------------------------------------------------------------------------------------------------------------
        self.basic_sequences_frame = tk.Frame(self.main_frame, borderwidth=1, relief=tk.SOLID)
        self.basic_sequences_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        self.sequence_frame_title = tk.Label(self.basic_sequences_frame, text="Basic sequences", font=("Arial", 12))
        self.sequence_frame_title.pack(side=tk.TOP)
        # Basic sequences listbox
        self.basic_sequence_listbox = tk.Listbox(self.basic_sequences_frame)  # Correctly named and initialized
        self.basic_sequence_listbox.pack(fill=tk.BOTH, expand=True, padx=5)
        self.basic_sequence_listbox.bind("<<ListboxSelect>>", lambda event: self.display_basic_sequence_waypoints())
        self.basic_sequence_listbox.bind("<Double-1>", self.edit_basic_sequence_description)

        # Basic sequences buttons frame
        self.basic_sequences_button_frame = tk.Frame(self.basic_sequences_frame)
        self.basic_sequences_button_frame.pack(side=tk.BOTTOM, pady=5)
        # Basic sequences buttons
        self.delete_basic_sequence_button = tk.Button(self.basic_sequences_button_frame, text="Delete Sequence", command=self.enter_delete_sequence_mode, state=tk.DISABLED)
        self.delete_basic_sequence_button.pack(side=tk.LEFT, fill=tk.X)
        self.update_basic_sequence_button = tk.Button(self.basic_sequences_button_frame, text="Update Sequence", command=self.enter_update_sequence_mode, state=tk.DISABLED)
        self.update_basic_sequence_button.pack(side=tk.LEFT, fill=tk.X)
        self.add_basic_sequence_button = tk.Button(self.basic_sequences_button_frame, text="add new sequence", command=self.add_sequence, state=tk.DISABLED)
        self.add_basic_sequence_button.pack(side=tk.LEFT, fill=tk.X)
        # Basic sequences frame---------------------------------------------------------------------------------------------------------------------------------


        # Tasks sequences frame---------------------------------------------------------------------------------------------------------------------------------
        self.tasks_sequences_frame = tk.Frame(self.main_frame, borderwidth=1, relief=tk.SOLID)
        self.tasks_sequences_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        self.task_frame_title = tk.Label(self.tasks_sequences_frame, text="Tasks sequences", font=("Arial", 12))
        self.task_frame_title.pack(side=tk.TOP)
        # Tasks sequences listbox
        self.tasks_sequences_listbox = tk.Listbox(self.tasks_sequences_frame)  # Correctly named and initialized
        self.tasks_sequences_listbox.pack(fill=tk.BOTH, expand=True, padx=5)
        self.tasks_sequences_listbox.bind("<<ListboxSelect>>", lambda event: self.display_tasks_sequences_waypoints())
        self.tasks_sequences_listbox.bind("<Double-1>", self.edit_tasks_sequences_description)

        # Tasks sequences buttons frame
        self.tasks_sequences_button_frame = tk.Frame(self.tasks_sequences_frame)
        self.tasks_sequences_button_frame.pack(side=tk.BOTTOM, pady=5)
        # Tasks sequences buttons
        self.delete_tasks_sequences_button = tk.Button(self.tasks_sequences_button_frame, text="Delete Sequence", command=self.enter_delete_task_sequence_mode, state=tk.DISABLED)
        self.delete_tasks_sequences_button.pack(side=tk.LEFT, fill=tk.X)
        self.update_tasks_sequences_button = tk.Button(self.tasks_sequences_button_frame, text="Update Sequence", command=self.enter_update_task_sequence_mode, state=tk.NORMAL)
        self.update_tasks_sequences_button.pack(side=tk.LEFT, fill=tk.X)
        self.add_tasks_sequences_button = tk.Button(self.tasks_sequences_button_frame, text="Add New Sequence", command=self.add_task_sequence, state=tk.NORMAL)
        self.add_tasks_sequences_button.pack(side=tk.LEFT, fill=tk.X)
        # Tasks sequences frame---------------------------------------------------------------------------------------------------------------------------------


        # Action frame---------------------------------------------------------------------------------------------------------------------------------
        self.action_frame = tk.Frame(self.main_frame, borderwidth=1, relief=tk.SOLID)
        self.action_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        self.action_frame_title = tk.Label(self.action_frame, text="Action area", font=("Arial", 12))
        self.action_frame_title.pack(side=tk.TOP)
        # Execution sequence listbox
        self.Execution_sequence_waypoints = tk.Text(self.action_frame, width=40, height=20)
        self.Execution_sequence_waypoints.pack(pady=10)
        self.Execution_sequence_waypoints.bind('<KeyRelease>', self.check_waypoint_entry)

        # Execution sequence buttons frame
        self.sequence_buttons_frame = tk.Frame(self.action_frame)
        self.sequence_buttons_frame.pack(pady=0)
        # Execution sequence buttons
        self.clear_sequence_button = tk.Button(self.sequence_buttons_frame, text="clear sequence", command=self.clear_sequence, state=tk.DISABLED)
        self.clear_sequence_button.pack(side=tk.LEFT)
        self.execute_button = tk.Button(self.sequence_buttons_frame, text="execute sequence", command=self.execute_action, state=tk.DISABLED)
        self.execute_button.pack(side=tk.LEFT, pady=10)
        self.append_mode_button = tk.Button(self.sequence_buttons_frame, text="Append Mode is OFF", command=self.toggle_append_mode)
        self.append_mode_button.pack(side=tk.LEFT)

        # Execution sequence frame horizontal separator line
        Execution_sequence_frame_horizontal_separator = ttk.Separator(self.action_frame, orient='horizontal')
        Execution_sequence_frame_horizontal_separator.pack(fill=tk.X, pady=5)

        # Manual movements frame
        self.manual_movements_frame = tk.Frame(self.action_frame)
        self.manual_movements_frame.pack(fill=tk.X, pady=0)

        # Gripper frame
        self.gripper_frame = tk.Frame(self.manual_movements_frame)
        self.gripper_frame.pack(expand=True)  # Center the internal frame within tool_button_frame
        # Gripper buttons
        self.open_gripper_button = tk.Button(self.gripper_frame, text="Open Gripper", command=self.open_gripper)
        self.open_gripper_button.pack(side=tk.LEFT, padx=10)
        self.close_gripper_button = tk.Button(self.gripper_frame, text="Close Gripper", command=self.close_gripper)
        self.close_gripper_button.pack(side=tk.LEFT, padx=10)

        # Movement frame
        self.movement_frame = tk.Frame(self.manual_movements_frame)
        self.movement_frame.pack(fill=tk.Y, pady=10)
        distance_label = tk.Label(self.movement_frame, text="Move Distance (mm):")
        distance_label.grid(row=0, column=0, columnspan=2, padx=10, pady=5)
        self.distance_entry = tk.Entry(self.movement_frame, width=10)
        self.distance_entry.grid(row=0, column=2, columnspan=4, padx=10, pady=5)
        self.distance_entry.insert(0, "10")  # Default to 10 mm
        # Movement buttons
        button_width = 10  # Set a uniform width for all buttons
        button_height = 2  # Set a uniform height for all buttons
        self.up_button = tk.Button(self.movement_frame, text="Up", width=button_width, height=button_height, command=lambda: self.move_robot("up"))
        self.up_button.grid(row=1, column=1, padx=2, pady=2)
        self.down_button = tk.Button(self.movement_frame, text="Down", width=button_width, height=button_height, command=lambda: self.move_robot("down"))
        self.down_button.grid(row=3, column=1, padx=2, pady=2)
        self.left_button = tk.Button(self.movement_frame, text="Left", width=button_width, height=button_height, command=lambda: self.move_robot("left"))
        self.left_button.grid(row=2, column=0, padx=2, pady=2)
        self.right_button = tk.Button(self.movement_frame, text="Right", width=button_width, height=button_height, command=lambda: self.move_robot("right"))
        self.right_button.grid(row=2, column=2, padx=2, pady=2)
        self.forward_button = tk.Button(self.movement_frame, text="Forward", width=button_width, height=button_height, command=lambda: self.move_robot("forward"))
        self.forward_button.grid(row=4, column=0, padx=2, pady=2)
        self.backward_button = tk.Button(self.movement_frame, text="Backward", width=button_width, height=button_height, command=lambda: self.move_robot("backward"))
        self.backward_button.grid(row=4, column=2, padx=2, pady=2)


        # Launch User Form Execution button
        self.launch_user_form_button = tk.Button(self.master, text="Launch User Form Execution", command=self.create_user_form_execution_window)
        self.launch_user_form_button.pack(pady=10)  # Add this at the bottom of the main window
        # Add a button to open the Video Form
        self.open_video_form_button = ttk.Button(self.master, text="Open Video Form", command=self.open_video_form)
        self.open_video_form_button.pack(pady=10)


        


        os.makedirs(self.txtFilesFolder, exist_ok=True)
        os.makedirs(self.experimentsFilesFolder, exist_ok=True)
        
                
        self.load_waypoints_from_file(os.path.join(self.txtFilesFolder, "waypoints.txt"))
        self.load_basic_sequences_from_file(os.path.join(self.txtFilesFolder, "basic_sequences.txt"))
        self.load_tasks_sequences_from_file(os.path.join(self.txtFilesFolder, "tasks_sequences.txt"))
        
        # Attempt to connect automatically on startup
        self.master.after(1000, self.auto_connect)
         
        # Initialize user_form as None
        self.user_form = None 
        self.video_process = None  # Initialize a variable to track the process
        self.open_video_form()
 

        # DI#0 reading
        self.digital_input_label = tk.Label(self.movement_frame, text="Digital Input 0 State: Unknown")
        self.digital_input_label.grid(row=5, column=0, padx=5, pady=5)
        
        # Initialize flags
        self.task_in_progress = False  # To track if a task is currently executing

        # Start the thread to continuously read digital input
        self.running = True
        self.input_thread = threading.Thread(target=self.monitor_digital_input)
        self.input_thread.start()
        

    def _find_next_experiment_number(self, base_dir=""):
        """
        Find the next available experiment number based on existing folders.
        If base_dir is empty, use the current working directory.
        """
        if not base_dir:
            base_dir = self.experimentsFilesFolder
            # print(f"[DEBUG] Base directory being scanned: {base_dir}")


        # Check if the base directory exists
        if not os.path.exists(base_dir):
            return 1

        # Find all folders matching the pattern Experiment_XXX
        experiment_folders = [f for f in os.listdir(base_dir) if re.match(r"Experiment_\d+_Data", f)]
        # print(f"[DEBUG] Experiment folders found: {experiment_folders}")

        if not experiment_folders:
            return 1  # No matching folders, start with 1

        # Extract numeric values from folder names
        experiment_numbers = [
            int(re.search(r"Experiment_(\d+)_Data", folder).group(1))
            for folder in experiment_folders
        ]

        # Return the next available experiment number
        return max(experiment_numbers, default=0) + 1


    def load_counters_from_file(self):
        """Load the button counters from the file."""
        file_path = os.path.join(os.path.join(self.txtFilesFolder, "button_counters.txt"))

        # Ensure the counters dictionary is initialized
        self.button_counters = {}

        if os.path.exists(file_path):
            # print(f"Loading counters from {file_path}")
            with open(file_path, "r") as file:
                for line in file:
                    # print(f"Reading line: {line.strip()}")
                    try:
                        button_name, count = line.strip().split(": ")
                        self.button_counters[button_name] = int(count)
                        # print(f"Loaded {button_name}: {count}")
                    except ValueError as e:
                        print(f"Skipping malformed line: {line.strip()} ({e})")
        else:
            print(f"File not found: {file_path}")

    def increment_counter(self, button_name, counter_label):
        """Increment the counter and update the label."""
        if button_name not in self.button_counters:
            self.button_counters[button_name] = 0
        self.button_counters[button_name] += 1
        counter_label.config(text=f"Count: {self.button_counters[button_name]}")
        # print(f"Updated counter for {button_name}: {self.button_counters[button_name]}")

        # Save updated counters
        self.save_counters_to_file()
        
    def save_counters_to_file(self):
        """Save the button counters to a file."""
        file_path = os.path.join(os.path.join(self.txtFilesFolder, "button_counters.txt"))
        os.makedirs(self.txtFilesFolder, exist_ok=True)

        with open(file_path, "w") as file:
            for button_name, count in self.button_counters.items():
                file.write(f"{button_name}: {count}\n")
        # print(f"Counters saved: {self.button_counters}")                  

    # Save Experiment data Button
    def save_experiment_data(self):
        """
        Save the experiment data
        """

        self.calculate_total_time = False

        # Get Experimenter details from the form
        experiment_no = self.experimenter_no_entry.get().strip()
        first_name = self.experimenter_first_name_entry.get().strip()
        last_name = self.experimenter_last_name_entry.get().strip()

        if not experiment_no or not first_name or not last_name:
            messagebox.showwarning("Incomplete Experimenter Details", "Please fill in all the fields.")
            return

        # Construct the folder path
        save_path = os.path.join(self.experimentsFilesFolder, f"Experiment_{experiment_no}_Data")
        
        try:
            # Create the folder if it doesn't exist
            os.makedirs(save_path, exist_ok=True)

            # Save Experiment data to a file with UTF-8 encoding
            file_path = os.path.join(save_path, "experiment_data.txt")
            with open(file_path, "w", encoding="utf-8") as file:
                # Save execution steps and times
                steps = self.user_form_execution_steps_listbox.get(0, tk.END)
                times = self.user_form_execution_time_listbox.get(0, tk.END)                
                total_time = sum(float(times[idx]) for idx in range(len(times)))

                # Write experiment data
                file.write(f"Experiment No.: {experiment_no}\n")
                file.write(f"First Name: {first_name}\n")
                file.write(f"Last Name: {last_name}\n")
                file.write(f"Total Time: {total_time:.2f}\n")
                file.write("\n\nExecution Steps:\n")



                if len(steps) != len(times):
                    print("[ERROR] Mismatched lengths between steps and times!")
                    raise ValueError("Steps and times listboxes have mismatched lengths.")

                for step, time in zip(steps, times):
                    try:
                        # print(f"[DEBUG] Writing Step: {step}, Time: {time}")
                        file.write(f"{step}: {time}\n")
                    except Exception as e:
                        print(f"[ERROR] Failed to write Step: {step}, Time: {time}. Error: {e}")

            # messagebox.showinfo("Experiment data Saved", f"Experiment data saved to {file_path}")
            print(f"[INFO]Experiment data saved to {file_path}")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to save Experiment data: {e}")
            print(f"[ERROR] Failed to save Experiment data: {e}")


    def update_total_time_periodically(self):
        """Update total time periodically while the experiment is active."""
        if not getattr(self, 'calculate_total_time', False):
            return  # Stop calculation if the flag is False

        try:
            # Calculate the total time
            total_time = sum(float(self.user_form_execution_time_listbox.get(idx)) for idx in range(self.user_form_execution_time_listbox.size()))
            self.total_time_label.config(text=f"{total_time:.2f}")
        except Exception as e:
            print(f"[ERROR] Failed to calculate total time: {e}")

        # Schedule the next update
        self.user_form_execution_window.after(1000, self.update_total_time_periodically)


    def create_user_form_execution_window(self):
        # Check if the window is already open
        if hasattr(self, 'user_form_execution_window') and self.user_form_execution_window.winfo_exists():
            self.user_form_execution_window.focus_set()
            return

        # Load counters from the file
        self.load_counters_from_file()
        # print(f"Loaded counters: {self.button_counters}")

        # Create the window
        self.user_form_execution_window = tk.Toplevel(self.master)
        self.user_form_execution_window.title("User Form Execution")
        self.user_form_execution_window.geometry("800x600")

        # Main layout frame
        main_frame = tk.Frame(self.user_form_execution_window)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Left Frame for buttons
        left_frame = tk.Frame(main_frame, padx=10, pady=10)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH)

        # Left-Top Frame for Experiment data
        leftTop_frame = tk.LabelFrame(left_frame, text="Experiment data", padx=10, pady=10)
        leftTop_frame.pack(side=tk.TOP, fill=tk.BOTH, padx=5, pady=5)

        # Experiment data on leftTop_frame
        tk.Label(leftTop_frame, text="No.:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        next_experiment_no = self._find_next_experiment_number()
        self.experimenter_no_entry = tk.Entry(leftTop_frame, width=10)
        self.experimenter_no_entry.grid(row=0, column=1, padx=5, pady=5)
        self.experimenter_no_entry.insert(0, str(next_experiment_no))

        tk.Label(leftTop_frame, text="First Name:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.experimenter_first_name_entry = tk.Entry(leftTop_frame, width=20)
        self.experimenter_first_name_entry.grid(row=1, column=1, padx=5, pady=5)

        tk.Label(leftTop_frame, text="Last Name:").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        self.experimenter_last_name_entry = tk.Entry(leftTop_frame, width=20)
        self.experimenter_last_name_entry.grid(row=2, column=1, padx=5, pady=5)

        # Total Time Label
        tk.Label(leftTop_frame, text="Total Time:").grid(row=4, column=0, padx=5, pady=5, sticky=tk.W)
        self.total_time_label = tk.Label(leftTop_frame, text="0.00", width=20, anchor='w')
        self.total_time_label.grid(row=4, column=1, padx=5, pady=5)
        self.calculate_total_time = True
        self.update_total_time_periodically()

        tk.Button(leftTop_frame, text="Save Experiment data", command=self.save_experiment_data).grid(row=3, columnspan=2, pady=10)

        # Left-bottom frame for Buttons
        leftBottom_frame = tk.LabelFrame(left_frame, text="Experiment Select", padx=10, pady=10)
        leftBottom_frame.pack(side=tk.TOP, fill=tk.BOTH, padx=5, pady=5)

        def create_button_with_counter(frame, text, command):
            """
            Helper function to create a button with an associated counter.
            
            :param frame: Parent frame in which to place the button and counter.
            :param text: Text to display on the button.
            :param command: Function to execute when the button is clicked.
            :return: The created button widget.
            """
            # Create a container frame for the button and its counter
            button_frame = tk.Frame(frame)
            button_frame.pack(side=tk.LEFT, padx=5)

            # Retrieve the initial counter value, defaulting to 0 if not found
            initial_count = self.button_counters.get(text, 0)

            # Create a label to display the counter
            counter_label = tk.Label(button_frame, text=f"Count: {initial_count}")
            counter_label.pack()

            # Create the button with the provided text and command
            button = tk.Button(
                button_frame,
                width=10,
                text=text,
                command=lambda: [self.increment_counter(text, counter_label), command()]
            )
            button.pack()

            # Return the created button widget for further customization
            return button

        # First line of buttons
        first_line_label = tk.Label(leftBottom_frame, text="G1", font=("Arial", 12, "bold"))
        first_line_label.pack(anchor=tk.W, padx=5, pady=(20, 0))
        first_line_execution_buttons_frame = tk.Frame(leftBottom_frame)
        first_line_execution_buttons_frame.pack(fill=tk.X)

        button = create_button_with_counter(first_line_execution_buttons_frame, "L-M1-H", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "L_M1_H.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(first_line_execution_buttons_frame, "L-H-M1", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "L_H_M1.txt")))
        # button.config(state=tk.DISABLED)
        create_button_with_counter(first_line_execution_buttons_frame, "M1-L-H", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M1_L_H.txt")))
        button = create_button_with_counter(first_line_execution_buttons_frame, "M1-H-L", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M1_H_L.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(first_line_execution_buttons_frame, "H-M1-L", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "H_M1_L.txt")))
        # button.config(state=tk.DISABLED)
        create_button_with_counter(first_line_execution_buttons_frame, "H-L-M1", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "H_L_M1.txt")))


        # Second line of buttons
        second_line_label = tk.Label(leftBottom_frame, text="G2", font=("Arial", 12, "bold"))
        second_line_label.pack(anchor=tk.W, padx=5, pady=(20, 0))
        second_line_execution_buttons_frame = tk.Frame(leftBottom_frame)
        second_line_execution_buttons_frame.pack(fill=tk.X)

        button = create_button_with_counter(second_line_execution_buttons_frame, "L-M2-H", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "L_M2_H.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(second_line_execution_buttons_frame, "L-H-M2", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "L_H_M2.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(second_line_execution_buttons_frame, "M2-L-H", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M2_L_H.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(second_line_execution_buttons_frame, "M2-H-L", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M2_H_L.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(second_line_execution_buttons_frame, "H-M2-L", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "H_M2_L.txt")))
        # button.config(state=tk.DISABLED)
        create_button_with_counter(second_line_execution_buttons_frame, "H-L-M2", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "H_L_M2.txt")))
        

        # Third line of buttons
        third_line_label = tk.Label(leftBottom_frame, text="G3", font=("Arial", 12, "bold"))
        third_line_label.pack(anchor=tk.W, padx=5, pady=(20, 0))
        third_line_execution_buttons_frame = tk.Frame(leftBottom_frame)
        third_line_execution_buttons_frame.pack(fill=tk.X)

        button = create_button_with_counter(third_line_execution_buttons_frame, "L-M1-M2", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "L_M1_M2.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(third_line_execution_buttons_frame, "L-M2-M1", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "L_M2_M1.txt")))
        # button.config(state=tk.DISABLED)
        create_button_with_counter(third_line_execution_buttons_frame, "M1-L-M2", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M1_L_M2.txt")))
        button = create_button_with_counter(third_line_execution_buttons_frame, "M1-M2-L", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M1_M2_L.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(third_line_execution_buttons_frame, "M2-L-M1", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M2_L_M1.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(third_line_execution_buttons_frame, "M2-M1-L", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M2_M1_L.txt")))
        # button.config(state=tk.DISABLED)


        # Fourth line of buttons
        fourth_line_label = tk.Label(leftBottom_frame, text="G4", font=("Arial", 12, "bold"))
        fourth_line_label.pack(anchor=tk.W, padx=5, pady=(20, 0))
        fourth_line_execution_buttons_frame = tk.Frame(leftBottom_frame)
        fourth_line_execution_buttons_frame.pack(fill=tk.X)

        button = create_button_with_counter(fourth_line_execution_buttons_frame, "H-M1-M2", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "H_M1_M2.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(fourth_line_execution_buttons_frame, "H-M2-M1", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "H_M2_M1.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(fourth_line_execution_buttons_frame, "M1-H-M2", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M1_H_M2.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(fourth_line_execution_buttons_frame, "M1-M2-H", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M1_M2_H.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(fourth_line_execution_buttons_frame, "M2-H-M1", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M2_H_M1.txt")))
        # button.config(state=tk.DISABLED)
        button = create_button_with_counter(fourth_line_execution_buttons_frame, "M2-M1-H", lambda: self.user_form_create(os.path.join(self.txtFilesFolder, "M2_M1_H.txt")))
        # button.config(state=tk.DISABLED)
  

        # Right Frame for experiment data
        right_frame = tk.LabelFrame(main_frame, text="User Form Execution", padx=10, pady=10)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)






        # # User Form Execution Widgets
        # task_frame_title = tk.Label(self.user_form_execution_window, text="User Form Execution", font=("Arial", 20))
        # task_frame_title.pack(side=tk.TOP)




        # Execution steps frame
        execution_steps_frame = tk.Frame(right_frame)
        execution_steps_frame.pack(fill=tk.BOTH, expand=True)

        # Execution steps listbox
        self.user_form_execution_time_listbox = tk.Listbox(execution_steps_frame, width=10)
        self.user_form_execution_time_listbox.pack(side=tk.LEFT, fill=tk.BOTH)
        self.user_form_execution_steps_listbox = tk.Listbox(execution_steps_frame, width=50)
        self.user_form_execution_steps_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)


        # User Form Execution buttons frame
        user_form_execution_button_frame = tk.Frame(right_frame)
        user_form_execution_button_frame.pack(side=tk.BOTTOM, pady=5)

        # User Form Execution buttons
        self.clear_user_form_execution_sequence_button = tk.Button(
            user_form_execution_button_frame, text="Clear State", command=self.clear_user_execution_sequence, state=tk.DISABLED
        )
        self.clear_user_form_execution_sequence_button.pack(side=tk.LEFT, fill=tk.X, pady=5)
        self.set_next_step_button = tk.Button(user_form_execution_button_frame, text="Select Next Step", command=self.select_next_step)
        self.set_next_step_button.pack(side=tk.LEFT, fill=tk.X, pady=5)
        self.set_prev_step_button = tk.Button(user_form_execution_button_frame, text="Select Prev. Step", command=self.select_prev_step)
        self.set_prev_step_button.pack(side=tk.LEFT, fill=tk.X, pady=5)
        self.reset_button = tk.Button(user_form_execution_button_frame, text="Reset", command=self.reset_steps_states)
        self.reset_button.pack(side=tk.LEFT, fill=tk.X, pady=5)
        self.jump_to_step_button = tk.Button(user_form_execution_button_frame, text="Jump to Step", command=self.jump_to_selected_step)
        self.jump_to_step_button.pack(side=tk.LEFT, fill=tk.X, pady=5)


        # Populate the new window with necessary initial data
        for _ in range(self.user_form_execution_steps_listbox.size()):
            self.time_counts.append(0)
            self.user_form_execution_time_listbox.insert(tk.END, "0.00")


    # def jump_to_selected_step(self):
    #     """Jump to the selected step in the listbox, print its time, and update time count."""
    #     selected_index = self.user_form_execution_steps_listbox.curselection()
    #     if not selected_index:
    #         messagebox.showwarning("No Selection", "Please select a step to jump to.")
    #         return

    #     # Fetch the selected step index and time
    #     step_index = selected_index[0]
    #     try:
    #         step_time = float(self.user_form_execution_time_listbox.get(step_index))
    #         print(f"[INFO] Jumping to step ({step_index}) with time: {step_time:.2f}")

    #         # Execute the selected step
    #         step_description = self.user_form_execution_steps_listbox.get(step_index)
    #         print(f"[EXECUTING] Step {step_index}: {step_description}")

    #         # Start updating the time count for the selected step
    #         self.start_time = time.time() - step_time  # Resume timing from the stored time
    #         self.current_index = step_index
    #         self.update_time_count()
    #     except ValueError:
    #         print(f"[ERROR] Invalid time value for step {step_index}")
    #     except Exception as e:
    #         print(f"[ERROR] Failed to execute step {step_index}: {e}")

    # def jump_to_selected_step(self):
    #     """Jump to the selected step in the listbox, print its time, and execute it."""
    #     selected_index = self.user_form_execution_steps_listbox.curselection()
    #     if not selected_index:
    #         messagebox.showwarning("No Selection", "Please select a step to jump to.")
    #         return

    #     # Fetch the selected step index and time
    #     step_index = selected_index[0]

    #     try:
    #         # Set the jump flag to avoid unintended behavior
    #         self.is_jumping = True

    #         # Update current_index to the selected step
    #         self.current_index = step_index

    #         step_time = float(self.user_form_execution_time_listbox.get(step_index))
    #         print(f"[INFO] Jumping to step ({step_index}) with time: {step_time:.2f}")

    #         # Highlight the selected step
    #         self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'yellow'})  # Update current index first
    #         self.user_form_execution_steps_listbox.update_idletasks()

    #         # Execute the selected step
    #         step_description = self.user_form_execution_steps_listbox.get(step_index)
    #         print(f"[EXECUTING] Step {step_index}: {step_description}")

    #         # Show the message for the selected step
    #         file_name = self.file_name_text_box.get("1.0", "end-1c").strip()
    #         self.show_message_in_frame(step_index, file_name)

    #         # Set the timer for the selected step
    #         self.start_time = time.time() - step_time
    #         self.update_time_count()

    #         # Trigger the execution logic for the selected step
    #         self.user_form_execute_step()

    #         # Reset the jump flag after execution
    #         self.is_jumping = False
    #     except ValueError:
    #         print(f"[ERROR] Invalid time value for step {step_index}")
    #     except Exception as e:
    #         print(f"[ERROR] Failed to execute step {step_index}: {e}")
    #         self.is_jumping = False


    def jump_to_selected_step(self):
        """Jump to the selected step in the listbox, print its time, and execute it."""
        selected_index = self.user_form_execution_steps_listbox.curselection()
        if not selected_index:
            messagebox.showwarning("No Selection", "Please select a step to jump to.")
            return

        # Fetch the selected step index and time
        step_index = selected_index[0]

        try:
            # Set the jump flag to avoid unintended behavior
            self.is_jumping = True

            # Update current_index to the selected step
            self.current_index = step_index

            # Fetch and print the step time
            step_time = float(self.user_form_execution_time_listbox.get(step_index))
            # print(f"[INFO] Jumping to step ({step_index}) with time: {step_time:.2f}")

            # Highlight the selected step
            self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'yellow'})  # Update current index first
            self.user_form_execution_steps_listbox.update_idletasks()

            # Show the message for the selected step
            file_name = self.file_name_text_box.get("1.0", "end-1c").strip()
            self.show_message_in_frame(step_index, file_name)

            # Start updating the time count for the selected step
            self.start_time = time.time() - step_time  # Resume timing from the stored time
            self.update_time_count()  # Continue counting time from the stored value

            # Trigger the execution logic for the selected step
            # self.user_form_execute_jump()

            # Reset the jump flag after execution
            self.is_jumping = False
        except ValueError:
            print(f"[ERROR] Invalid time value for step {step_index}")
        except Exception as e:
            print(f"[ERROR] Failed to execute step {step_index}: {e}")
            self.is_jumping = False

    def user_form_execute_jump(self):
        """Execute the currently selected step during a jump."""
        step_text = self.user_form_execution_steps_listbox.get(self.current_index)

        if step_text.startswith("הנחיה:"):
            self.stop_time_count()  # Stop the timer for the current step
            self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'green'})
            self.user_form_execution_steps_listbox.update_idletasks()
        elif step_text.startswith("רצף:"):
            self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'orange'})
            self.user_form_execution_steps_listbox.update_idletasks()

            sequence_number = int(step_text.split("רצף:")[1].split(" ")[0].strip())
            self.start_time_count()  # Start timing for the sequence step
            self.execute_task_sequence(sequence_number)  # Execute the sequence
            self.stop_time_count()  # Stop timing after the sequence is executed
            self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'green'})
            self.user_form_execution_steps_listbox.update_idletasks()
        
    # def update_time_count(self):
    #     """Continuously update the time for the current step."""
    #     if self.current_index < 0 or self.start_time is None:
    #         return

    #     # Calculate elapsed time and update the listbox
    #     elapsed_time = time.time() - self.start_time
    #     self.user_form_execution_time_listbox.delete(self.current_index)
    #     self.user_form_execution_time_listbox.insert(self.current_index, f"{elapsed_time:.2f}")

    #     # Schedule the next update
    #     if self.user_form_execution_window and self.user_form_execution_window.winfo_exists():
    #         self.user_form_execution_window.after(100, self.update_time_count)

    def update_time_count(self):
        """Continuously update the time for the current step."""
        if self.start_time:
            elapsed_time = time.time() - self.start_time
            # Check if the step is still active (not marked green or switched to another step)
            step_bg = self.user_form_execution_steps_listbox.itemcget(self.current_index, 'bg')
            if step_bg == 'green':  # Stop the timer if the step is marked green
                self.stop_time_count()  # Ensure no arguments are passed
                return

            # Otherwise, continue updating the time
            self.user_form_execution_time_listbox.delete(self.current_index)
            self.user_form_execution_time_listbox.insert(self.current_index, f"{elapsed_time:.2f}")
            self.user_form_execution_time_listbox.update_idletasks()
            if self.user_form_execution_window and self.user_form_execution_window.winfo_exists():
                self.user_form_execution_window.after(1000, self.update_time_count)



        
    def read_digital_input(self, index):
        """
        Reads the state of the digital input at the specified index.

        :param index: The index of the digital input to read (0-based).
        :return: The state of the digital input (True for HIGH, False for LOW).
        """
        if self.robot is not None:
            try:
                return self.robot.get_digital_in(index)
            except Exception as e:
                print(f"Error reading digital input {index}: {e}")
                return False  # Default to False in case of an error
        else:
            print("Robot is not connected.")
            return False

    def monitor_digital_input(self):
        while self.running:
            if self.user_form_is_open():  # Check if user_form is open
                state = self.read_digital_input(0)
                if state and not self.task_in_progress:
                    try:
                        winsound.Beep(1000, 300)  # 1000 Hz frequency, 300ms duration (Windows)
                    except:
                        print("\a")  # Alternative console beep (may not work on all OS)
                    self.save_experiment_data()
                    self.task_in_progress = True,
                    self.execute_step_button_action()
                    time.sleep(2)
            time.sleep(0.1)  # Adjust the polling interval as needed

    def user_form_is_open(self):
        """
        Check if the user_form is currently open and visible.
        Returns True if the form is open, otherwise False.
        """
        if self.user_form is None:
            return False
        return self.user_form.winfo_exists()

    
    def execute_step_button_action(self):
        """
        This method should perform the same action as pressing the execute_step_button.
        Ensure this method is called when DI#0 goes HIGH.
        """
        # Example of calling the button's command directly
        self.execute_step_button.invoke()

        # After task is done, reset the flag
        self.task_in_progress = False

    def update_hmi(self, state):
        state_text = "HIGH" if state else "LOW"
        self.digital_input_label.config(text=f"Digital Input 0 State: {state_text}")

    def stop_monitoring(self):
        self.running = False
        self.input_thread.join()
        

    def open_video_form(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # Ensure any existing process is terminated before starting a new one
        if hasattr(self, "video_process") and self.video_process is not None:
            print("[DEBUG] Terminating previous video process...")
            self.video_process.terminate()
            self.video_process.wait()  # Ensure proper termination
            self.video_process = None
            

        # Kill any previous instances of KobiThesis_Video.py
        for proc in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
            try:
                if proc.info["cmdline"] and "KobiThesis_Video.py" in proc.info["cmdline"]:
                    print(f"[DEBUG] Killing old process: {proc.info['pid']}")
                    proc.terminate()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
                        
        def run_video_form():
            try:
                print("[DEBUG] Launching KobiThesis_Video.py...")
                self.video_process = subprocess.Popen(
                    [sys.executable, os.path.join(script_dir, "KobiThesis_Video.py")],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True  # Capture output as text
                )

                stdout, stderr = self.video_process.communicate()
                if stdout:
                    print("[VIDEO STDOUT]:", stdout)
                if stderr:
                    print("[VIDEO STDERR]:", stderr)

            except subprocess.SubprocessError as e:
                print(f"[ERROR] Failed to run video form: {e}")

        # Start subprocess in a separate thread
        video_thread = threading.Thread(target=run_video_form, daemon=True)
        video_thread.start()

    def clear_user_execution_sequence(self):
        selected_index = self.user_form_execution_steps_listbox.curselection()
        if selected_index:
            index = selected_index[0]
            # Clear the background color (set it to white)
            self.user_form_execution_steps_listbox.itemconfig(index, {'bg': 'white'})
            # Clear the selection to prevent it from remaining highlighted
            self.user_form_execution_steps_listbox.selection_clear(0, tk.END)
        else:
            messagebox.showwarning("No Selection", "Please select a sequence to clear.")


    def user_form_create(self, file_name=None):
        # First, close the existing user_form if it exists
        if self.user_form is not None and self.user_form.winfo_exists():
            self.user_form.destroy()
            self.user_form = None

        # Clear the listbox before loading new content
        self.user_form_execution_steps_listbox.delete(0, tk.END)

        self.time_counts = [0.0] * self.user_form_execution_steps_listbox.size()
        self.user_form_execution_time_listbox.delete(0, tk.END)

        # Load the content from the specified file and populate the listbox
        if file_name is not None and isinstance(file_name, str):
            try:
                with open(file_name, 'r', encoding='utf-8') as file:
                    lines = file.readlines()
                    for line in lines:
                        line = line.strip()
                        title, message, number = self.extract_step_message_parts(line)
                        if line.startswith("הנחיה:"):
                            # Store only the title in the listbox
                            self.user_form_execution_steps_listbox.insert(tk.END, "הנחיה:"+title)
                        elif line.startswith("רצף:"):
                            # Store only the title in the listbox
                            self.user_form_execution_steps_listbox.insert(tk.END, "רצף:"+ number + " " + title)                            

                        # else:
                            # Insert the full line if it's not an "הנחיה:" line
                            # self.user_form_execution_steps_listbox.insert(tk.END, line)
            except FileNotFoundError:
                messagebox.showerror("Error", f"File {file_name} not found.")
            except UnicodeDecodeError:
                messagebox.showerror("Error", f"Failed to load file {file_name}: Encoding error. Try a different encoding.")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load file {file_name}: {e}")

        for _ in range(self.user_form_execution_steps_listbox.size()):
            self.time_counts.append(0)
            self.user_form_execution_time_listbox.insert(tk.END, "0.00")
            
        # Re-open the user_form after clearing and reloading the listbox
        self.user_form = tk.Toplevel(self.master)
        self.user_form.title("User Form")
        # self.user_form.attributes("-fullscreen", True)  # Set the user form to full screen
        self.user_form.attributes("-topmost", True)  # Always on top
        # self.user_form.geometry(f"{self.user_form.winfo_screenwidth()}x{self.user_form.winfo_screenheight()}")  # Set to screen size
        self.user_form.geometry("800x800")  # Set to screen size
        self.current_index = -1

        self.user_form.bind("<Escape>", lambda e: self.exit_fullscreen())

        # Create the main frame for user form
        self.User_form_main_frame = tk.Frame(self.user_form, borderwidth=1, relief=tk.SOLID)
        self.User_form_main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Top frame
        self.top_frame = tk.Frame(self.User_form_main_frame, borderwidth=2, relief=tk.SOLID)
        self.top_frame.pack(fill=tk.Y, padx=10, pady=10)
        self.top_frame.pack_forget()

        self.file_name_text_box = tk.Text(self.top_frame, height=1, width=20)
        self.file_name_text_box.pack(pady=20)

        if file_name is not None:
            self.file_name_text_box.insert(tk.END, str(file_name))

        # Check if the user_form_state.json file exists to determine if this is the first run
        if os.path.exists("user_form_state.json"):
            try:
                with open("user_form_state.json", "r") as f:
                    state = json.load(f)
                    self.current_index = state.get("current_index", -1)

                    # Clear the Execution_sequence_waypoints before loading the waypoints
                    self.Execution_sequence_waypoints.delete("1.0", tk.END)

                    # Load the waypoints
                    self.Execution_sequence_waypoints.insert("1.0", state.get("execution_text", ""))
            except FileNotFoundError:
                self.current_index = -1  # Default to -1 if the file is not found

        # Handle the event when the user form is closed
        try:
            if self.user_form is not None and self.user_form.winfo_exists():
                self.user_form.protocol("WM_DELETE_WINDOW", self.on_user_form_close)
        except tkinter.TclError:
            print("Error: Tried to set protocol on a non-existent window.")


        # Messages frame
        self.messages_frame = tk.Frame(self.User_form_main_frame, borderwidth=2, relief=tk.SOLID)
        self.messages_frame.pack(fill=tk.BOTH, anchor="e", expand=True, padx=10, pady=10)
        messages_frame_title = tk.Label(self.messages_frame, text="Messages", font=("Arial", 12))
        messages_frame_title.pack(side=tk.TOP, pady=0)

        # Bottom frame
        self.bottom_frame = tk.Frame(self.User_form_main_frame, relief=tk.SOLID)
        self.bottom_frame.pack(fill=tk.Y, side=tk.BOTTOM, padx=10, pady=10)
        # Control buttons
        self.execute_step_button = tk.Button(self.bottom_frame, text="הבא", font=("Arial", 32, "bold"), height=2, width=7, command=self.user_form_execute_step)
        # self.execute_step_button.pack(side=tk.LEFT, fill=tk.X, pady=5)
        self.execute_step_button.pack_forget()

        # Highlight the first step in yellow
        if self.user_form_execution_steps_listbox.size() > 0:
            self.user_form_execution_steps_listbox.itemconfig(0, {'bg': 'yellow'})
            self.current_index = 0  # Set the current index to the first step


        self.show_message_in_frame(0, file_name)  # Display the message in the messages_frame


    def extract_step_message_parts(self, message):
        """Extract the title and the message content from a message starting with 'הנחיה:'."""
        title = ""
        content = ""
        number=""
        if message.startswith("רצף:"):
            number = message.split("רצף:")[1].split("___כותרת:")[0].strip()

        if "___כותרת:" in message and "___הודעה:" in message:
            title = message.split("___כותרת:")[1].split("___הודעה:")[0].strip()
            content = message.split("___הודעה:")[1].strip().replace(";", "\n")
        elif "___כותרת:" in message and not("___הודעה:" in message):
            title = message.split("___כותרת:")[1].strip()
        elif not("___כותרת:" in message) and "___הודעה:" in message:
            content = message.split("___הודעה:")[1].strip().replace(";", "\n")
        return title, content, number


    def start_time_count(self):
        """Start the timer for the step at the given index."""
        self.start_time = time.time()
        self.update_time_count()


    def stop_time_count(self):
        """Stop the timer and update the total time for the current step."""
        if self.start_time is not None:
            elapsed_time = time.time() - self.start_time
            self.time_counts[self.current_index] += elapsed_time
            self.user_form_execution_time_listbox.delete(self.current_index)
            self.user_form_execution_time_listbox.insert(
                self.current_index, f"{self.time_counts[self.current_index]:.2f}"
            )
            self.start_time = None  # Stop timing



    def update_time_count(self):
        """Continuously update the time for the current step."""
        if self.start_time:
            elapsed_time = time.time() - self.start_time
            # Check if the step is still active (not marked green or switched to another step)
            step_bg = self.user_form_execution_steps_listbox.itemcget(self.current_index, 'bg')
            if step_bg == 'green': # or self.current_index != index:
                self.stop_time_count(self.current_index)
                return

            # Otherwise, continue updating the time
            self.user_form_execution_time_listbox.delete(self.current_index)
            self.user_form_execution_time_listbox.insert(self.current_index, f"{self.time_counts[self.current_index] + elapsed_time:.2f}")
            self.user_form_execution_time_listbox.update_idletasks()
            if self.user_form_execution_window and self.user_form_execution_window.winfo_exists():
                self.user_form_execution_window.after(1000, lambda: self.update_time_count())


    def user_form_execute_step(self):
        # Get the currently selected (yellow-marked) step
        total_steps = self.user_form_execution_steps_listbox.size()
        step_text = self.user_form_execution_steps_listbox.get(self.current_index)              
        if step_text.startswith("הנחיה:"):
            
            self.stop_time_count()  # Stop the timer for the current step
            self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'green'})
            self.user_form_execution_steps_listbox.update_idletasks()
            self.master.after(500, self.select_next_step())
        elif step_text.startswith("רצף:"):
            # self.stop_time_count(self.current_index-1)  # Stop the timer for the current step
            self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'orange'})
            self.user_form_execution_steps_listbox.update_idletasks()

            sequence_number = int(step_text.split("רצף:")[1].split(" ")[0].strip())
            
            self.start_time_count()  # Start timing for the sequence step
            self.execute_task_sequence(sequence_number)  # Execute the sequence
            self.stop_time_count()  # Stop timing after the sequence is executed
            self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'green'})
            self.user_form_execution_steps_listbox.update_idletasks()
            self.master.after(500, self.select_next_step())

        if self.next_index >= total_steps:
            # messagebox.showinfo("Info", "All sequences have been executed successfully.")
            self.reset_steps_states()
            # Close the user_form after all steps are executed
            if self.user_form is not None:
                self.user_form.destroy()
                self.user_form = None

 
    def execute_task_sequence(self, sequence_number):
        """Execute a task sequence based on its number."""
        if sequence_number <= len(self.tasks_sequences_list):               
            task_sequence = self.tasks_sequences_list[sequence_number - 1][1]  # Get the sequence data
            for step in task_sequence:
                if isinstance(step, int):
                    self.move_to_waypoint(self.waypoints[step]["description"])
                elif step == "O":
                    self.open_gripper()
                elif step == "C":
                    self.close_gripper()
        else:
            messagebox.showerror("Error", f"Task sequence number {sequence_number} is out of range.")


    def show_message_in_frame(self, index, file_name):
        # Open the text file and read the lines
        try:
            with open(file_name, 'r', encoding='utf-8') as file:
                lines = file.readlines()
            
            # Check if the index is within the range of the file lines
            if index < 0 or index >= len(lines):
                raise IndexError("Index out of range.")

            # Get the specific line at the given index
            message = lines[index].strip()

            # Parse the message to extract the title and content
            title, content, number = self.extract_step_message_parts(message)

            # Clear previous message
            for widget in self.messages_frame.winfo_children():
                if isinstance(widget, tk.Label):
                    widget.destroy()

            # Display the title (if present) and message content
            if title:
                title_label = tk.Label(self.messages_frame, text=title, fg="black", font=("Arial", 80, "bold"), anchor="center")
                title_label.pack(side=tk.TOP, pady=(10, 5), fill=tk.BOTH)

            message_label = tk.Label(self.messages_frame, text=content, fg="black", font=("Arial", 45), justify="right")
            message_label.pack(pady=5, expand=True, fill=tk.BOTH)

        except FileNotFoundError:
            messagebox.showerror("Error", f"File {file_name} not found.")
        except IndexError as e:
            messagebox.showerror("Error", str(e))
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load message: {e}")

        
    def exit_fullscreen(self):
        if self.user_form is not None:
            self.user_form.attributes("-fullscreen", False)


    def select_prev_step(self):
        # Move to the previous unmarked step in the list
        self.prev_index = self.current_index - 1

        while self.prev_index >= 0:
            prev_bg = self.user_form_execution_steps_listbox.itemcget(self.prev_index, 'bg')
            if prev_bg not in ['green']:  # Skip green and yellow marked steps
                # Remove yellow mark from the current step if it's not green
                if self.current_index >= 0:
                    current_bg = self.user_form_execution_steps_listbox.itemcget(self.current_index, 'bg')
                    if current_bg == 'yellow':
                        self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'white'})
                        self.user_form_execution_steps_listbox.update_idletasks()  # Ensure UI updates immediately

                # Highlight the previous step in yellow
                self.user_form_execution_steps_listbox.itemconfig(self.prev_index, {'bg': 'yellow'})
                # Check if the newly highlighted step starts with "הנחיה:"
                prev_step = self.get_step_from_file(self.prev_index, self.file_name_text_box.get("1.0", "end-1c").strip())  # Retrieve the previous step from the file
                # message_text = prev_step[2:].strip()  # Extract the text after "הנחיה:"
                self.show_message_in_frame(self.prev_index, self.file_name_text_box.get("1.0", "end-1c").strip())
                # Store the index of the currently highlighted step
                self.current_index = self.prev_index

                # Clear the selection to prevent blue highlighting
                self.user_form_execution_steps_listbox.selection_clear(0, tk.END)

                return  # Exit after selecting the previous step
            self.prev_index -= 1

        # If no more unmarked steps are found, keep the current yellow mark
        if self.current_index >= 0:
            current_bg = self.user_form_execution_steps_listbox.itemcget(self.current_index, 'bg')
            if current_bg == 'yellow':
                messagebox.showinfo("Info", "No unmarked steps before this one.")


    def select_next_step(self, jump=False):
        """Move to the next unmarked step in the list and continue the timer."""
        if getattr(self, 'is_jumping', False):  # Skip advancing if a jump is in progress
            self.is_jumping = False  # Reset the flag after skipping
            return

        # Determine the next index
        self.next_index = self.current_index if jump else self.current_index + 1

        while self.next_index < self.user_form_execution_steps_listbox.size():
            next_bg = self.user_form_execution_steps_listbox.itemcget(self.next_index, 'bg')
            if next_bg not in ['green']:  # Skip green marked steps
                # Remove yellow mark from the current step if it's not green
                if self.current_index >= 0:
                    current_bg = self.user_form_execution_steps_listbox.itemcget(self.current_index, 'bg')
                    if current_bg == 'yellow':
                        self.stop_time_count()  # Stop timing for the current step
                        self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'white'})
                        self.user_form_execution_steps_listbox.update_idletasks()

                # Highlight the next step in yellow
                self.user_form_execution_steps_listbox.itemconfig(self.next_index, {'bg': 'yellow'})
                self.user_form_execution_steps_listbox.update_idletasks()

                # Fetch the time for the next step and resume the timer
                try:
                    step_time = float(self.user_form_execution_time_listbox.get(self.next_index))
                    self.start_time = time.time() - step_time  # Resume timing from the stored time
                except ValueError:
                    print(f"[ERROR] Invalid time value for step {self.next_index}")
                except Exception as e:
                    print(f"[ERROR] Failed to resume timer for step {self.next_index}: {e}")

                # Show the message for the selected step
                file_name = self.file_name_text_box.get("1.0", "end-1c").strip()
                self.show_message_in_frame(self.next_index, file_name)

                # Store the index of the currently highlighted item
                self.current_index = self.next_index

                # Execute the step logic if needed
                next_step = self.user_form_execution_steps_listbox.get(self.next_index)   
                if next_step.startswith("רצף:"):
                    self.master.after(0, self.user_form_execute_step)                      
                elif next_step.startswith("הנחיה:"):
                    self.update_time_count()  # Continue updating time
                return  # Exit after selecting the next step
            self.next_index += 1

        # If no more unmarked steps are found
        if self.current_index >= 0:
            current_bg = self.user_form_execution_steps_listbox.itemcget(self.current_index, 'bg')
            if current_bg == 'yellow':
                messagebox.showinfo("Info", "All steps have been set or completed.")


    def get_step_from_file(self, index, file_path):
        """Retrieve a specific line from a text file based on the provided index."""
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                lines = file.readlines()
                if index < 0 or index >= len(lines):
                    raise IndexError("Index out of range.")
                return lines[index].strip()
        except FileNotFoundError:
            messagebox.showerror("Error", f"File {file_path} not found.")
        except IndexError as e:
            messagebox.showerror("Error", str(e))
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load step: {e}")
            return None


    def reset_steps_states(self):
        # Reset the current index to start from the first sequence
        self.current_index = -1

        # Remove yellow marks from all items
        for i in range(self.user_form_execution_steps_listbox.size()):
            self.user_form_execution_steps_listbox.itemconfig(i, {'bg': 'white'})

        # Clear the content of the Execution_sequence_frame
        self.Execution_sequence_waypoints.delete("1.0", tk.END)

        
    def on_user_form_close(self):
        """Handle the user form closing event and save its state."""
        # Save the current state to a file
        state = {
            "current_index": self.current_index,
            "execution_text": self.Execution_sequence_waypoints.get("1.0", tk.END).strip()
        }
        
        with open("user_form_state.json", "w") as f:
            json.dump(state, f)

        # Close the user form
        self.user_form.destroy()
        self.user_form = None


    def move_item_up(self):
        if self.waypoints_listbox.curselection():
            self.move_in_listbox(self.waypoints_listbox, self.waypoints, self.descriptions, direction='up')
        elif self.basic_sequence_listbox.curselection():
            self.move_in_listbox(self.basic_sequence_listbox, self.basic_sequences_list, None, direction='up')
        elif self.tasks_sequences_listbox.curselection():
            self.move_in_listbox(self.tasks_sequences_listbox, self.tasks_sequences_list, None, direction='up')


    def move_item_down(self):
        if self.waypoints_listbox.curselection():
            self.move_in_listbox(self.waypoints_listbox, self.waypoints, self.descriptions, direction='down')
        elif self.basic_sequence_listbox.curselection():
            self.move_in_listbox(self.basic_sequence_listbox, self.basic_sequences_list, None, direction='down')
        elif self.tasks_sequences_listbox.curselection():
            self.move_in_listbox(self.tasks_sequences_listbox, self.tasks_sequences_list, None, direction='down')


    def move_in_listbox(self, listbox, data_list, aux_list=None, direction='up'):
        selected_index = listbox.curselection()
        if selected_index:
            index = selected_index[0]
            if direction == 'up' and index > 0:
                # Swap items in the data list (and aux list if provided)
                data_list[index], data_list[index - 1] = data_list[index - 1], data_list[index]
                if aux_list:
                    aux_list[index], aux_list[index - 1] = aux_list[index - 1], aux_list[index]
                self.update_waypoints_listbox(listbox, data_list)
                listbox.selection_set(index - 1)
            elif direction == 'down' and index < len(data_list) - 1:
                # Swap items in the data list (and aux list if provided)
                data_list[index], data_list[index + 1] = data_list[index + 1], data_list[index]
                if aux_list:
                    aux_list[index], aux_list[index + 1] = aux_list[index + 1], aux_list[index]
                self.update_waypoints_listbox(listbox, data_list)
                listbox.selection_set(index + 1)


    # def update_waypoints_listbox(self, listbox, data_list):
    #     listbox.delete(0, tk.END)
    #     if listbox == self.waypoints_listbox:
    #         for i, description in enumerate(self.descriptions):
    #             listbox.insert(tk.END, f"{i + 1}: {description}")
    #     elif listbox == self.basic_sequence_listbox:
    #         for idx, (name, _) in enumerate(self.basic_sequences_list):
    #             listbox.insert(tk.END, f"{idx + 1}. {name}")
    #     elif listbox == self.tasks_sequences_listbox:
    #         for idx, (name, _) in enumerate(self.tasks_sequences_list):
    #             listbox.insert(tk.END, f"{idx + 1}. {name}")

    def update_waypoints_listbox(self):
        """
        Refresh the UI listbox to display the available waypoints with their index numbers.
        """
        if not hasattr(self, "waypoints_listbox"):
            print("[ERROR] Waypoints listbox is not initialized.")
            return

        self.waypoints_listbox.delete(0, tk.END)  # Clear previous items

        for index, wp in enumerate(self.waypoints, start=1):  # ✅ Add waypoint numbers
            if "description" in wp:
                display_text = f"{index}. {wp['description']}"  # Format: 1. Description
                self.waypoints_listbox.insert(tk.END, display_text)
            else:
                print(f"[WARNING] Waypoint missing description: {wp}")

        print("[DEBUG] Waypoints UI updated successfully.")



    def toggle_append_mode(self):
        self.append_mode = not self.append_mode
        self.append_mode_button.config(text=f"Append Mode {'is ON' if self.append_mode else 'is OFF'}")


    def on_waypoint_click(self, event=None):
        selected_index = self.waypoints_listbox.curselection()
        if selected_index:
            self.edit_waypoint_button.config(state=tk.NORMAL)
            index = selected_index[0]
            if self.delete_mode:
                self.delete_waypoint(index)
                self.delete_mode = False  # Reset delete mode after deletion
            else:
                self.add_waypoint_to_basic_sequence(event)  # Handle normal selection


    def edit_waypoint_description(self, event):
        selected_index = self.waypoints_listbox.curselection()
        if selected_index:
            index = selected_index[0]
            current_description = self.descriptions[index]
            new_description = simpledialog.askstring("Edit Waypoint Description", 
                                                    "Enter new description:", 
                                                    initialvalue=current_description)
            if new_description is not None:
                self.descriptions[index] = new_description
                self.update_hmi_with_waypoints()  # Refresh the Listbox with the new description
                self.save_waypoints_to_file()  # Save the updated descriptions to file
                messagebox.showinfo("Info", "Waypoint description updated successfully.")
    
    
    def add_waypoint_to_basic_sequence(self, event=None):
        selected_index = self.waypoints_listbox.curselection()
        if selected_index:
            index = selected_index[0]
            current_content = self.Execution_sequence_waypoints.get("1.0", tk.END).strip()

            if current_content:
                # Convert current content to a list
                sequence_items = current_content.split(',')
                sequence_items = [item.strip() for item in sequence_items]
            else:
                sequence_items = []

            # Append the new waypoint
            sequence_items.append(str(index + 1))

            # Update the Sequence_frame
            sequence_text = ", ".join(sequence_items)
            self.Execution_sequence_waypoints.delete("1.0", tk.END)
            self.Execution_sequence_waypoints.insert("1.0", sequence_text)

            # Update button states
            self.check_waypoint_entry()
            self.update_basic_sequence_button.config(state=tk.NORMAL)


    def update_waypoint(self, waypoint_index):
        if self.robot:
            current_position = self.robot.getj()  # Get current robot joint positions
            confirm = messagebox.askyesno(
                "Confirm Update",
                f"Are you sure you want to update Waypoint {waypoint_index + 1} to the current robot position?"
            )
            if confirm:
                self.waypoints[waypoint_index] = current_position
                self.save_waypoints_to_file()  # Save the updated waypoints immediately
                self.update_hmi_with_waypoints()  # Update the UI to reflect the changes
                messagebox.showinfo("Info", f"Updated Waypoint {waypoint_index + 1}.")


    def enter_update_sequence_mode(self):
        self.update_sequence_mode = True  # Activate update mode
        messagebox.showinfo("Update Mode", "Update mode activated. Please select a sequence to update.")


    def enter_delete_sequence_mode(self):
        self.delete_sequence_mode = True  # Activate delete mode
        messagebox.showinfo("Delete Mode", "Delete mode activated. Please select a sequence to delete.")


    def move_robot(self, direction):
        """Move the robot in the specified direction."""
        if self.robot:
            try:
                distance = float(self.distance_entry.get()) / 1000  # Convert mm to meters
                pose = self.robot.get_pose()
                
                if direction == "up":
                    pose.pos.y += distance
                elif direction == "down":
                    pose.pos.y -= distance
                elif direction == "left":
                    pose.pos.x -= distance
                elif direction == "right":
                    pose.pos.x += distance
                elif direction == "forward":
                    pose.pos.z += distance
                elif direction == "backward":
                    pose.pos.z -= distance

                self.robot.set_pose(pose, acc=1, vel=0.2)
            except Exception as e:
                messagebox.showerror("Error", f"Failed to move robot: {e}")
        else:
            messagebox.showwarning("Warning", "No robot connected!")


    def open_gripper(self):
        """Send command to open the tool."""
        if self.robot:
            try:
                self.robot.send_program("set_tool_digital_out(0, False)")
                time.sleep(1)
            except Exception as e:
                messagebox.showerror("Error", f"Failed to open tool: {e}")


    def close_gripper(self):
        """Send command to close the tool."""
        if self.robot:
            try:
                self.robot.send_program("set_tool_digital_out(0, True)")
                time.sleep(1)
            except Exception as e:
                messagebox.showerror("Error", f"Failed to close tool: {e}")


    def check_waypoint_entry(self, event=None):
        """Enable or disable buttons based on the content of Sequence_frame."""
        content = self.Execution_sequence_waypoints.get("1.0", tk.END).strip()  # Get content and remove leading/trailing whitespace
        if content:
            self.clear_sequence_button.config(state=tk.NORMAL)
            self.add_basic_sequence_button.config(state=tk.NORMAL)
            self.execute_button.config(state=tk.NORMAL)
            if self.basic_sequence_listbox.curselection():
                self.update_basic_sequence_button.config(state=tk.NORMAL)
        else:
            self.clear_sequence_button.config(state=tk.DISABLED)
            self.add_basic_sequence_button.config(state=tk.DISABLED)
            self.execute_button.config(state=tk.DISABLED)
            self.update_basic_sequence_button.config(state=tk.DISABLED)


    def update_speed_value(self, value):
        self.speed_value_label.config(text=f"{float(value):.2f}")

    def update_acc_value(self, value):
        self.acc_value_label.config(text=f"{float(value):.2f}")

    def auto_connect(self):
        self.show_message("Connecting to robot...")
        self.connect_robot()


    def show_message(self, message):
        self.message_label = tk.Label(self.master, text=message, fg="blue")
        self.message_label.pack(pady=10)


    def clear_message(self):
        if hasattr(self, 'message_label') and self.message_label:
            self.message_label.pack_forget()
            self.message_label.destroy()


    def connect_robot(self):
        try:
            self.robot = urx.Robot(self.robot_ip)
            self.reset_protective_stop()
            self.clear_message()
            self.check_waypoint_entry()
            messagebox.showinfo("Info", f"Connected to UR5 robot at {self.robot_ip}")
            self.connect_button.config(state=tk.DISABLED)
            self.disconnect_button.config(state=tk.NORMAL)
            self.add_waypoint_button.config(state=tk.NORMAL)
            self.delete_waypoint_button.config(state=tk.NORMAL)
            self.edit_waypoint_button.config(state=tk.NORMAL)
        except Exception as e:
            self.clear_message()
            messagebox.showerror("Error", f"Failed to connect to robot: {e}")
            self.master.after(3000, self.auto_connect)  # Retry after 3 seconds


    def disconnect_robot(self):
        if self.robot:
            self.robot.close()
            self.robot = None
            self.connect_button.config(state=tk.NORMAL)
            self.disconnect_button.config(state=tk.DISABLED)
            self.execute_button.config(state=tk.DISABLED)
            self.add_waypoint_button.config(state=tk.DISABLED)
            self.delete_waypoint_button.config(state=tk.DISABLED)
            self.edit_waypoint_button.config(state=tk.DISABLED)


    def reset_protective_stop(self):
        if self.robot:
            script = """
            def clear_protective_stop():
                textmsg("Clearing protective stop")
                textmsg("Sending stopl")
                stopl(1.0)
                textmsg("Sending stopl done")
            end
            clear_protective_stop()
            """
            self.robot.send_program(script)


    # def move_to_waypoint(self, waypoint):
    #     if self.robot:
    #         speed = self.speed_scale.get()  # Get current speed from the slider
    #         acc_ = self.acc_scale.get()  # Get current acc from the slider
    #         print(f"Moving to target waypoint: {waypoint} with speed: {speed} and acc: {acc_}")
    #         self.robot.movej(waypoint, acc=acc_, vel=speed)
    #         self.wait_until_idle(waypoint)
    #         # print("Moved to joint waypoint")

    def move_to_waypoint(self, waypoint_name):
        """
        Moves the robot to a given waypoint with specific speed and acceleration.
        """
        print(f"[DEBUG] Searching for waypoint by description: {waypoint_name}")

        # Ensure waypoints are loaded as dictionaries
        if not self.waypoints or not isinstance(self.waypoints[0], dict):
            print("[ERROR] Waypoints list is not properly loaded as dictionaries.")
            return

        # Find the waypoint by description
        waypoint_data = next((wp for wp in self.waypoints if wp["description"] == waypoint_name), None)

        if waypoint_data:
            waypoint = waypoint_data["waypoint"]
            speed = waypoint_data.get("speed", 5)
            acceleration = waypoint_data.get("acceleration", 5)

            print(f"[DEBUG] Moving to {waypoint_name} with speed {speed} and acceleration {acceleration}")

            if self.robot:          
                self.robot.movej(waypoint, acc=acceleration, vel=speed)
                self.wait_until_idle(waypoint)
        else:
            print(f"[ERROR] Waypoint '{waypoint_name}' not found in waypoints data.")




    def wait_until_idle(self, target_waypoint, threshold=0.01, timeout=30, stable_count_threshold=3):
        # print("Waiting for robot to reach the target waypoint...")
        start_time = time.time()
        consecutive_stable_count = 0
        while True:
            current_position = self.robot.getj()  # Get current joint positions
            distance = sum((c - t) ** 2 for c, t in zip(current_position, target_waypoint)) ** 0.5
            if distance < threshold:
                consecutive_stable_count += 1
                if consecutive_stable_count >= stable_count_threshold:
                    break
            else:
                consecutive_stable_count = 0

            if time.time() - start_time > timeout:
                # print("Timeout reached. Exiting wait loop.")
                break
            time.sleep(0.1)


    def execute_action(self, suppress_messages=False, mark_steps=False):
        self.suppress_messages = suppress_messages  # Set the flag based on the parameter
        self.mark_sequences = mark_steps

        # Get the sequence to execute
        entry_text = self.Execution_sequence_waypoints.get("1.0", tk.END).strip()
        sequence_items = entry_text.split(',')

        sequence_indices = []
        try:
            for item in sequence_items:
                item = item.strip()
                if item.isdigit():
                    sequence_indices.append(int(item) - 1)
                elif item == "O" or item == "C":
                    sequence_indices.append(item)
                else:
                    raise ValueError("Invalid sequence item")
            
            if all(isinstance(i, int) and 0 <= i < len(self.waypoints) or isinstance(i, str) for i in sequence_indices):
                self.sequence = sequence_indices
                
                # Mark the current sequence in orange (running) before execution
                if self.mark_sequences:
                    if self.current_index >= 0:
                        self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'orange'})
                        self.user_form_execution_steps_listbox.update_idletasks()  # Ensure UI updates immediately

                # Execute the sequence
                self.execute_sequence()

                # Mark the sequence in green (completed) after execution
                if self.mark_sequences:
                    if self.current_index >= 0:
                        self.user_form_execution_steps_listbox.itemconfig(self.current_index, {'bg': 'green'})
                        self.user_form_execution_steps_listbox.update_idletasks()  # Ensure UI updates immediately

                    # Automatically select and mark the next sequence in yellow
                    self.master.after(500, self.select_next_step())

            else:
                if not self.suppress_messages:
                    messagebox.showerror("Error", "One or more waypoint indices are invalid.")
        except ValueError:
            if not self.suppress_messages:
                messagebox.showerror("Error", "Please enter valid waypoint numbers and commands ('O' for Open, 'C' for Close) separated by commas.")


    def update_sequence_display(self, append_mode=False):
        sequence_display = []
        for item in self.sequence:
            sequence_display.append(str(item + 1) if isinstance(item, int) else item)  # Convert integer waypoints to 1-based index strings or keep the command as is

        self.Execution_sequence_waypoints.delete("1.0", tk.END)  # Clear the Text widget
        if sequence_display:
            self.Execution_sequence_waypoints.insert("1.0", ", ".join(sequence_display))  # Insert only the numbers and commands
        self.check_waypoint_entry()


    def add_sequence(self):
        sequence_content = self.Execution_sequence_waypoints.get("1.0", tk.END).strip()
        if sequence_content:
            try:
                sequence_indices = [
                    int(item.strip()) - 1 if item.strip().isdigit() else item.strip()
                    for item in sequence_content.split(',')
                ]
                sequence_description = simpledialog.askstring("Sequence Description", "Enter description for this sequence:")
                self.basic_sequences_list.append((sequence_description, sequence_indices))
                self.basic_sequence_listbox.insert(tk.END, f"{len(self.basic_sequences_list)}. {sequence_description}")
                self.clear_sequence()
                self.save_basic_sequences_to_file()  # Save sequences after adding a new one
                messagebox.showinfo("Info", f"New sequence '{sequence_description}' added successfully.")
            except ValueError:
                messagebox.showerror("Error", "Invalid sequence data. Please enter valid waypoint numbers and commands ('O' for Open, 'C' for Close) separated by commas.")
        else:
            messagebox.showerror("Error", "No sequence data to add.")


    def add_waypoint(self):
        if self.robot:
            current_position = self.robot.getj()  # Get current robot joint positions
            description = simpledialog.askstring("Waypoint Description", "Enter description for this waypoint:")
            if description is not None:
                self.waypoints.append(current_position)
                self.descriptions.append(description)  # Store description as string
                self.update_hmi_with_waypoints()
                self.save_waypoints_to_file()
                messagebox.showinfo("Info", "Waypoint added successfully.")


    def update_hmi_with_waypoints(self):
        self.waypoints_listbox.delete(0, tk.END)
        for i, description in enumerate(self.descriptions):
            self.waypoints_listbox.insert(tk.END, f"{i + 1}: {description}")


    def edit_basic_sequence_description(self, event):
        selected_index = self.basic_sequence_listbox.curselection()
        if selected_index:
            current_name = self.basic_sequences_list[selected_index[0]][0]
            new_name = simpledialog.askstring("Edit Sequence Name", "Enter new sequence name:", initialvalue=current_name)
            if new_name:
                self.basic_sequences_list[selected_index[0]] = (new_name, self.basic_sequences_list[selected_index[0]][1])
                self.update_basic_sequence_listbox()
                self.save_basic_sequences_to_file()  # Save sequences after editing a sequence name


    def edit_tasks_sequences_description(self, event):
        selected_index = self.tasks_sequences_listbox.curselection()
        if selected_index:
            current_name = self.tasks_sequences_list[selected_index[0]][0]
            new_name = simpledialog.askstring("Edit Sequence Name", "Enter new sequence name:", initialvalue=current_name)
            if new_name:
                self.tasks_sequences_list[selected_index[0]] = (new_name, self.tasks_sequences_list[selected_index[0]][1])
                self.update_tasks_sequences_listbox()
                self.save_tasks_sequences_to_file()  # Save sequences after editing a sequence name


    def update_basic_sequence_listbox(self):
        self.basic_sequence_listbox.delete(0, tk.END)
        for idx, (name, _) in enumerate(self.basic_sequences_list):
            self.basic_sequence_listbox.insert(tk.END, f"{idx + 1}. {name}")


    def add_seq_open_tool(self):
        """Add 'O' to the sequence, representing an open tool command."""
        current_content = self.Execution_sequence_waypoints.get("1.0", tk.END).strip()
        sequence_text = f"{current_content}, O" if current_content else "O"
        self.Execution_sequence_waypoints.delete("1.0", tk.END)
        self.Execution_sequence_waypoints.insert("1.0", sequence_text)
        self.check_waypoint_entry()  # Update button states


    def add_seq_close_tool(self):
        """Add 'C' to the sequence, representing a close tool command."""
        current_content = self.Execution_sequence_waypoints.get("1.0", tk.END).strip()
        sequence_text = f"{current_content}, C" if current_content else "C"
        self.Execution_sequence_waypoints.delete("1.0", tk.END)
        self.Execution_sequence_waypoints.insert("1.0", sequence_text)
        self.check_waypoint_entry()  # Update button states


    def execute_sequence(self):
        """Execute the current sequence of waypoints and tool commands."""
        for item in self.sequence:
            if isinstance(item, int):
                self.move_to_waypoint(self.waypoints[item]["description"])
            elif item == "O":
                self.open_gripper()
            elif item == "C":
                self.close_gripper()
        if not self.suppress_messages:
            messagebox.showinfo("Info", "Sequence execution completed.")
        self.suppress_messages = False


    def display_basic_sequence_waypoints(self):
        selected_index = self.basic_sequence_listbox.curselection()
        if selected_index:
            sequence_name, sequence = self.basic_sequences_list[selected_index[0]]

            if self.delete_sequence_mode:
                # Handle delete mode
                confirm = messagebox.askyesno(
                    "Confirm Deletion",
                    f"Are you sure you want to delete the sequence '{sequence_name}'?"
                )
                if confirm:
                    del self.basic_sequences_list[selected_index[0]]
                    self.update_basic_sequence_listbox()
                    self.save_basic_sequences_to_file()
                    messagebox.showinfo("Info", f"Sequence '{sequence_name}' deleted successfully.")
                self.delete_sequence_mode = False
                return

            elif self.update_sequence_mode:
                # Handle update mode
                _, selected_sequence = self.basic_sequences_list[selected_index[0]]
                self.sequence.extend(selected_sequence)
                new_sequence_content = self.Execution_sequence_waypoints.get("1.0", tk.END).strip()
                try:
                    sequence_indices = [
                        int(i.strip()) - 1 if i.strip().isdigit() else i.strip()
                        for i in new_sequence_content.split(',')
                    ]
                    confirm = messagebox.askyesno(
                        "Confirm Update",
                        f"Are you sure you want to update the sequence '{sequence_name}' with the new sequence?"
                    )
                    if confirm:
                        self.basic_sequences_list[selected_index[0]] = (sequence_name, sequence_indices)
                        self.update_basic_sequence_listbox()
                        self.save_basic_sequences_to_file()
                        messagebox.showinfo("Info", f"Sequence '{sequence_name}' updated successfully.")
                except ValueError:
                    messagebox.showerror("Error", "Please enter valid waypoint numbers and commands ('O' for Open, 'C' for Close) separated by commas.")
                
                self.update_sequence_display()               
                self.update_sequence_mode = False
                return

            elif self.append_mode:
                # Handle append mode
                current_content = self.Execution_sequence_waypoints.get("1.0", tk.END).strip()
                new_content = ", ".join(
                    str(index + 1) if isinstance(index, int) else index for index in sequence
                )
                if current_content:
                    sequence_text = f"{current_content}, {new_content}"
                else:
                    sequence_text = new_content
            else:
                # Replace content if not in append mode or update mode
                sequence_text = ", ".join(
                    str(index + 1) if isinstance(index, int) else index for index in sequence
                )

            self.Execution_sequence_waypoints.delete("1.0", tk.END)
            self.Execution_sequence_waypoints.insert("1.0", sequence_text)

            self.delete_basic_sequence_button.config(state=tk.NORMAL)
            self.update_basic_sequence_button.config(state=tk.NORMAL)
            
            self.check_waypoint_entry()  # Check and update button states after selecting a sequence
        else:
            self.delete_basic_sequence_button.config(state=tk.DISABLED)


    def clear_sequence(self):
        self.sequence = []
        self.single_button_pressed = False
        self.update_sequence_display()


    def save_waypoints_to_file(self):
        waypoints_with_descriptions = [
            {"waypoint": pos, "description": desc} 
            for pos, desc in zip(self.waypoints, self.descriptions)
        ]

        file_path = os.path.join(self.txtFilesFolder, "waypoints.txt")  # Save to waypoints.txt by default
        with open(file_path, 'w') as file:
            json.dump(waypoints_with_descriptions, file)


    # def load_waypoints_from_file(self, file_path=None):
    #     if not file_path:
    #         file_path = filedialog.askopenfilename(defaultextension=".txt", filetypes=[("Text files", "*.txt")])

    #     if file_path:
    #         try:
    #             with open(file_path, 'r') as file:
    #                 waypoints_with_descriptions = json.load(file)
    #             self.waypoints = []
    #             self.descriptions = []
                
    #             for data in waypoints_with_descriptions:
    #                 self.waypoints.append(data["waypoint"])
    #                 self.descriptions.append(data["description"])
    #             self.update_hmi_with_waypoints()  # Update the HMI with loaded waypoints
    #         except Exception as e:
    #             messagebox.showerror("Error", f"Failed to load waypoints: {e}")

    def load_waypoints_from_file(self, file_path=None):
        if not file_path:
            file_path = os.path.join(self.txtFilesFolder, "waypoints.txt")

        try:
            with open(file_path, 'r') as file:
                waypoints_data = json.load(file)

            self.waypoints = []  # Clear previous waypoints

            for wp in waypoints_data:
                if isinstance(wp, dict):  # ✅ Ensure waypoint is a dictionary
                    self.waypoints.append(wp)
                else:
                    print(f"[WARNING] Skipping invalid waypoint entry: {wp}")

            print(f"[DEBUG] Successfully loaded {len(self.waypoints)} waypoints.")
            print(f"[DEBUG] Waypoints List: {self.waypoints}")

            # After loading, update the UI
            self.update_waypoints_listbox()

        except Exception as e:
            print(f"[ERROR] Failed to load waypoints: {e}")



    def save_basic_sequences_to_file(self):
        sequences_list_to_save = [
            (name, [i + 1 if isinstance(i, int) else i for i in indices]) 
            for name, indices in self.basic_sequences_list
        ]
        
        file_path = os.path.join(self.txtFilesFolder, "basic_sequences.txt")
        with open(file_path, 'w') as file:
            json.dump(sequences_list_to_save, file)
        messagebox.showinfo("Info", f"Sequences saved to {file_path}")


    def load_basic_sequences_from_file(self, file_path=None):
        if not file_path:
            file_path = os.path.join(self.txtFilesFolder, "basic_sequences.txt")

        if file_path:
            try:
                with open(file_path, 'r') as file:
                    basic_sequences_list_raw = json.load(file)
                self.basic_sequences_list = [
                    (name, [(i - 1) if isinstance(i, int) else i for i in indices]) 
                    for name, indices in basic_sequences_list_raw
                ]
                self.update_basic_sequence_listbox()  # Update the sequence listbox with loaded sequences
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load basic sequences: {e}")


    def enter_delete_mode(self):
        self.delete_mode = True
        messagebox.showinfo("Delete Mode", "Select a waypoint to delete.")


    def delete_waypoint(self, waypoint_index):
        if waypoint_index is not None and 0 <= waypoint_index < len(self.waypoints):
            # Remove the waypoint and its description
            del self.waypoints[waypoint_index]
            del self.descriptions[waypoint_index]

            # Update the HMI to reflect the changes
            self.update_hmi_with_waypoints()

            # Update sequences that might reference this waypoint
            self.update_basic_sequences_after_deletion(waypoint_index)

            # Save the updated waypoints to the file
            self.save_waypoints_to_file()

            # Show a confirmation message
            messagebox.showinfo("Info", f"Waypoint {waypoint_index + 1} deleted successfully.")
        else:
            messagebox.showerror("Error", "Invalid selection for deletion.")


    def update_basic_sequences_after_deletion(self, deleted_index):
        for seq_name, seq in self.basic_sequences_list:
            updated_seq = []
            for i in seq:
                if isinstance(i, int):
                    if i < deleted_index:
                        updated_seq.append(i)
                    elif i > deleted_index:
                        updated_seq.append(i - 1)
                else:
                    updated_seq.append(i)
            
            seq[:] = updated_seq

        updated_current_sequence = []
        for i in self.sequence:
            if isinstance(i, int):
                if i < deleted_index:
                    updated_current_sequence.append(i)
                elif i > deleted_index:
                    updated_current_sequence.append(i - 1)
            else:
                updated_current_sequence.append(i)
        
        self.sequence = updated_current_sequence
        self.update_sequence_display()
        self.save_basic_sequences_to_file()


    def edit_selected_waypoint(self, event=None):
        selected_index = self.waypoints_listbox.curselection()
        if selected_index:
            self.update_mode = True  # Activate update mode
            index = selected_index[0]
            self.finalize_waypoint_update(index)  # Directly proceed to updating the selected waypoint
        else:
            messagebox.showerror("Error", "Please select a waypoint to update.")


    def finalize_waypoint_update(self, waypoint_index):
        if self.robot:
            try:
                current_position = self.robot.getj()  # Get current robot joint positions
                confirm = messagebox.askyesno(
                    "Confirm Update",
                    f"Are you sure you want to update Waypoint {waypoint_index + 1} to the current robot position?"
                )
                if confirm:
                    # print(f"Updating Waypoint {waypoint_index + 1} to {current_position}")
                    self.waypoints[waypoint_index] = current_position
                    self.save_waypoints_to_file()  # Save the updated waypoints immediately
                    self.update_hmi_with_waypoints()  # Update the UI to reflect the changes
                    messagebox.showinfo("Info", f"Updated Waypoint {waypoint_index + 1}.")
                else:
                    # print("Update canceled by user.")
                    messagebox.showinfo("Info", "Update canceled.")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to update waypoint: {e}")
            finally:
                self.update_mode = False  # Exit update mode after updating


    def exit_app(self):
        # Close the video process if it's running
        if self.video_process is not None:
            self.video_process.terminate()
            try:
                self.video_process.wait(timeout=5)  # Wait for up to 5 seconds for graceful termination
            except subprocess.TimeoutExpired:
                print("Process did not terminate in time, forcing kill.")
                self.video_process.kill()  # Forcefully kill the process
                self.video_process.wait()  # Wait for the kill to complete
        
        # Close the user form if it's open
        if self.user_form is not None and self.user_form.winfo_exists():
            self.user_form.destroy()
        
        # Stop the digital input monitoring thread
        self.running = False
        if self.input_thread.is_alive():
            self.input_thread.join()  # Ensure the monitoring thread stops cleanly
        
        # Close the main application window
        self.master.quit()  # This will stop the main Tkinter loop
        self.master.destroy()  # This will close the main window and release resources

        # Explicitly exit the program
        sys.exit(0)



    def load_tasks_sequences_from_file(self, file_path=None):
        if not file_path:
            file_path = filedialog.askopenfilename(defaultextension=".txt", filetypes=[("Text files", "*.txt")])

        if file_path:
            try:
                with open(file_path, 'r') as file:
                    tasks_sequences_list_raw = json.load(file)
                self.tasks_sequences_list = [
                    (name, [(i - 1) if isinstance(i, int) else i for i in indices])
                    for name, indices in tasks_sequences_list_raw
                ]
                self.update_tasks_sequences_listbox()  # Update the listbox with loaded sequences
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load sequences: {e}")


    def save_tasks_sequences_to_file(self):
        tasks_sequence_list_to_save = [
            (name, [(i + 1) if isinstance(i, int) else i for i in sequence])
            for name, sequence in self.tasks_sequences_list
        ]

        file_path = os.path.join(self.txtFilesFolder, "tasks_sequences.txt")
        with open(file_path, 'w') as file:
            json.dump(tasks_sequence_list_to_save, file)
        messagebox.showinfo("Info", f"Sequences of sequences saved to {file_path}")


    def enter_delete_task_sequence_mode(self):
        self.delete_tasks_sequences_mode = True  # Activate delete mode for sequence of sequences
        messagebox.showinfo("Delete Mode", "Delete mode activated. Please select a sequence of sequences to delete.")


    def enter_update_task_sequence_mode(self):
        self.update_tasks_sequences_mode = True  # Activate update mode for sequence of sequences
        messagebox.showinfo("Update Mode", "Update mode activated. Please select a sequence of sequences to update.")


    def add_task_sequence(self):
        sequence_content = self.Execution_sequence_waypoints.get("1.0", tk.END).strip()
        if sequence_content:
            try:
                sequence_indices = [
                    int(item.strip()) - 1 if item.strip().isdigit() else item.strip()
                    for item in sequence_content.split(',')
                ]
                sequence_description = simpledialog.askstring("Sequence Description", "Enter description for this sequence:")
                self.tasks_sequences_list.append((sequence_description, sequence_indices))
                self.tasks_sequences_listbox.insert(tk.END, f"{len(self.tasks_sequences_list)}. {sequence_description}")
                self.clear_sequence()
                self.save_tasks_sequences_to_file()  # Save sequence of sequences after adding a new one
                messagebox.showinfo("Info", f"New task_sequence '{sequence_description}' added successfully.")
            except ValueError:
                messagebox.showerror("Error", "Invalid sequence data. Please enter valid sequence numbers and commands separated by commas.")
        else:
            messagebox.showerror("Error", "No task_sequence data to add.")


    def update_tasks_sequences_display(self, append_mode=False):
        sequence_display = []
        for item in self.sequence:
            sequence_display.append(str(item + 1) if isinstance(item, int) else item)  # Convert integer waypoints to 1-based index strings or keep the command as is

        self.Execution_sequence_waypoints.delete("1.0", tk.END)  # Clear the Text widget
        if sequence_display:
            self.Execution_sequence_waypoints.insert("1.0", ", ".join(sequence_display))  # Insert only the numbers and commands
        self.check_waypoint_entry()


    def display_tasks_sequences_waypoints(self):
        selected_index = self.tasks_sequences_listbox.curselection()
        if selected_index:
            sequence_name, sequence = self.tasks_sequences_list[selected_index[0]]

            if self.delete_tasks_sequences_mode:
                # Handle delete mode for sequences of sequences
                confirm = messagebox.askyesno(
                    "Confirm Deletion",
                    f"Are you sure you want to delete the sequence of sequences '{sequence_name}'?"
                )
                if confirm:
                    del self.tasks_sequences_list[selected_index[0]]
                    self.update_tasks_sequences_listbox()
                    self.save_tasks_sequences_to_file()
                    messagebox.showinfo("Info", f"Sequence of sequences '{sequence_name}' deleted successfully.")
                self.delete_tasks_sequences_mode = False
                return

            elif self.update_tasks_sequences_mode:
                # Handle update mode
                _, selected_tasks_sequences = self.tasks_sequences_list[selected_index[0]]
                self.tasks_sequences.extend(selected_tasks_sequences)
                new_sequence_content = self.Execution_sequence_waypoints.get("1.0", tk.END).strip()
                try:
                    sequence_indices = [
                        int(i.strip()) - 1 if i.strip().isdigit() else i.strip()
                        for i in new_sequence_content.split(',')
                    ]
                    confirm = messagebox.askyesno(
                        "Confirm Update",
                        f"Are you sure you want to update the sequence of sequences '{sequence_name}' with the new sequence?"
                    )
                    if confirm:
                        self.tasks_sequences_list[selected_index[0]] = (sequence_name, sequence_indices)
                        self.update_tasks_sequences_listbox()
                        self.save_tasks_sequences_to_file()
                        messagebox.showinfo("Info", f"Sequence of sequences '{sequence_name}' updated successfully.")
                except ValueError:
                    messagebox.showerror("Error", "Please enter valid sequence numbers and commands separated by commas.")

                self.update_tasks_sequences_display()                
                self.update_tasks_sequences_mode = False
                return

            elif self.append_mode:
                # Handle append mode
                current_content = self.Execution_sequence_waypoints.get("1.0", tk.END).strip()
                new_content = ", ".join(
                    str(index + 1) if isinstance(index, int) else index for index in sequence
                )
                if current_content:
                    sequence_text = f"{current_content}, {new_content}"
                else:
                    sequence_text = new_content
            else:
                # Replace content if not in append mode or update mode
                sequence_text = ", ".join(
                    str(index + 1) if isinstance(index, int) else index for index in sequence
                )

            self.Execution_sequence_waypoints.delete("1.0", tk.END)
            self.Execution_sequence_waypoints.insert("1.0", sequence_text)

            self.delete_tasks_sequences_button.config(state=tk.NORMAL)
            self.update_tasks_sequences_button.config(state=tk.NORMAL)
            
            self.check_waypoint_entry()  # Check and update button states after selecting a sequence
        else:
            self.delete_tasks_sequences_button.config(state=tk.DISABLED)


    def update_tasks_sequences_listbox(self):
        # Clear the existing items in the sequence of sequences listbox
        self.tasks_sequences_listbox.delete(0, tk.END)
        
        # Add updated sequences to the listbox
        for idx, (name, _) in enumerate(self.tasks_sequences_list):
            self.tasks_sequences_listbox.insert(tk.END, f"{idx + 1}. {name}")

        # Disable buttons at startup
        self.delete_tasks_sequences_button.config(state=tk.DISABLED)
        # self.update_tasks_sequences_button.config(state=tk.DISABLED)


    def on_closing(self):
        # Stop all camera threads
        # for sn in self.camera_running.keys():
        #     self.camera_running[sn] = False

        # Wait for threads to finish
        # for sn, thread in self.camera_threads.items():
        #     thread.join(timeout=1)

        self.master.destroy()

def main():
    root = tk.Tk()
    app = URRobotApp(root)

    # Handle closing the application gracefully
    root.protocol("WM_DELETE_WINDOW", app.on_closing)

    root.mainloop()

if __name__ == "__main__":
    main()