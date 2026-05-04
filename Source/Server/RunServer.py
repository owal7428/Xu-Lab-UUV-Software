import sys
import os
import csv
import time
import signal
import subprocess
import multiprocessing as mp
import argparse
from datetime import datetime

from NetworkServer import *
from MCUCom import *

WIDTH = 1280
HEIGHT = 720
FPS = 30
BITRATE = 4_000_000

# Create SensorData directory where this script lives on disk
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(BASE_DIR, "SensorData")
os.makedirs(DATA_DIR, exist_ok=True)

# Setup sensor data files

# --- Accelerometer ---
accel_path = os.path.join(DATA_DIR, "accelerometer.csv")
if not os.path.isfile(accel_path):
    with open(accel_path, "w", newline="") as f:
        csv.writer(f).writerow(["timestamp", "acc_x", "acc_y", "acc_z"])

# --- Gyroscope ---
gyro_path = os.path.join(DATA_DIR, "gyroscope.csv")
if not os.path.isfile(gyro_path):
    with open(gyro_path, "w", newline="") as f:
        csv.writer(f).writerow(["timestamp", "gyro_x", "gyro_y", "gyro_z"])

# --- Magnetometer ---
mag_path = os.path.join(DATA_DIR, "magnetometer.csv")
if not os.path.isfile(mag_path):
    with open(mag_path, "w", newline="") as f:
        csv.writer(f).writerow(["timestamp", "heading_x", "heading_y", "heading_z"])

# --- Pressure ---
pressure_path = os.path.join(DATA_DIR, "pressure.csv")
if not os.path.isfile(pressure_path):
    with open(pressure_path, "w", newline="") as f:
        csv.writer(f).writerow(["timestamp", "pressure"])

# --- Water Temperature ---
water_temp_path = os.path.join(DATA_DIR, "water_temp.csv")
if not os.path.isfile(water_temp_path):
    with open(water_temp_path, "w", newline="") as f:
        csv.writer(f).writerow(["timestamp", "water_temp"])

# --- Humidity ---
humid_path = os.path.join(DATA_DIR, "humidity.csv")
if not os.path.isfile(humid_path):
    with open(humid_path, "w", newline="") as f:
        csv.writer(f).writerow(["timestamp", "humidity"])

# --- Air Temperature ---
air_temp_path = os.path.join(DATA_DIR, "air_temp.csv")
if not os.path.isfile(air_temp_path):
    with open(air_temp_path, "w", newline="") as f:
        csv.writer(f).writerow(["timestamp", "air_temp"])

# --- Controller Board Temperature ---
board_temp_path = os.path.join(DATA_DIR, "board_temp.csv")
if not os.path.isfile(board_temp_path):
    with open(board_temp_path, "w", newline="") as f:
        csv.writer(f).writerow(["timestamp", "board_temp"])


# Worker function to run the video stream in a background process (daemon)
def stream_worker(url: str):
    running = True
    proc = None

    # Stop loop when terminate / interrupt signals are received
    def handle_exit(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, handle_exit)
    signal.signal(signal.SIGINT, handle_exit)

    # Run video stream in a loop so it can restart on disconnects or errors
    while running:
        proc = subprocess.Popen(
            [
                # Run Pi camera indefinitely
                "rpicam-vid", "-t", "0",

                # Video format options
                "--codec", "h264",
                "--inline",
                "--width", str(WIDTH),
                "--height", str(HEIGHT),
                "--framerate", str(FPS),

                # Encoding options
                "--bitrate", str(BITRATE),
                "--intra", "30",

                # Server options
                "--listen",
                "-o", url
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True
        )

        # Wait until process exits or shutdown requested
        while running:
            try:
                proc.wait(timeout=0.5)
                break  # process exited
            except subprocess.TimeoutExpired:
                continue # process is still running
        
        # If we were asked to stop, exit loop
        if not running:
            break
        
        # If process exited unexpectedly, log and then restart
        _, err = proc.communicate()
        if err:
            print(err)
        print("Stream process exited, restarting...")

        time.sleep(1)
    
    # Loop has exited, clean up and shutdown process

    if proc and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
    
    sys.exit(0)


def main():
    Server = NetworkServer()
    SPIServer = MCUComServer()

    ControlMessage = ControlMessage_pb2.ControlMessage()

    while True:
        Message = Server.Receive()

        if Message is not None:
            # Deserialize control message
            ControlMessage.ParseFromString(Message)
            
            SPIServer.set_servo(
                forward=ControlMessage.Forward,
                pitch=ControlMessage.Pitch,
                roll=ControlMessage.Roll
            )
            SPIServer.set_thruster(thrust=ControlMessage.Thrust)

            return_packet = SPIServer.send_receive()

            # If valid packet, write sensor data to CSV
            if return_packet is not None:
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

                # Append accelerometer data
                with open(accel_path, "a", newline="") as f:
                    csv.writer(f).writerow([ts, return_packet.IMUCom.acc_x, return_packet.IMUCom.acc_y, return_packet.IMUCom.acc_z])
                
                # Append gyroscope data
                with open(gyro_path, "a", newline="") as f:
                    csv.writer(f).writerow([ts, return_packet.IMUCom.gyro_x, return_packet.IMUCom.gyro_y, return_packet.IMUCom.gyro_z])
                
                # Append magnetometer data
                with open(mag_path, "a", newline="") as f:
                    csv.writer(f).writerow([ts, return_packet.MagCom.head_x, return_packet.MagCom.head_y, return_packet.MagCom.head_z])
                
                # Append pressure data
                with open(pressure_path, "a", newline="") as f:
                    csv.writer(f).writerow([ts, return_packet.Bar30Com.pressure])
                
                # Append water temp data
                with open(water_temp_path, "a", newline="") as f:
                    csv.writer(f).writerow([ts, return_packet.Bar30Com.water_temp])
                
                # Append humidity data
                with open(humid_path, "a", newline="") as f:
                    csv.writer(f).writerow([ts, return_packet.HumidCom.humidity])
                
                # Append air temp data
                with open(air_temp_path, "a", newline="") as f:
                    csv.writer(f).writerow([ts, return_packet.HumidCom.air_temp])
                
                # Append board temp data
                with open(board_temp_path, "a", newline="") as f:
                    csv.writer(f).writerow([ts, return_packet.BoardCom.board_temp])


if __name__ == "__main__":
    mp.set_start_method("spawn")  # safe cross-platform process spawning
    
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--url",
        default="tcp://0.0.0.0:5555",
        help="Stream URL"
    )
    args = parser.parse_args()

    # Start streaming process
    p = mp.Process(target=stream_worker, args=(args.url,), daemon=True)
    p.start()

    # Run main server loop
    try:
        main()
        time.sleep(1)
    except KeyboardInterrupt:
        pass # exit on Ctrl + C
    finally:
        # Terminate background process on exit
        if p.is_alive():
            p.terminate()
            p.join()
        
    print("Done.")
