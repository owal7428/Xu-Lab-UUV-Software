import sys
import time
import signal
import subprocess
import multiprocessing as mp
import argparse

from NetworkServer import *

WIDTH = 1280
HEIGHT = 720
FPS = 30
BITRATE = 4_000_000

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

    ControlMessage = ControlMessage_pb2.ControlMessage()

    while True:
        Message = Server.Receive()

        if Message is not None:
            ControlMessage.ParseFromString(Message)
            print(f"Received control message: Forward={ControlMessage.Forward}, Pitch={ControlMessage.Pitch}, Roll={ControlMessage.Roll}, Thrust={ControlMessage.Thrust}")


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
