import time
from loralib import Lora, LoraGCSClient

def test_companion():
    def callback(packet):
        print(f"Received from GCS: {packet}")
        if packet.startswith("add_waypoint"):
            return "waypoint_added"
        elif packet == "start_mission":
            return "mission_started"
        elif packet == "emergency_shutdown":
            return "shutdown_initiated"
        elif packet == "start_manual_control":
            return "manual_control_started"
        elif packet.startswith("manual_control"):
            return "manual_ack"
        elif packet in ("state","latitude","longitude","degree_from_north","speed"):
            return "test_response"
        else:
            return None

    print("Companion Lora callback started and listening for messages...")
    lora = Lora(message_callback=callback)


def test_gcs_client():
    client = LoraGCSClient()
    print("Testing GCS client...")
    print("Start mission response:", client.start_mission())
    print("Add waypoint response:", client.add_waypoint(12.34, 56.78))
    print("Emergency shutdown response:", client.emergency_shutdown())
    print("Start manual control response:", client.start_manual_control())
    time.sleep(1)
    client.manual_speed = 5
    client.manual_yaw = 90
    time.sleep(1)
    client.stop_manual_control()
    print("Manual control stopped.")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Test Lora library functionality.")
    parser.add_argument("mode", choices=["gcs", "companion"], help="Mode to run the test in")
    args = parser.parse_args()
    if args.mode == "gcs":
        test_gcs_client()
    else:
        test_companion()
