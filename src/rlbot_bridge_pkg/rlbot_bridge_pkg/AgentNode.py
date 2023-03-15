
import sys, os
from rlbot.setup_manager import SetupManager
from rlbot.utils.python_version_check import check_python_version
cfg = os.environ.get("RLBOT_AGENT_DIR")+"/rlbot.cfg"
print(f"Config dir: {cfg}")
# from rlbot_msgs.msg import GameTickPacketMsg
def main():
    print("starting")
    check_python_version()
    manager = SetupManager()
    manager.load_config(config_location=cfg)
    manager.connect_to_game()
    manager.launch_early_start_bot_processes()
    manager.start_match()
    manager.launch_bot_processes()
    manager.infinite_loop()  # Runs forever until interrupted


if __name__ == '__main__':
    main()
