"""EWM220 配置帧最小自检。"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lora_profiles import Ewm220Config, ewm220_read_all, ewm220_write_all


def main() -> None:
    config = Ewm220Config(uart_baud=115200, air_rate=2, channel=23)
    assert ewm220_read_all() == bytes.fromhex("c1 00 09")
    write = ewm220_write_all(config)
    assert write[:3] == bytes.fromhex("c0 00 09")
    decoded = Ewm220Config.from_response(bytes.fromhex("c1 00 09") + write[3:])
    assert decoded.uart_baud == 115200 and decoded.air_rate == 2 and decoded.channel == 23
    print("LoRa profile self-test passed")


if __name__ == "__main__":
    main()
