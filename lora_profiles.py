"""电脑端 LoRa 模块配置协议。"""

from dataclasses import dataclass

UART_CODES = {1200: 0, 2400: 1, 4800: 2, 9600: 3, 19200: 4, 38400: 5, 57600: 6, 115200: 7}
AIR_RATES = {0: 2400, 1: 2400, 2: 2400, 3: 4800, 4: 9600, 5: 19200, 6: 38400, 7: 62500}

# 已完成闭环验证的工程默认值；配置模式仍使用模块手册规定的 9600。
DEFAULT_CONFIG_BAUD = 9600
DEFAULT_DATA_BAUD = 115200
DEFAULT_AIR_RATE = 5
DEFAULT_CHANNEL = 0x17


@dataclass(frozen=True)
class Ewm220Config:
    """EWM220M 的 00~08 寄存器快照。"""

    addh: int = 0
    addl: int = 0
    uart_baud: int = DEFAULT_DATA_BAUD
    air_rate: int = DEFAULT_AIR_RATE
    packet_length: int = 0
    rssi_enable: bool = False
    channel: int = DEFAULT_CHANNEL
    transparent: bool = True
    fec: bool = True
    lbt: bool = False
    wor_period: int = 3
    encryption_key: bytes = b"\x00\x00"

    def to_registers(self) -> bytes:
        if self.uart_baud not in UART_CODES or self.air_rate not in AIR_RATES:
            raise ValueError("EWM220 UART 或空口速率不支持")
        if not 0 <= self.channel <= 83:
            raise ValueError("EWM220 信道必须为 0~83")
        reg0 = (UART_CODES[self.uart_baud] << 5) | (self.air_rate & 0x07)
        reg1 = ((self.packet_length & 0x03) << 6) | (0x20 if self.rssi_enable else 0)
        # bit5 是保留位，不能把 EWM32 的 FEC 位写入 EWM220。
        reg3 = (0x00 if self.transparent else 0x40) | (0x10 if self.lbt else 0) | (self.wor_period & 0x07)
        key = bytes(self.encryption_key)
        if len(key) != 2:
            raise ValueError("EWM220 加密密钥必须为 2 字节")
        return bytes((self.addh, self.addl, reg0, reg1, self.channel, reg3, key[0], key[1], 0))

    @classmethod
    def from_response(cls, response: bytes) -> "Ewm220Config":
        if len(response) < 12 or response[0] not in (0xC0, 0xC1) or response[1:3] != b"\x00\x09":
            raise ValueError("不是有效的 EWM220 参数响应")
        data = response[3:12]
        reg0, reg1, channel, reg3 = data[2:6]
        uart_code = (reg0 >> 5) & 0x07
        uart_baud = next((baud for baud, code in UART_CODES.items() if code == uart_code), None)
        if uart_baud is None:
            raise ValueError("EWM220 UART 编码非法")
        return cls(addh=data[0], addl=data[1], uart_baud=uart_baud, air_rate=reg0 & 0x07,
                   packet_length=(reg1 >> 6) & 0x03, rssi_enable=bool(reg1 & 0x20), channel=channel,
                   transparent=not bool(reg3 & 0x40), fec=True, lbt=bool(reg3 & 0x10),
                   wor_period=reg3 & 0x07, encryption_key=data[6:8])


def ewm220_read_all() -> bytes:
    return b"\xC1\x00\x09"


def ewm220_write_all(config: Ewm220Config) -> bytes:
    return b"\xC0\x00\x09" + config.to_registers()
