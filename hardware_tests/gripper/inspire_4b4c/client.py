"""Inspire 4B4C serial protocol adapter.

Protocol from the vendor ROS 2 SDK: EB 90 | id | length | opcode | payload |
checksum. The checksum is the low byte of the id..payload sum.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class ProtocolError(RuntimeError):
    """The gripper did not return a valid response."""


class ByteTransport(Protocol):
    def write(self, data: bytes) -> int: ...
    def read(self, size: int = 1) -> bytes: ...
    @property
    def in_waiting(self) -> int: ...


@dataclass(frozen=True)
class GripperStatus:
    running_status: int
    error_code: int
    temperature_c: int


class Inspire4B4C:
    OPEN_MIN, OPEN_MAX = 0, 1000
    ID_MIN, ID_MAX = 1, 254
    SPEED_MIN, SPEED_MAX = 1, 1000
    FORCE_MIN, FORCE_MAX = 50, 1000

    SET_ID, SET_PARAM = 0x04, 0x01
    MOVE_GRASP, MOVE_RELEASE, SET_OPEN_LIMIT, GET_OPEN_LIMIT = 0x10, 0x11, 0x12, 0x13
    ESTOP, CLEAR_ERROR, MOVE_GRASP_HOLD, MOVE_TARGET = 0x16, 0x17, 0x18, 0x54
    GET_STATUS, GET_OPENING = 0x41, 0xD9

    def __init__(self, transport: ByteTransport, gripper_id: int = 1, response_timeout_reads: int = 8) -> None:
        self.transport = transport
        self.gripper_id = self._bounded("gripper_id", gripper_id, self.ID_MIN, self.ID_MAX)
        self.response_timeout_reads = response_timeout_reads

    @classmethod
    def open_serial(cls, port: str = "/dev/ttyUSB0", baudrate: int = 115200, gripper_id: int = 1) -> "Inspire4B4C":
        """Connect to hardware only when called explicitly; requires `pyserial`."""
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("请先安装 pyserial：python3 -m pip install pyserial") from exc
        return cls(serial.Serial(port=port, baudrate=baudrate, timeout=0.1), gripper_id)

    @staticmethod
    def frame(gripper_id: int, opcode: int, payload: bytes = b"") -> bytes:
        if not 0 <= gripper_id <= 255 or not 0 <= opcode <= 255:
            raise ValueError("gripper_id and opcode must be bytes")
        body = bytes((gripper_id, 1 + len(payload), opcode)) + payload
        return b"\xEB\x90" + body + bytes((sum(body) & 0xFF,))

    def set_id(self, new_id: int) -> bool:
        new_id = self._bounded("new_id", new_id, self.ID_MIN, self.ID_MAX)
        accepted = self._command(self.SET_ID, bytes((new_id,)))
        if accepted: self.gripper_id = new_id
        return accepted

    def set_open_limits(self, maximum: int, minimum: int, *, persist: bool = False) -> bool:
        maximum, minimum = self._opening(maximum), self._opening(minimum)
        if minimum > maximum: raise ValueError("minimum opening cannot exceed maximum opening")
        accepted = self._command(self.SET_OPEN_LIMIT, self._u16(maximum) + self._u16(minimum))
        return self.persist_parameters() if accepted and persist else accepted

    def move_to(self, opening: int) -> bool:
        return self._command(self.MOVE_TARGET, self._u16(self._opening(opening)))

    def release(self, speed: int) -> bool:
        return self._command(self.MOVE_RELEASE, self._u16(self._speed(speed)))

    def grasp(self, speed: int, force: int, *, hold: bool = False) -> bool:
        opcode = self.MOVE_GRASP_HOLD if hold else self.MOVE_GRASP
        return self._command(opcode, self._u16(self._speed(speed)) + self._u16(self._force(force)))

    def emergency_stop(self) -> bool: return self._command(self.ESTOP)
    def clear_error(self) -> bool: return self._command(self.CLEAR_ERROR)
    def persist_parameters(self) -> bool: return self._command(self.SET_PARAM)

    def get_open_limits(self) -> tuple[int, int]:
        response = self._query(self.GET_OPEN_LIMIT, 9)
        return self._read_u16(response, 5), self._read_u16(response, 7)

    def get_opening(self) -> int:
        value = self._read_u16(self._query(self.GET_OPENING, 7), 5)
        return 1000 if 1000 < value < 2000 else 0 if value > 30000 else value

    def get_status(self) -> GripperStatus:
        response = self._query(self.GET_STATUS, 8)
        return GripperStatus(response[5], response[6], response[7])

    def _command(self, opcode: int, payload: bytes = b"") -> bool:
        response = self._exchange(opcode, payload, 6)
        return response[5] == 1

    def _query(self, opcode: int, min_response: int) -> bytes:
        return self._exchange(opcode, b"", min_response)

    def _exchange(self, opcode: int, payload: bytes, min_response: int) -> bytes:
        self.transport.write(self.frame(self.gripper_id, opcode, payload))
        response = bytearray()
        for _ in range(self.response_timeout_reads):
            response.extend(self.transport.read(max(1, getattr(self.transport, "in_waiting", 0))))
            if len(response) >= min_response: break
        if len(response) < min_response: raise ProtocolError("夹爪未返回完整响应")
        if response[:2] != b"\xEB\x90": raise ProtocolError("响应帧头错误")
        if response[2] != self.gripper_id: raise ProtocolError("响应夹爪 ID 不匹配")
        if response[4] != opcode: raise ProtocolError("响应操作码不匹配")
        return bytes(response)

    @staticmethod
    def _u16(value: int) -> bytes: return value.to_bytes(2, "little")
    @staticmethod
    def _read_u16(data: bytes, offset: int) -> int:
        if len(data) < offset + 2: raise ProtocolError("响应长度不足")
        return data[offset] | (data[offset + 1] << 8)
    @staticmethod
    def _bounded(name: str, value: int, lower: int, upper: int) -> int:
        if not isinstance(value, int) or isinstance(value, bool) or not lower <= value <= upper:
            raise ValueError(f"{name} 必须在 [{lower}, {upper}] 范围内")
        return value
    def _opening(self, value: int) -> int: return self._bounded("opening", value, self.OPEN_MIN, self.OPEN_MAX)
    def _speed(self, value: int) -> int: return self._bounded("speed", value, self.SPEED_MIN, self.SPEED_MAX)
    def _force(self, value: int) -> int: return self._bounded("force", value, self.FORCE_MIN, self.FORCE_MAX)
