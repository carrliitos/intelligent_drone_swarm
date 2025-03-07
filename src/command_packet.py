import struct

class PacketHeader:
  COMMANDER_HEADER = 0x30

class CommandPacket:
  def build(self, roll: float, pitch: float, yaw: float, thrust: int) -> bytes:
    """
    Builds a command packet for the ESP-Drone.

    Args:
      roll (float): Roll value (-1.0 to 1.0).
      pitch (float): Pitch value (-1.0 to 1.0).
      yaw (float): Yaw value (-1.0 to 1.0).
      thrust (int): Thrust value (0 to 65535).

    Returns:
      bytes: Serialized command packet with CRC.
    """
    # Ensure values are within expected ranges
    roll = max(min(roll, 1.0), -1.0)
    pitch = max(min(pitch, 1.0), -1.0)
    yaw = max(min(yaw, 1.0), -1.0)
    thrust = max(min(thrust, 65535), 0)

    # Pack data into bytes (little-endian)
    packet = struct.pack('<BfffH', PacketHeader.COMMANDER_HEADER, roll, pitch, yaw, thrust)

    # Calculate CRC (simple sum of bytes & mask to 1 byte)
    crc = sum(packet) & 0xFF
    packet += struct.pack('<B', crc)

    return packet

  def extract(self, packet: bytes):
    """
    Extracts values from a command packet.

    Args:
      packet (bytes): Serialized command packet.

    Returns:
      tuple: (header, roll, pitch, yaw, thrust, crc).
    """
    if len(packet) != 16:
      raise ValueError("Invalid packet size. Expected 16 bytes.")

    # Unpack packet based on structure
    header, roll, pitch, yaw, thrust, crc = struct.unpack('<BfffHB', packet)
    
    # Validate CRC
    calculated_crc = sum(packet[:-1]) & 0xFF
    if calculated_crc != crc:
      raise ValueError("CRC check failed!")

    return header, roll, pitch, yaw, thrust, crc
