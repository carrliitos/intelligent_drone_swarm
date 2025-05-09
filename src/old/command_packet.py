import struct

COMMANDER_HEADER = 0x30

class CommandPacket:
  def build_command_packet(self, roll, pitch, yaw, thrust):
    roll = max(min(roll, 1.0), -1.0)
    pitch = max(min(pitch, 1.0), -1.0)
    yaw = max(min(yaw, 1.0), -1.0)
    thrust = max(min(thrust, 65535), 0)

    packet = struct.pack('<BfffH', COMMANDER_HEADER, roll, pitch, yaw, thrust)

    crc = sum(packet) & 0xFF
    packet += struct.pack('<B', crc)

    return packet

  def build_zero_point_command(self):
    packet = struct.pack('<BfffH', COMMANDER_HEADER, 0.0, 0.0, 0.0, 0)
    crc = sum(packet) & 0xFF
    packet += struct.pack('<B', crc)

    return packet
