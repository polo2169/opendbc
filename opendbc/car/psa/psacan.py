def psa_checksum(address: int, sig, d: bytearray) -> int:
  # Values validated on the Peugeot 308 T9 corpus. Addresses not listed keep
  # the historical PSA default used by the upstream DBC.
  chk_ini = {
    0x228: 0x3,
    0x2ED: 0x2,
    0x2F5: 0xB,
    0x329: 0xD,
    0x349: 0x6,
    0x38D: 0x7,
    0x3AD: 0xD,
    0x3C9: 0xE,
    0x3CD: 0x3,
    0x42D: 0xC,
    0x452: 0x4,
    0x50D: 0xC,
  }.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF


def create_lka_steering(packer, lat_active: bool, apply_angle: float, status: int):
  values = {
    'DRIVE': 1,
    'STATUS': status,
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
