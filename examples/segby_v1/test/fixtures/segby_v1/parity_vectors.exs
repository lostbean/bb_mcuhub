# GENERATED parity vectors (§03/§06) — the cross-language witness.
# Each row: a value, its exact CRC-covered body bytes, and the CRC, computed
# by running the real encoder. Asserted by the Elixir suite AND the
# host-compiled C harness. Regenerate with `mix wire.gen`; a non-empty diff
# means a contract moved and the bytes moved with it. Hand-editing a row is
# the tell.
[
  %{
    hub: :blaster, port: :range_front, type: SegbyV1.ValueTypes.Range,
    node: 0x02, port_id: 0x5B,
    seq: 42, t_dev: 1234, stamped: false,
    value: %{distance_m: 1.0},
    body: <<0x2, 0x5B, 0x0, 0x2A, 0x3F, 0x80, 0x0, 0x0>>,
    crc: 0x9267
  },
  %{
    hub: :blaster, port: :status_led, type: SegbyV1.ValueTypes.Led,
    node: 0x02, port_id: 0xCE,
    seq: 42, t_dev: 1234, stamped: false,
    value: %{b: 3, g: 2, r: 1},
    body: <<0x2, 0xCE, 0x0, 0x2A, 0x1, 0x2, 0x3>>,
    crc: 0x916A
  },
  %{
    hub: :blaster, port: :pose, type: :imu,
    node: 0x02, port_id: 0xD3,
    seq: 42, t_dev: 1234, stamped: true,
    value: %{ax: 3.5, ay: 4.0, az: 4.5, qw: 1.0, qx: 0.5, qy: 1.0, qz: 1.5, wx: 2.0, wy: 2.5, wz: 3.0},
    body: <<0x2, 0xD3, 0x0, 0x2A, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x4, 0xD2, 0x3F, 0x80, 0x0, 0x0, 0x3F, 0x0, 0x0, 0x0, 0x3F, 0x80, 0x0, 0x0, 0x3F, 0xC0, 0x0, 0x0, 0x40, 0x0, 0x0, 0x0, 0x40, 0x20, 0x0, 0x0, 0x40, 0x40, 0x0, 0x0, 0x40, 0x60, 0x0, 0x0, 0x40, 0x80, 0x0, 0x0, 0x40, 0x90, 0x0, 0x0>>,
    crc: 0xD7AE
  },
  %{
    hub: :wheels, port: :motor_left, type: :effort,
    node: 0x05, port_id: 0x18,
    seq: 42, t_dev: 1234, stamped: false,
    value: %{nm: 1.0},
    body: <<0x5, 0x18, 0x0, 0x2A, 0x3F, 0x80, 0x0, 0x0>>,
    crc: 0x5011
  },
  %{
    hub: :wheels, port: :vel_left, type: SegbyV1.ValueTypes.WheelSpeed,
    node: 0x05, port_id: 0x1C,
    seq: 42, t_dev: 1234, stamped: false,
    value: %{rad_s: 1.0},
    body: <<0x5, 0x1C, 0x0, 0x2A, 0x3F, 0x80, 0x0, 0x0>>,
    crc: 0x91D7
  },
  %{
    hub: :wheels, port: :status_left, type: :status,
    node: 0x05, port_id: 0x8A,
    seq: 42, t_dev: 1234, stamped: false,
    value: %{applied_seq: 7, floored: false},
    body: <<0x5, 0x8A, 0x0, 0x2A, 0x0, 0x7, 0x0>>,
    crc: 0x1C39
  },
  %{
    hub: :wheels, port: :status_right, type: :status,
    node: 0x05, port_id: 0xB4,
    seq: 42, t_dev: 1234, stamped: false,
    value: %{applied_seq: 7, floored: false},
    body: <<0x5, 0xB4, 0x0, 0x2A, 0x0, 0x7, 0x0>>,
    crc: 0xB316
  },
  %{
    hub: :wheels, port: :vel_right, type: SegbyV1.ValueTypes.WheelSpeed,
    node: 0x05, port_id: 0xCC,
    seq: 42, t_dev: 1234, stamped: false,
    value: %{rad_s: 1.0},
    body: <<0x5, 0xCC, 0x0, 0x2A, 0x3F, 0x80, 0x0, 0x0>>,
    crc: 0xD0B9
  },
  %{
    hub: :wheels, port: :motor_right, type: :effort,
    node: 0x05, port_id: 0xE0,
    seq: 42, t_dev: 1234, stamped: false,
    value: %{nm: 1.0},
    body: <<0x5, 0xE0, 0x0, 0x2A, 0x3F, 0x80, 0x0, 0x0>>,
    crc: 0xEC24
  }
]
