# Changelog

## 0.2.0

### Changed

- Raise the minimum supported bb version to 0.25. CI covers bb 0.25 and the
  locked bb 0.33 release.
- Let bb own actuator command dispatch and payload validation. Commands sent
  while the robot is disarmed no longer write to a hub command slot.
- Send example wheel efforts through the public bb command API so they carry
  the current arm epoch on bb versions that require it. Custom command value
  types remain supported.

### Fixed

- Avoid looking up a MuJoCo interpreter when simulator child options are
  supplied, allowing the example's fake-child tests to run without MuJoCo.
