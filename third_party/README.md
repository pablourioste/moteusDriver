# Third-party code

## `bmi270_config.h` — Bosch BMI270 configuration blob

**Source:** [BMI270-Sensor-API](https://github.com/boschsensortec/BMI270-Sensor-API),
`bmi270.c`, array `bmi270_config_file[]`
**Copyright:** Bosch Sensortec GmbH
**Licence:** Apache License 2.0
**Size:** 8192 bytes

### Why this file exists

The BMI270 is not a register-and-go part. After power-on it keeps the
accelerometer and gyroscope **disabled** until an ~8 KB configuration blob has
been uploaded into its internal core and `INTERNAL_STATUS` reports `0b0001`.

Until that happens every data register reads zero — while `CHIP_ID` reads back
correctly the whole time. The symptom looks exactly like a wiring fault and is
not one. This costs people days.

`ImuDriver::uploadConfigBlob()` implements the datasheet sequence: disable
advanced power save, `INIT_CTRL = 0`, burst-write the blob through
`INIT_ADDR_0/1` + `INIT_DATA`, `INIT_CTRL = 1`, then poll `INTERNAL_STATUS`.

### Build integration

`CMakeLists.txt` auto-enables `BMI270_CONFIG_HEADER` when this file is
present, which defines `HAVE_BMI270_CONFIG_BLOB` for `cube_drivers`. Without
it the driver still compiles, but `initialize()` fails with an explanatory
message rather than silently returning zeros.

### Regenerating

If upstream changes, or to verify this copy:

```bash
curl -sL https://raw.githubusercontent.com/boschsensortec/BMI270-Sensor-API/master/bmi270.c \
  -o /tmp/bmi270.c

python3 - <<'PY'
import re
src = open('/tmp/bmi270.c').read()
m = re.search(r'const uint8_t bmi270_config_file\[\]\s*=\s*\{(.*?)\};', src, re.S)
if not m:
    raise SystemExit('array not found - upstream layout changed')
body = m.group(1)
n = len(re.findall(r'0x[0-9a-fA-F]{2}', body))
with open('third_party/bmi270_config.h', 'w') as f:
    f.write('// Extracted from BMI270-Sensor-API (Apache-2.0), Bosch Sensortec.\n')
    f.write('#pragma once\n#include <cstdint>\n\n')
    f.write('static const uint8_t bmi270_config_file[] = {')
    f.write(body)
    f.write('};\n')
print(f'wrote {n} bytes')
PY
```

Expect **8192** bytes. A materially different count means the upstream file
layout changed and the regex needs revisiting — do not ship a partial blob.

### Why it is vendored rather than fetched at build time

It is a fixed binary that must match the driver's upload sequence, and a
build that silently downloads code is worse than one that carries it
explicitly with its licence stated. Transcribing it by hand would be worse
still: an error produces an IMU that initialises cleanly and then returns
subtly wrong data — the worst possible failure mode for a balancing rig.
