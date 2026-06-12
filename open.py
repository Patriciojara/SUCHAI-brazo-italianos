#!/usr/bin/env python3
import time
from smbus2 import SMBus, i2c_msg

BUS_NUM = 1
HADES_ADDR = 0x30

RESULT_NAMES = {
    0: "NONE",
    1: "ACCEPTED",
    2: "BUSY",
    3: "TIMEOUT",
    4: "ERROR",
    5: "INVALID",
}

MOTION_NAMES = {
    0: "IDLE",
    1: "RUNNING",
    2: "SUCCEEDED",
    3: "TIMED_OUT",
    4: "ERROR",
    5: "INTERRUPTED",
}

def next_seq():
    path = "/tmp/hades_seq.txt"
    try:
        with open(path, "r") as f:
            seq = int(f.read().strip())
    except Exception:
        seq = 0

    seq = (seq + 1) & 0xFF
    if seq == 0:
        seq = 1

    with open(path, "w") as f:
        f.write(str(seq))

    return seq

def i2c_write(bus, data, retries=5, delay=0.05):
    for attempt in range(retries):
        try:
            msg = i2c_msg.write(HADES_ADDR, data)
            bus.i2c_rdwr(msg)
            return
        except OSError as e:
            print(f"I2C write error, intento {attempt + 1}/{retries}: {e}")
            time.sleep(delay)
    raise RuntimeError("No se pudo escribir por I2C")

def i2c_read(bus, reg, n, retries=5, delay=0.05):
    for attempt in range(retries):
        try:
            write_reg = i2c_msg.write(HADES_ADDR, [reg])
            read_data = i2c_msg.read(HADES_ADDR, n)
            bus.i2c_rdwr(write_reg, read_data)
            return list(read_data)
        except OSError as e:
            print(f"I2C read error, intento {attempt + 1}/{retries}: {e}")
            time.sleep(delay)
    raise RuntimeError(f"No se pudo leer registro 0x{reg:02X}")

def wait_ack(bus, seq, timeout_s=0.5):
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        data = i2c_read(bus, 0x08, 6)

        last_result = data[0]
        last_cmd = data[1]
        last_seq = data[2]
        pending = data[3]
        motion = data[4]

        if last_seq == seq:
            return last_result, last_cmd, last_seq, pending, motion

        time.sleep(0.01)

    raise TimeoutError("No llegó ACK del comando")

def wait_motion_finished(bus, timeout_s=7.0):
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        state = i2c_read(bus, 0x0C, 1)[0]
        print(f"MOTION_STATE = {state} ({MOTION_NAMES.get(state, 'UNKNOWN')})")

        if state != 1:
            return state

        time.sleep(0.05)

    raise TimeoutError("El movimiento sigue RUNNING después del timeout")

def main():
    seq = next_seq()

    payload = [
        0x60,                    # registro inicial
        0x01,                    # MOTOR_MODE = OPEN / FORWARD
        0x01,                    # MOTOR_ENABLE = ON
        0x88, 0x13, 0x00, 0x00,  # DURATION = 5000 ms little-endian
        seq,                     # SEQUENCE
        0x01,                    # COMMAND = MOVE
    ]

    print("Enviando OPEN...")
    print("Payload:", " ".join(f"0x{x:02X}" for x in payload))

    with SMBus(BUS_NUM) as bus:
        i2c_write(bus, payload)

        result, cmd, last_seq, pending, motion = wait_ack(bus, seq)

        print(
            f"ACK: result={result}({RESULT_NAMES.get(result, 'UNKNOWN')}), "
            f"cmd=0x{cmd:02X}, seq={last_seq}, pending={pending}, "
            f"motion={motion}({MOTION_NAMES.get(motion, 'UNKNOWN')})"
        )

        if result != 1:
            raise RuntimeError(f"OPEN rechazado: {RESULT_NAMES.get(result, result)}")

        print("OPEN aceptado. Esperando fin del movimiento...")
        final_state = wait_motion_finished(bus)

        print(f"OPEN terminado: {final_state} ({MOTION_NAMES.get(final_state, 'UNKNOWN')})")

if __name__ == "__main__":
    main()