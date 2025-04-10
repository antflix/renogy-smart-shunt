import logging
# from .BaseClient import BaseClient
from .BaseShuntClient import BaseClient
from .Utils import bytes_to_int, parse_temperature
import csv
import time
from datetime import datetime
# Read and parse BT-1 RS232 type bluetooth module connected to Renogy Rover/Wanderer/Adventurer
# series charge controllers. Also works with BT-2 RS485 module on Rover Elite, DC Charger etc.
# Does not support Communication Hub with multiple devices connected

FUNCTION = {
    3: "READ",
    6: "WRITE"
}

CHARGING_STATE = {
    0: 'deactivated',
    1: 'activated',
    2: 'mppt',
    3: 'equalizing',
    4: 'boost',
    5: 'floating',
    6: 'current limiting'
}

LOAD_STATE = {
  0: 'off',
  1: 'on'
}

BATTERY_TYPE = {
    1: 'open',
    2: 'sealed',
    3: 'gel',
    4: 'lithium',
    5: 'custom'
}
last_values = {}

def scan_unknown_bytes(bs, skip_ranges=[(21, 24), (25, 28), (30, 32), (66, 69), (70, 73)],
                       threshold=0.1, output_file="renogy_scan.csv"):
    global last_values
    max_len = len(bs)
    lengths = [1, 2, 3, 4]
    scales = [1, 0.1, 0.01, 0.001]
    signed_options = [False, True]
    results = []

    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    for offset in range(max_len):
        if any(start <= offset < end for start, end in skip_ranges):
            continue

        for length in lengths:
            if offset + length > max_len:
                continue

            raw_bytes = bs[offset:offset+length]
            for signed in signed_options:
                try:
                    value = int.from_bytes(raw_bytes, byteorder='big', signed=signed)
                    for scale in scales:
                        scaled_value = round(value * scale, 3)
                        key = f"{offset}_{length}_{signed}_{scale}"
                        last_val = last_values.get(key, scaled_value)
                        delta = round(scaled_value - last_val, 3)
                        trend = "="
                        if delta > threshold:
                            trend = "↑"
                        elif delta < -threshold:
                            trend = "↓"
                        last_values[key] = scaled_value

                        results.append({
                            'timestamp': timestamp,
                            'offset': offset,
                            'length': length,
                            'signed': signed,
                            'scale': scale,
                            'raw': raw_bytes.hex(),
                            'value': scaled_value,
                            'delta': delta,
                            'trend': trend
                        })
                except Exception:
                    continue

    file_exists = False
    try:
        with open(output_file, "r"):
            file_exists = True
    except FileNotFoundError:
        pass

    with open(output_file, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        if not file_exists:
            writer.writeheader()
        writer.writerows(results)

    print(f"[{timestamp}] Scan written with {len(results)} rows.")


class ShuntClient(BaseClient):
    def __init__(self, config, on_data_callback=None, on_error_callback=None):
        super().__init__(config)
        logging.debug(msg="DEBUG Client")
        self.on_data_callback = on_data_callback
        self.on_error_callback = on_error_callback
        self.data = {}
        # self.sections = [
        #     {'register': 12, 'words': 8, 'parser': self.parse_device_info},
        #     {'register': 26, 'words': 1, 'parser': self.parse_device_address},
        #     {'register': 256, 'words': 34, 'parser': self.parse_chargin_info},
        #     {'register': 57348, 'words': 1, 'parser': self.parse_battery_type}
        # ]
        self.sections = [
            {'register': 256, 'words': 110, 'parser': self.parse_shunt_info}
        ]
        self.set_load_params = {'function': 6, 'register': 266}

    def on_data_received(self, response):
        operation = bytes_to_int(response, 1, 1)
        if operation == 6: # write operation
            self.parse_set_load_response(response)
            self.on_write_operation_complete()
            self.data = {}
        else:
            # read is handled in base class
            super().on_data_received(response)

    def on_write_operation_complete(self):
        logging.info("on_write_operation_complete")
        if self.on_data_callback is not None:
            self.on_data_callback(self, self.data)

    def set_load(self, value = 0):
        logging.info("setting load {}".format(value))
        request = self.create_generic_read_request(self.device_id, self.set_load_params["function"], self.set_load_params["register"], value)
        self.device.characteristic_write_value(request)

    def parse_device_info(self, bs):
        data = {}
        data['function'] = FUNCTION.get(bytes_to_int(bs, 1, 1))
        data['model'] = (bs[3:17]).decode('utf-8').strip()
        self.data.update(data)

    def parse_device_address(self, bs):
        data = {}
        data['device_id'] = bytes_to_int(bs, 4, 1)
        self.data.update(data)

    def parse_shunt_info(self, bs):
        data = {}
        # temp_unit = self.config['data']['temperature_unit']
        data['charge_battery_voltage'] = bytes_to_int(bs, 25, 3, scale = 0.001) # 0xA6 (#1)
        data['starter_battery_voltage'] = bytes_to_int(bs, 30, 2, scale = 0.001) # 0xA6 (#2)
        data['discharge_amps'] = bytes_to_int(bs, 21, 3, scale = 0.001, signed=True) # 0xA4 (#1)
        data['discharge_watts'] = round((data['charge_battery_voltage'] * data['discharge_amps']), 2)
        data['temperature_sensor_1'] = 0.00 if bytes_to_int(bs, 67, 1) == 0 else bytes_to_int(bs, 66, 3, scale = 0.001) # 0xAD (#3)
        data['temperature_sensor_2'] = 0.00 if bytes_to_int(bs, 71, 1) == 0 else bytes_to_int(bs, 70, 3, scale = 0.001) # 0xAD (#4)
        # unknown values:
        # - time_remaining
        # - discharge_duration
        # - consumed_amp_hours
        scan_unknown_bytes(bs)

        self.data.update(data)
        # logging.debug(msg=f"DATA: {self.data}")
        return data
        
