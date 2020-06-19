import time
from io import StringIO
import tempfile

import pyshark
import serial  # pyserial
import pandas as pd

pd.set_option('float_format', '{:f}'.format)


def do_network_capture(duration, interface):
    capture = pyshark.LiveCapture(interface=interface)
    capture.sniff(timeout=duration)
    packets = [capture[n] for n in range(len(capture))]
    return packets


def read_arduino_results_from_serial(ser):
    lines = None
    while True:
        line = ser.readline().decode("US-ASCII")
        if line.startswith("resultsStart"):
            lines = []
        elif line.startswith("resultsEnd"):
            return pd.read_csv(StringIO("\n".join(lines)))
        elif lines is not None:
            lines.append(line.strip())


def packets_to_df(packets):
    packets = [p for p in packets
               if hasattr(p, 'ip') and hasattr(p.ip, 'proto') and p.ip.proto in ['17', '6']]  # only TCP and UDP

    def get_port(p, is_src):
        if p.ip.proto == '6':  # TCP
            return p.tcp.srcport if is_src else p.tcp.dstport
        elif p.ip.proto == '17':  # UDP
            return p.udp.srcport if is_src else p.udp.dstport

    def get_data_len(p):
        if p.ip.proto == '6': # TCP
            return p.tcp.len
        elif p.ip.proto == '17':  # UDP
            return p.udp.length

    def get_proto_name(p):
        if p.ip.proto == '6':
            return 'TCP'
        elif p.ip.proto == '17':
            return 'UDP'

    return pd.DataFrame(
        [(p.sniff_timestamp, get_data_len(p), p.ip.src, get_port(p, True),
          p.ip.dst, get_port(p, False), get_proto_name(p)) for p in packets],
        columns=['t', 'data_len', 'src', 'srcport', 'dest', 'destport', 'proto'])


def try_sync_handshake(ser):
    s = ser.readline()  # ser should have a timeout set (e.g. 1s)
    if s[0:6] == b"SYNC\r\n":
        host_t = int(time.time() * 1000000)
        ser.write(b"OK\r\n")
        ser.flush()
        return host_t, ser.readline()
    else:
        return None


def process_sync_handshake(sync):
    host_t = sync[0]
    s1, s2 = [int(x) for x in sync[1].decode("US-ASCII").split()[1:3]]
    if s2 < s1:
        # micros() on the arduino has overflowed... it's only a 32 bit unsigned int so we can fix it...
        s2 += 2 ** 32
    gap = s2 - s1
    if gap > 2000:
        print(f"WARNING: large sync gap for arduino timestamp: {gap}")
    offset_t = host_t - ((s1 + s2) / 2)

    print(f"timestamp sync: rtt: {gap} us, offset: {offset_t}")

    return offset_t


def main(serial_port, ethernet_nic):
    ser = serial.Serial(serial_port, 115200, timeout=1)
    capture_duration = 2.1

    while True:
        sync = try_sync_handshake(ser)  # return None or a tuple of (host_t, sync response)
        if sync:
            packets = do_network_capture(capture_duration, ethernet_nic)
            ard_df = read_arduino_results_from_serial(ser)

            print("------ CAPTURE COMPLETE -------")
            offset_t = process_sync_handshake(sync)

            ard_df['t0'] = (ard_df['t0'] + (offset_t / 1000)) / 1000.0
            ard_df['t'] = ard_df['t'] / 1000.0 + ard_df['t0']

            print("Light sensor:")
            print(ard_df)

            cap_df = packets_to_df(packets)
            print("Packets:")
            print(cap_df)

            print(f"Writing results to {tempfile.gettempdir()}")
            ard_df.to_csv(f"{tempfile.gettempdir()}/ard.csv", index=False)
            cap_df.to_csv(f"{tempfile.gettempdir()}/cap.csv", index=False)

            print("------ DONE -------")


if __name__ == '__main__':
    main('COM4',  # Arduino
         '3'  # interface that you might be doing RDP/Horizon over
         )
