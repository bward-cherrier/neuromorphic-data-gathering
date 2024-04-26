#!/usr/bin/env python3
"""
Import testing data from aedat4 files into a simple binary format.

Usage:
1. Download all needed aedat4 files into one directory.
2. Call import_data script to import the testing data into a simple binary format. ./import_data.py -d /home/radam/tmp_glints
"""
import argparse
from pathlib import Path
import json
import struct

from dv import AedatFile
import numpy as np

# Events data type used by numpy
DTYPE = np.dtype(
    {
        'names': ['ts', 'x', 'y', 'pol', 'a', 'b', 'c'],
        'formats': ['<u8', '<u2', '<u2', '<u1', '<u1', '<u1', '<u1']
    },
    align=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-d", "--data", help="path to a directory at which the aedat4 files are stored", required=True)
    args = parser.parse_args()

    aedat_dir = Path(args.data)

    for dataset_dir in aedat_dir.iterdir():
        aedat_path = aedat_dir / f"{dataset_dir.name}"
        if aedat_path.suffix != '.aedat4':
            continue

        if not aedat_path.exists():
            print(f"WARNING: aedat file does not exist: {aedat_path}")
            continue

        print(f"Processing {aedat_path} file...")
        event_data_path = aedat_dir / f"{aedat_path.stem}.data"

        # ts_min = range[0]
        # ts_max = range[1]

        with AedatFile(str(aedat_path)) as fin:
            with event_data_path.open("wb") as fout:
                events_list = np.hstack([packet for packet in fin["events"].numpy()])
                events_array = np.frombuffer(events_list.data, dtype=DTYPE)

                # Copy the matrices to achieve C-contiguous arrays
                ts = events_array["ts"].copy()
                x = events_array["x"].copy()
                y = events_array["y"].copy()
                pol = events_array["pol"].copy()

                # Place the data in a byte array at the correct locations
                num_events = ts.shape[0]
                out_events = np.zeros((num_events, 16), dtype=np.uint8)

                ts = np.frombuffer(ts, dtype=np.uint8)
                ts = ts.reshape(num_events, 8)
                out_events[:, 0:8] = ts

                x = np.frombuffer(x, dtype=np.uint8)
                x = x.reshape(num_events, 2)
                out_events[:, 8:10] = x

                y = np.frombuffer(y, dtype=np.uint8)
                y = y.reshape(num_events, 2)
                out_events[:, 10:12] = y

                pol = np.frombuffer(pol, dtype=np.uint8)
                pol = pol.reshape(num_events, 1)
                out_events[:, 12:13] = pol

                fout.write(out_events.tobytes())

    print("Finished importing all test data.")


if __name__ == '__main__':
    main()
