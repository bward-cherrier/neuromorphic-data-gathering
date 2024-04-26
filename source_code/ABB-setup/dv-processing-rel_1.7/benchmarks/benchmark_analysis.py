#!/usr/bin/env python3

import os
import json
import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser(
    description='Process and plot the results from a benchmark run using google-benchmark, stored in a json file')
parser.add_argument('files', nargs='+', help='The json files to be processed')
parser.add_argument('--out_dir', '-o', help='The directory to store the generated figures in')
args = parser.parse_args()
data = {}

files = args.files
out_dir = args.out_dir

if out_dir is not None and ~out_dir.endswith("/"):
    out_dir += "/"

for file in files:
    with open(file) as benchmark_file:
        json_data = json.load(benchmark_file)
        for bm in json_data["benchmarks"]:
            if ("BigO" not in bm["name"] and "RMS" not in bm["name"]):
                name_and_params = bm["name"].split("/")
                benchmark_name = name_and_params[0]

                if len(name_and_params) > 2:
                    raise RuntimeError("Currently only one range parameter is supported.")

                if benchmark_name not in data:
                    data[benchmark_name] = np.empty([1 + len(name_and_params) - 1, 0])

                new_data = np.append(np.array([[float(bm["cpu_time"])]]),
                                     np.array([name_and_params[1:]], dtype=float),
                                     axis=0)
                data[benchmark_name] = np.append(data[benchmark_name], new_data, axis=1)

    for bm in data:
        plt.plot(data[bm][1, :], data[bm][0, :] * 10**-9, label=bm)

    plt.title(file.split("/")[-1])
    plt.xlabel("data size")
    plt.ylabel("cpu time / s")
    plt.legend()

    if (out_dir is not None):
        out_file = out_dir + os.path.splitext(file.split("/")[-1])[0] + ".png"
        print("saving into file: " + out_file)
        plt.savefig(out_file, format="png")
    else:
        out_file = os.path.splitext(file)[0] + ".png"
        print("saving into file: " + out_file)
        plt.savefig(out_file, format="png")
