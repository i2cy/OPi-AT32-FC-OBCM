#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: i2cy(i2cy@outlook.com)
# Project: OPi-AT32-FC-OBCM
# Filename: blackbox
# Created on: 2024/5/10

import queue
from ctypes import c_bool
from multiprocessing import Process, Queue, Value
from typing import Any

from i2cylib.filesystem import IcFAT
from i2cylib.engineering import PID
import time
import struct
import os

"""
BlackBox File Format:

Using IcFAT virtual filesystem in practise. Log up to 255 columns of data. Max size per file is 512 MB.

sub_file_tree:
    - meta_data.bin: saved headers and datatype in frame that seperated with 0xff
        <str column_name> <0xfe> <str data type> <0xff>
    - data.bin: saved blackbox log
        <0xff> <double time_stamp> <uint8 column_index> <Any data_payload>...(repeat <uint8 column_index> <Any data_payload>)
"""

MAX_TIME_WAIT_TO_WRITE_SEC = 0.5
MAX_DATA_LENGTH_WAIT_TO_WRITE = 16384
META_FILENAME = "meta_data.bin"
DATA_FILENAME = "data.bin"
__VERSION__ = "1.0"


class BlackBoxLogger:

    def __init__(self, filename=None):
        """
        signals logger
        :param filename:
        """

        self.mp_live = Value(c_bool, 0)
        self.mp_log_queue = Queue()  # format: (time.time(), {'column_A': <column_data>, 'column_B'.....})
        self.mp_cmd_queue = Queue()  # format: {<command_name>: <Any payload>}
        if filename is None:
            filename = "OPi-AT32-blackbox_{}.obbl".format(time.strftime("%Y%m%d-%H%M%S"))
        self.filename = filename
        self.__processes = []
        self.__start_ts = 0.0

    def __proc_file_writer(self):
        """
        sub-process file writer
        :return:
        """

        filename = None
        file_io = None
        header_nums = 0
        headers = {}
        cache = b""
        t_last_write = time.time()

        while self.mp_live.value:

            # handling command
            try:
                cmd = self.mp_cmd_queue.get_nowait()  # fetch command if there is any

                # process command
                for ele in cmd.keys():
                    if ele == "set_filename":
                        # close old file if opened
                        if file_io is not None:
                            file_io.release()
                            file_io = None
                        # change filename
                        filename = cmd[ele]
                        header_nums = 0
                        headers = {}
                        cache = b""
            except queue.Empty:
                pass

            # open file
            if file_io is None:
                if filename is not None:
                    # initiate IcFat manager if filename changes
                    file_io = IcFAT(filename)
                    file_io.makefs(16384, cluster_length=32768, description="OPi-FC_BB_V{}".format(__VERSION__))
                    f = file_io.open("meta_data.bin", "w")
                    f.close()
                    f = file_io.open("data.bin", "w")
                    f.close()
                else:
                    # stop recording if no file opened
                    time.sleep(0.02)
                    continue

            # get data frame from log queue
            try:
                data = self.mp_log_queue.get(block=True, timeout=0.2)

                cache += b"\xff" + struct.pack("<d", data[0])  # append timestamp first
                # preprocess data to bytes
                for ele in data[1].keys():
                    # get column meta
                    try:
                        meta = headers[ele]  # format: [<column_index>, <data_format_str>]
                    except KeyError:
                        # create new header meta if header does not exist
                        meta = [header_nums]
                        header_nums += 1
                        # determine the data format
                        if isinstance(data[1][ele], int):
                            meta.append("i")
                        else:
                            meta.append("d")
                        headers.update({ele: meta})

                        # write header meta to log file
                        meta_file = file_io.open(META_FILENAME, "a")
                        meta_file.seek(meta_file.length)
                        meta_file.write(ele.encode("utf-8") + b"\xfe" + meta[1].encode("utf-8") + b"\xff")
                        meta_file.close()
                        file_io.release()  # update changes immediately to the real filesystem

                    # append packed data to cache
                    cache += struct.pack("<B{}".format(meta[1]), meta[0], data[1][ele])
            except queue.Empty:
                pass

            # write cached data
            if len(cache):
                t1 = time.time()
                if len(cache) > MAX_DATA_LENGTH_WAIT_TO_WRITE or t1 - t_last_write > MAX_TIME_WAIT_TO_WRITE_SEC:
                    # write data
                    data_file = file_io.open(DATA_FILENAME, "a")
                    data_file.seek(data_file.length)
                    data_file.write(cache)
                    data_file.close()
                    file_io.release()  # update changes immediately to the real filesystem

                    # reset buffer
                    cache = b""
                    t_last_write = t1

        print(headers)

    def set_filename(self, filename):
        self.mp_cmd_queue.put({'set_filename': filename})
        self.filename = filename

    def start(self):
        """
        start black box logger and all of its sub processes
        :return:
        """
        self.set_filename(self.filename)
        self.__processes.append(Process(target=self.__proc_file_writer))
        self.mp_live.value = True
        self.__start_ts = time.time()
        [ele.start() for ele in self.__processes]

    def kill(self):
        """
        kill all processes
        :return:
        """
        self.mp_live.value = False
        [ele.join() for ele in self.__processes]
        self.__processes = []

    def log(self, messages: dict):
        """
        log one frame of blackbox data to blackbox. message format should be restricted as shown
        :param messages: {<str column_nameA>: <int/float payload>, <str column_nameB>...}
        :return:
        """
        t1 = time.time() - self.__start_ts
        self.mp_log_queue.put((t1, messages))


class Trace(object):

    def __init__(self, file_io: IcFAT, all_meta_list, index):
        """
        2D array, [(timestamp_1, data_1), (timestamp_2, data_2), ...]
        :param file_io: IcFat object
        :param all_meta_list: meta list
        :param index: trace column index
        """
        self.name = all_meta_list[index][0]
        self.__file = file_io.open(DATA_FILENAME, "r")
        self.__column_index = index
        self.__meta_list = all_meta_list
        self.__length = None

    def __iter__(self):
        self.__index = 0
        return self

    def __next__(self):
        ts = None
        data = None
        while True:
            # fetch header
            header = self.__file.read(1)
            if header == b"":
                # stop iterate if EOF met
                self.__length = self.__index
                raise StopIteration

            elif header == b"\xff":
                # read timestamp
                raw = self.__file.read(8)
                if len(raw) < 8:
                    # stop iterate if EOF met
                    self.__length = self.__index
                    raise StopIteration
                ts = struct.unpack("<d", raw)[0]

            elif header[0] < len(self.__meta_list):
                # read payload
                r_len = 4
                if self.__meta_list[header[0]][1] == 'd':
                    # 8 bytes if dtype is double
                    r_len = 8

                if header[0] == self.__column_index:
                    # read data selectively
                    raw = self.__file.read(r_len)
                    if len(raw) < r_len:
                        # stop iterate if EOF met
                        self.__length = self.__index
                        raise StopIteration
                    data = struct.unpack("<{}".format(self.__meta_list[header[0]][1]), raw)[0]
                    if ts is None:
                        # skip broken frame
                        continue
                    self.__index += 1
                    break  # break if selective data found
                else:
                    try:
                        # skip next payload
                        self.__file.seek(self.__file.tell() + r_len)
                    except Exception:
                        # stop iterate if EOF met
                        self.__length = self.__index
                        raise StopIteration

            else:
                # skip broken frame
                continue

        return ts, data


class BlackBoxReader:

    def __init__(self, filename: str):
        """
        read and decode blackbox file of OPi-AT32-FC
        :param filename: str
        """
        if os.path.exists(filename) and os.path.isfile(filename):
            self.file_io = IcFAT(filename)
        else:
            raise FileNotFoundError("\"{}\" not found".format(filename))
        if self.file_io.description != "OPi-FC_BB_V{}".format(__VERSION__):
            raise RuntimeError("file in IcFAT format exists. But the file does not seems to be a BlackBox file, "
                               "file description: {}".format(self.file_io.description))

        # retrieve header meta
        meta_file = self.file_io.open(META_FILENAME, "r")
        meta_raw = meta_file.read()
        meta_file.close()
        if not len(meta_raw):
            raise RuntimeError("empty blackbox file: meta empty")
        meta_raw = meta_raw.split(b"\xff")[:-1]
        self.meta = [(ele.split(b"\xfe")[0].decode(), ele.split(b"\xfe")[1].decode()) for ele in meta_raw]

    def list_columns(self):
        """
        list all column names in this blackbox log
        :return: [<column_A>, <column_B>...]
        """
        return [ele[0] for ele in self.meta]

    def get_trace(self, name):
        """
        get trace iterator of given trace name
        :param name: str
        :return: Trace iterable
        """
        index = 0
        found = False
        for index, ele in enumerate(self.meta):
            if ele[0] == name:
                found = True
                break

        if not found:
            raise IndexError("trace with name \"{}\" not found in BlackBox log".format(name))

        return Trace(self.file_io, self.meta, index)


if __name__ == "__main__":  # unit test
    import threading
    import random
    import math
    import matplotlib.pyplot as plt

    test_time = 100


    def test_logger_thread(logger: BlackBoxLogger, log_content: list, dt: float):
        for ele in log_content:
            logger.log({str(len(log_content)): ele})
            time.sleep(dt)


    print("initializing blackbox logger...")
    logger = BlackBoxLogger()
    logger.start()

    print("generating random curve, total test log time: {} s".format(test_time))
    log_1k = [random.randint(-2147483648, 2147483647) for _ in range(10000)]
    log_2k = [random.randint(-2147483648, 2147483647) for _ in range(20000)]
    log_3k = [random.randint(-2147483648, 2147483647) for _ in range(30000)]
    log_4K_sin = [math.sin(i / (5000 * math.pi)) for i in range(4000 * 100)]
    log_4K_cos = [math.cos(i / (5000 * math.pi)) for i in range(4000 * 100)]

    print("running all sub threads to test blackbox log...")

    threads = [threading.Thread(target=test_logger_thread, args=(logger, log_1k, test_time / len(log_1k))),
               threading.Thread(target=test_logger_thread, args=(logger, log_2k, test_time / len(log_2k))),
               threading.Thread(target=test_logger_thread, args=(logger, log_3k, test_time / len(log_3k)))]

    [ele.start() for ele in threads]

    for i, ele in enumerate(log_4K_sin):
        logger.log({"log_4K_sin": ele, "log_4K_cos": log_4K_cos[i]})
        time.sleep(test_time / len(log_4K_sin))

    [ele.join() for ele in threads]

    print("log complete, stopping blackbox logger")
    logger.kill()

    reader = BlackBoxReader(logger.filename)
    print("all traces in blackbox log: {}".format(reader.list_columns()))
    print("verifying blackbox log of trace: 10000......", end="")
    for i, ele in enumerate(reader.get_trace("10000")):
        if log_1k[i] != ele[1]:
            print("failed at frame {}. saw {}, expecting {}".format(i, ele[1], log_1k[i]))
            break
    print("\nverifying blackbox log of trace: 20000......", end="")
    for i, ele in enumerate(reader.get_trace("20000")):
        if log_2k[i] != ele[1]:
            print("failed at frame {}. saw {}, expecting {}".format(i, ele[1], log_2k[i]))
            break
    print("\nverifying blackbox log of trace: 30000......", end="")
    for i, ele in enumerate(reader.get_trace("30000")):
        if log_3k[i] != ele[1]:
            print("failed at frame {}. saw {}, expecting {}".format(i, ele[1], log_3k[i]))
            break
    print("\nverifying blackbox log of trace: log_40K_sin......", end="")
    x_sin = []
    y_sin = []
    for i, ele in enumerate(reader.get_trace("log_4K_sin")):
        x_sin.append(ele[0])
        y_sin.append(ele[1])
        if log_4K_sin[i] != ele[1]:
            print("failed at frame {}".format(i))
            break
    x_cos = []
    y_cos = []
    print("\nverifying blackbox log of trace: log_40K_cos......", end="")
    for i, ele in enumerate(reader.get_trace("log_4K_cos")):
        x_cos.append(ele[0])
        y_cos.append(ele[1])
        if log_4K_cos[i] != ele[1]:
            print("failed at frame {}".format(i))
            break

    print("\ndone")

    plt.plot(x_cos, y_cos)
    plt.plot(x_sin, y_sin)
    plt.show()
