########################################
# Name: mspv1ctrl.py
# Func: MSPv1API for python3
# Auther: urmouky@outlook.com
# Date: 2023.6.18
########################################
import serial
import threading
import time
import struct
import keyboard
import colorama
import sys

colorama.init(autoreset=True)

class MSPCtrl():
    class _MSPCommands():
        MSP_NULL = 0                    # 0x00
        MSP_TEST = 1                    # 0x01
        MSP_MODE_RANGES = 34            # 0x22
        MSP_SET_MODE_RANGE = 35         # 0x23
        MSP_ADJUSTMENT_RANGES = 52      # 0x34
        MSP_SET_ADJUSTMENT_RANGE = 53   # 0x35
        MSP_IDENT = 100
        MSP_STATUS = 101
        MSP_RAW_IMU = 102
        MSP_SERVO = 103
        MSP_MOTOR = 104
        MSP_RC = 105
        MSP_RAW_GPS = 106
        MSP_COMP_GPS = 107
        MSP_ATTITUDE = 108
        MSP_ALTITUDE = 109
        MSP_ANALOG = 110
        MSP_BOX = 113
        MSP_MISC = 114
        MSP_BOXNAMES = 116
        MSP_BOXIDS = 119
        MSP_SET_RAW_RC = 200
        MSP_ACC_CALIBRATION = 205
        MSP_MAG_CALIBRATION = 206
        MSP_SET_MISC = 207
        MSP_SET_WP = 209
        MSP_SET_HEAD = 211

    class _MSP_READING_STEPS:
        """Enum of MSP States"""
        IDLE = 0
        HEADER_START = 1
        HEADER_M = 2
        HEADER_ARROW = 3
        HEADER_SIZE = 4
        HEADER_CMD = 5

    class _MSPResponse:
        """Combine MSP response data and finished communication flag"""
        def __init__(self):
            self.finished = False
            self.data = []
    
    class _MSP_WP_ACTION:
        ACT_WAYPOINT = 1
        ACT_POSHOLD = 3
        ACT_RTH = 4
        ACT_SETPOI = 5
        ACT_JUMP = 6
        ACT_SETHEAD = 7
        ACT_LAND = 8

    def __init__(self, debug=False, port:str="TTYS1"):
        self._timebase = time.time()
        self._serial = serial.Serial()
        self._lock = threading.RLock()
        self._monitorThread = threading.Thread(target=self._monitorSerialPort)
        self._exitNow = threading.Event()
        self._rcDaemonThread = threading.Thread(target=self._rcSender)
        self._stopRC = threading.Event()
        self._responses: dict[any, self._MSPResponse] = {}
        self._responseTimeout = 0.3
        self._debug = debug
        self._critical_error_count = 0
        self._error_max = 3     # 看门狗触发值
        self._port = port
        self._baudrate = 115200 
        self._serial = None
        self.RCChannels = {
            "pitch":1500,
            "yaw":1500,
            "roll":1500,
            "throttle":885,
            "aux1":1500,
            "aux2":1500,
            "aux3":1500,
            "aux4":1500
        }
        if self._debug:
            self._ReadStep = {
                0 : "IDLE",
                1 : "HEADER_START",
                2 : "HEADER_M",
                3 : "HEADER_ARROW",
                4 : "HEADER_SIZE",
                5 : "HEADER_CMD"
            }
        self.connect()

    def __del__(self):
        pass

    def _toInt16(self, data):
        if len(data) == 2:
            return struct.unpack("@h", struct.pack("<BB", data[0], data[1]))[0]
        else:
            return None
    
    def _toUInt16(self, data):
        if len(data) == 2:
            return struct.unpack("@H", struct.pack("<BB", data[0], data[1]))[0]
        else:
            return None
        
    def _toInt32(self, data):
        if len(data) == 4:
            return struct.unpack("@i", struct.pack("<BBBB", data[0], data[1], data[2], data[3]))[0]
        else:
            return None
    
    def _toUInt32(self, data):
        if len(data) == 4:
            return struct.unpack("@I", struct.pack("<BBBB", data[0], data[1], data[2], data[3]))[0]
        else:
            return None

    def _fromInt(self, value):
        return struct.unpack("<B", struct.pack("@h", value))

    def _fromInt16(self, value):
        return struct.unpack("<BB", struct.pack("@h", value))

    def _fromUInt16(self, value):
        return struct.unpack("<BB", struct.pack("@H", value))

    def _fromInt32(self, value):
        return struct.unpack("<BBBB", struct.pack("@i", value))

    def _fromUInt32(self, value):
        return struct.unpack("<BBBB", struct.pack("@I", value))

    def _timecheck(self):
        return round(time.time() - self._timebase, 7)
    
    def limit(value, min_val = 988, max_val = 2012):
        return max(min(value, max_val), min_val)

    def disconnect(self):
        """  """
        print("[{0}] [\033[0;33;40mWarning\033[0m]: Seriel port disconnected!".format(self._timecheck()))
        if self._monitorThread.is_alive():
            self._exitNow.set()
            # self._monitorThread.join()
        if self._serial == None:
            print("[{0}] [\033[0;31;40mError\033[0m]: Unknown port {1}.".format(self._timecheck(), self._port))
        else:
            self._serial.close()
        return True

    def connect(self):
        try:
            self._serial = serial.Serial(self._port, self._baudrate, timeout=2)
            # self._serial.open()
            try:
                self._exitNow.clear()
                self._monitorThread.start()
                if self._debug:
                    print("[{0}] [\033[0;34;40mDebug\033[0m]: Searial port {1} connected.".format(self._timecheck(), self._port))
                return True
            except:
                print("[{0}] [\033[0;31;40mError\033[0m]: Creating monitor thread failed.".format(self._timecheck()))
                return False
        except:
            print("[{0}] [\033[0;31;40mError\033[0m]: Opening serial port {1} failed.".format(self._timecheck(), self._port))
            return False
    
    # MSP V1 command frame has the structure:
    # <Starting Keyword> <Header Size> <Command> <Data> <Checksum>
    # <Starting Keyword>        uint8
    # <Header Size>             uint8
    # <Command>                 uint8
    # <Data>
    # <Checksum>                uint8

    def _monitorSerialPort(self):
        """This is a MSP decoding thread."""

        step = self._MSP_READING_STEPS.IDLE
        data = bytearray()
        datasize = 0
        dataCheckSum = 0
        command = self._MSPCommands.MSP_NULL
        print("[{0}] [\033[0;34;40mDebug\033[0m]: Entered _monitorSerialPort().".format(self._timecheck()))
        print(self._exitNow.is_set())
        while (not self._exitNow.is_set()):
            # Read the information byte-by-byte and set _MSP_READING_STEPS
            print(self._exitNow.is_set())
            try:
                if self._serial.in_waiting > 0:
                    inByte = ord(self._serial.read())
                    if step == self._MSP_READING_STEPS.IDLE:
                        if self._debug:
                            print("[{0}] [\033[0;34;40mDebug\033[0m]: Now step is {1}, inByte is {2}.".format(self._timecheck(), self._ReadStep[step], chr(inByte)), end="\n")
                        step = self._MSP_READING_STEPS.HEADER_START if (inByte==ord('$')) else self._MSP_READING_STEPS.IDLE #chr(36)=='$'
                    elif step == self._MSP_READING_STEPS.HEADER_START:
                        if self._debug:
                            print("[{0}] [\033[0;34;40mDebug\033[0m]: Now step is {1}, inByte is {2}.".format(self._timecheck(), self._ReadStep[step], chr(inByte)), end="\n")
                        step = self._MSP_READING_STEPS.HEADER_M if (inByte==ord('M')) else self._MSP_READING_STEPS.IDLE #chr(77)=='M'
                    elif step == self._MSP_READING_STEPS.HEADER_M:
                        if self._debug:
                            print("[{0}] [\033[0;34;40mDebug\033[0m]: Now step is {1}, inByte is {2}.".format(self._timecheck(), self._ReadStep[step], chr(inByte)), end="\n")
                        step = self._MSP_READING_STEPS.HEADER_ARROW if (inByte==ord('>')) else self._MSP_READING_STEPS.IDLE #chr(62)=='>'
                        if inByte == ord('!'):
                            print("[{0}] [\033[0;33;40mWarning\033[0m]: MSP error recieved.".format(self._timecheck()))
                            return False
                    elif step == self._MSP_READING_STEPS.HEADER_ARROW:
                        if self._debug:
                            print("[{0}] [\033[0;34;40mDebug\033[0m]: Now step is {1}, inByte is {2}.".format(self._timecheck(), self._ReadStep[step], inByte), end="\n")
                        datasize = inByte
                        data = bytearray()
                        dataCheckSum = inByte
                        step = self._MSP_READING_STEPS.HEADER_SIZE
                    elif (step == self._MSP_READING_STEPS.HEADER_SIZE):
                        if self._debug:
                            print("[{0}] [\033[0;34;40mDebug\033[0m]: Now step is {1}, inByte is {2}.".format(self._timecheck(), self._ReadStep[step], inByte), end="\n")
                        command = inByte
                        dataCheckSum = dataCheckSum ^ inByte
                        step = self._MSP_READING_STEPS.HEADER_CMD
                    elif (step == self._MSP_READING_STEPS.HEADER_CMD) and (len(data) < datasize):
                        if self._debug:
                            print("[{0}] [\033[0;34;40mDebug\033[0m]: Now step is {1}, inByte is {2}.".format(self._timecheck(), self._ReadStep[step], inByte), end="\n")
                        data.append(inByte)
                        dataCheckSum = (dataCheckSum ^ inByte)
                    elif (step == self._MSP_READING_STEPS.HEADER_CMD) and (len(data) >= datasize):
                        if self._debug:
                            print("[{0}] [\033[0;34;40mDebug\033[0m]: Now step is {1}, inByte is {2}.".format(self._timecheck(), self._ReadStep[step], inByte), end="\n")
                        if dataCheckSum == inByte :
                            self._commandProcess(command, data)
                        else:
                            pass
                        step = self._MSP_READING_STEPS.IDLE
                else:
                    time.sleep(0)
            except:
                self._critical_error_count += 1
                if self._critical_error_count > self._error_max:
                    self._error_handler("_monitorSerialPort")
        self._serial.close()

    def _commandProcess(self, command, data):
        """     """
        if self._debug:
            print("[{0}] [\033[0;34;40mDebug\033[0m]: Command: {1}.".format(self._timecheck(), command))
            # print("[{0}] [\033[0;34;40mDebug\033[0m]: self._responses={1}".format(self._timecheck(), self._responses))
        if command in self._responses:
            self._responses[command].data = data
            self._responses[command].finished = True
            if self._debug:
                print("[{0}] [\033[0;34;40mDebug\033[0m]: Recieved data={1}, writing to MSPCtrl._response[{2}].\n".format(self._timecheck(), data, command))
                print("[{0}] [\033[0;34;40mDebug\033[0m]: self._responses={1}".format(self._timecheck(), self._responses[command].data), end="\n")
            self._commandReceived(command, data)
            return True
        else:
            return False

    def _sendCommand(self, command, data=None):
        """     """
        if data is None:
            datasize = 0
        else:
            # MSP limited data length under 256 bytes.
            if len(data) < 256:
                datasize = len(data)
            else:
                print("[{0}] [\033[0;31;40mError\033[0m]: A packets longer than 256 bytes detected.".format(self._timecheck()))
                return False
        output = bytearray()
        output.append(ord('$'))
        output.append(ord('M'))
        output.append(ord('<'))
        output.append(datasize)
        checksum = datasize
        output.append(command)
        checksum = checksum ^ command
        if datasize > 0:
            for i in data:
                output.append(i)
                checksum = checksum ^ i
        output.append(checksum)
        try:
            self._serial.write(output)
            self._responses[command] = self._MSPResponse()
            if self._debug:
                print("[{0}] [\033[0;34;40mDebug\033[0m]: Command sent successfully, Frame is:{1}".format(self._timecheck(), output))
                print("[{0}] [\033[0;34;40mDebug\033[0m]: Updated self._responses, now is:{1}".format(self._timecheck(), self._responses))
        except Exception:
            self._critical_error_count += 1
            print("[{0}] [\033[0;31;40mError\033[0m]: Writing serial port {1} failed.".format(self._timecheck(), self._serial))
            if self._critical_error_count > self._error_max:
                self._error_handler("_sendCommand")
            return False
        return True
    
    
    
    def _error_handler(self, enternum):
        """ERROR_Handler
            If you enter this procedure, there is a serious problem with the communication link. 
            Include:
                Electromagnetic interference
                Connection error
                Equipment failure
                ...
            And this program will attempt to reconnect to the port.
        """
        self._critical_error_count = 0
        print("[{0}] [\033[0;31;40mError\033[0m]: ERROR_HANDLER was called by {1}, trying to reconnect serial port: {2}".format(self._timecheck(), enternum, self._port))
        self.disconnect()
        time.sleep(0.1)
        self._monitorThread = threading.Thread(target=self._monitorSerialPort)
        self.startControl()
        self.connect()


    def _waitForResponse(self, command):
        """     """
        if command in self._responses:
            startTime = time.time()
            while True:
                if self._responses[command].finished:
                    return True
                if time.time() - startTime > self._responseTimeout:
                    print("[{0}] [\033[0;33;40mWarning\033[0m]: Waiting for response timeout.".format(self._timecheck()))
                    self._critical_error_count += 1
                    if self._critical_error_count > self._error_max:
                        self._error_handler("_waitForResponse")
                    return False
                time.sleep(0)
        else:
            print("[{0}] [\033[0;33;40mWarning\033[0m]: Unknown command: {1}".format(self._timecheck(), command))
            return False

    def _sendAndWait(self, command, data=None):
        """     """
        if self._sendCommand(command, data):
            return self._waitForResponse(command)
        else:
            return False

    def _sendAndGet(self, command, exceptSize=None):
        """     """
        if self._sendAndWait(command):
            recData = self._responses[command].data
            # del self._responses[command]
            if exceptSize is not None:
                if len(recData) == exceptSize:
                    return recData
                else:
                    print("[{0}] [\033[0;31;40mError\033[0m]: A length error detected, excepted:{1} received:{2}".format(self._timecheck(), exceptSize, len(recData)))
                    return False
            else:
                return recData
        else:
            return None

    def _commandReceived(self, command, data, error=False):
        pass

    def startControl(self):
        """This method control rc controller thread start."""
        try:
            self._rcDaemonThread.start()
            if self._debug:
                print("[{0}] [\033[0;34;40mDebug\033[0m]: RC daemon thread started.".format(self._timecheck()))
        except:
            pass

    def stopControl(self):
        """This method control rccontroller thread stop.RC freshrate = 10-100Hz, 8Ch. support."""
        self._stopRC.set()
        self._exitNow.set()
        return self._stopRC.is_set()

    def _rcSender(self):
        """
            This is MSP controller thread function.
            When this daemon is started, 
            the remote control signal will be sent automatically and continuously (50Hz) by simply changing the channel value in MSPCtrl.RCChannels.
        """
        if self._debug:
            time_last = time.time()
            conter = 0
        while (self._stopRC.is_set):
            self.setRC(self.RCChannels)
            if self._debug:
                conter = conter + 1
                if (time.time() - time_last >= 1):
                    print("[{0}] [\033[0;34;40mDebug\033[0m]: RC sender FreshRATE: {1}".format(self._timecheck(), conter))
                    conter = 0
                    time_last = time.time()
                print("[{0}] [\033[0;34;40mDebug\033[0m]: RC has set to {1}.".format(self._timecheck(), self.RCChannels), end="\n")
            time.sleep(0.001)
        return False
    
    def _wgs84encoder(self, cord):
        cordmsp = [0, 0, 0]
        cordmsp[0] = int(cord[0] * 1e7)
        cordmsp[1] = int(cord[1] * 1e7)
        cordmsp[2] = int(cord[2])
        return cordmsp

    def getIdent(self):
        """Get identifying information from the device

        Returns:
            dict:
                {
                    "version": (int)
                    "type": (int)
                }
            See MULTITYPENAMES for a string representation of 'type'
        """
        mspVersion = 0
        quadType = 0
        recdata = self._sendAndGet(self._MSPCommands.MSP_IDENT)
        if recdata:
            mspVersion = recdata[0]
            quadType = recdata[1]
        #end if
        return {"version":mspVersion, "type":quadType}

    def getAnalog(self):
        """Get analog sensor data from the device

        Returns:
            dict
            {
                "vbat": (int)
                "powermetersum": (int)
                "rssi": (int)
                "amperage": (int)
            }
        """
        vbat = 0
        pms = 0
        rssi = 0
        amperage = 0
        rdata = self._sendAndGet(self._MSPCommands.MSP_ANALOG, 7)
        if rdata:
            vbat = rdata[0]
            pms = self._toUInt16(rdata[1:3])
            rssp = self._toUInt16(rdata[3:5])
            amperage = self._toUInt16(rdata[5:7])
        return {"vbat":vbat, "powermetersum":pms, "rssi":rssi, "amperage":amperage}

    def getAltitude(self):
        """
            !!!Warning: This method does not return the correct value for now!!!
        """
        alt = 0
        vari = 0
        recData = self._sendAndGet(self._MSPCommands.MSP_ALTITUDE, 10)
        if recData:
            alt = self._toInt32(recData[0:4])
            vari = self._toInt16(recData[4:6])
            return {"altitude": alt, "vari": vari}

    def getIMU(self):
        """Get raw IMU data from the device

        Returns:
            dict
            {
                "accx": (int)
                "accy": (int)
                "accz": (int)
                "gyrx": (int)
                "gyry": (int)
                "gyrz": (int)
                "magx": (int)
                "magy": (int)
                "magz": (int)
            }
            Values depend on the sensor that is installed, and will be zero for any sensor that is not available
        """
        acc = [0,0,0]
        gyr = [0,0,0]
        mag = [0,0,0]
        rdata = self._sendAndGet(self._MSPCommands.MSP_RAW_IMU, exceptSize=18)
        if rdata:
            acc[0] = self._toInt16(rdata[0:2])
            acc[1] = self._toInt16(rdata[2:4])
            acc[2] = self._toInt16(rdata[4:6])
            gyr[0] = self._toInt16(rdata[6:8])
            gyr[1] = self._toInt16(rdata[8:10])
            gyr[2] = self._toInt16(rdata[10:12])
            mag[0] = self._toInt16(rdata[12:14])
            mag[1] = self._toInt16(rdata[14:16])
            mag[2] = self._toInt16(rdata[16:18])
        #end if
        return {"accx":acc[0], "accy":acc[1], "accz":acc[2],
                "gyrx":gyr[0], "gyry":gyr[1], "gyrz":gyr[2],
                "magx":mag[0], "magy":mag[1], "magz":mag[2]}
    #end def getIMU

        data = bytearray()
        throttle = 0
        pitch = 0

    def getRC(self):
        """Get current RC data from the device
        Returns:
            dict
            {
                "pitch": (int)
                "roll": (int)
                "yaw": (int)
                "throttle": (int)
                "aux1": (int)
                "aux2": (int)
                "aux3": (int)
                "aux4": (int)
            }
        Note: These are the PWM values being used to compute the signal to be sent to the motor. In CleanFlight,
            they should be between 1000 and 2000.
        """
        pitch = 0
        roll = 0
        yaw = 0
        throttle = 0
        aux = [0,0,0,0]
        rdata = self._sendAndGet(self._MSPCommands.MSP_RC)
        if rdata:
            if (len(rdata) >= 8):
                pitch = self._toUInt16(rdata[0:2])
                roll = self._toUInt16(rdata[2:4])
                yaw = self._toUInt16(rdata[4:6])
                throttle = self._toUInt16(rdata[6:8])
                for i in range(0, 3):
                    if (len(rdata) >= (10 + 2 * i)):
                        aux[i] = self._toUInt16(rdata[8+2*i:10+2*i])
                #end for
            #end if
        #end if
        return {"pitch":pitch, "roll":roll, "yaw":yaw, "throttle":throttle,
                "aux1":aux[0], "aux2":aux[1], "aux3":aux[2], "aux4":aux[3]}

    def getGPS(self):
        """Get GPS coordinate and fix data from the device

        Returns:
            dict
            {
                "fix": (bool)
                "numsat": (int)
                "latitude": (int)
                "longitude": (int)
                "altitude": (int)
                "speed": (int)
                "course": (int)
            }
        """
        gpsFix = False
        gpsNumSat = 0
        gpsLat = 0
        gpsLong = 0
        gpsAltitude = 0
        gpsSpeed = 0
        gpsCourse = 0
        rdata = self._sendAndGet(self._MSPCommands.MSP_RAW_GPS, 18)
        if rdata:
            gpsFix = True if (rdata[0] == 1) else False
            gpsNumSat = rdata[1]
            gpsLat = self._toUInt32(rdata[2:6])
            gpsLong = self._toUInt32(rdata[6:10])
            gpsAltitude = self._toUInt16(rdata[10:12])
            gpsSpeed = self._toUInt16(rdata[12:14])
            gpsCourse = self._toUInt16(rdata[14:16])
        #end if
        return {"fix":gpsFix, "numsat":gpsNumSat, "latitude":gpsLat, "longitude":gpsLong,
                "altitude":gpsAltitude, "speed":gpsSpeed, "course":gpsCourse}


    def _setWP(self, wp_no: int, act: int, cord: list, navp1: int, navp2: int, navp3: int, nav_flag: int = 0xa5):
        """Sends new waypoint to FC.
        Args:
            (uint8)  wp_no   : Waypoint serial number
            (uint8)  action  : 0~8 
            (uint32) cord[3] : Coordinates array, it mast has 3 parameters:
                        (uint32)latitude  : WGS84 floating point values multiplied by 1e7.
                        (uint32)longitude : WGS84 floating point values multiplied by 1e7.
                        (uint32)altitude  : in centimetres.
            (uint16) navp1   : Varies according to action
            (uint16) navp2   : Varies according to action
            (uint16) navp3   : Varies according to action
            (uint8)  flag    : 0xa5 = last, otherwise set to 0 (or 0x48 (72) for FBH WP, INAV 3.1+)
        Returns:
            bool: True if successful, False otherwise
        Notes:
            This method has not tested yet, take careful.
        """
        
        data = bytearray()
        lat = cord[0]
        lon = cord[1]
        alt = cord[2]
        r = self._fromInt(wp_no)
        data.append(r[0])
        r = self._fromInt(act)
        data.append(r[0])
        r = self._fromInt32(lat)
        data.append(r[0])
        data.append(r[1])
        data.append(r[2])
        data.append(r[3])
        r = self._fromInt32(lon)
        data.append(r[0])
        data.append(r[1])
        data.append(r[2])
        data.append(r[3])
        r = self._fromInt32(alt)
        data.append(r[0])
        data.append(r[1])
        data.append(r[2])
        data.append(r[3])
        r = self._fromInt16(navp1)
        data.append(r[0])
        data.append(r[1])
        r = self._fromInt16(navp2)
        data.append(r[0])
        data.append(r[1])
        r = self._fromInt16(navp3)
        data.append(r[0])
        data.append(r[1])
        r = self._fromInt(nav_flag)
        data.append(r[0])
        return self._sendAndWait(self._MSPCommands.MSP_SET_WP)
    

    def setWPtool(self, seqnum: int, action: _MSP_WP_ACTION, cord: list[3], p1: int, p2: int, p3: int):
        """It's a easy waypoint planner tool.
        Args:
            (int8)  seqnum: 
            (int8)  action: you can input values from (class)_MSP_WP_ACTION. Available values are:        
                            ACT_WAYPOINT: Normal waypoint set, the uav will pass the coordinate, then p1 is speed(cm/s), p3 is alt mode.
                            ACT_POSHOLD : Position Hold waypoint, the uav will go to the cord and wait a while, then p1 is hold time(s), p2 is speed(cm/s), p3 is alt mode.
                            ACT_RTH     : 
                            ACT_SETPOI  :
                            ACT_JUMP    : 
                            ACT_SETHEAD :
                            ACT_LAND    :
            
        """
        
        if action == self._MSP_WP_ACTION.ACT_WAYPOINT:
            cordmsp = self._wgs84encoder(cord)
            p1m = p1     # speed(cm/s)
            p2m = 0      # NC
            p3m = p3     # alt mode(Relative/Absolute)
        elif action == self._MSP_WP_ACTION.ACT_JUMP:
            cordmsp = [0,0,0]
            p1m = p1     # target wp number
            p2m = p2     # repeat times
            p3m = 0      # NC
        elif action == self._MSP_WP_ACTION.ACT_POSHOLD:
            cordmsp = self._wgs84encoder(cord)
            p1m = p1     # hold time
            p2m = p2     # speed(cm/s)
            p3m = p3     # alt mode(Relative/Absolute)
        elif action == self._MSP_WP_ACTION.ACT_RTH:
            cordmsp = [0,0,0]
            p1m = p1     # land switch
            p2m = 0      # NC
            p3m = 0      # NC
        elif action == self._MSP_WP_ACTION.ACT_LAND:
            cordmsp = self._wgs84encoder(cord)
            p1m = p1     # speed
            p2m = p2     # elevation adjustment
            p3m = p3     # alt mode(Relative/Absolute)
        try:
            verify = self._setWP(wp_no=seqnum, act=action, cord=cordmsp, navp1=p1m, navp2=p2m, navp3=p3m)
            if verify == True:
                print("[{0}] [\033[0;34;40mDebug\033[0m]: NAV_WP No.{1} has uploaded.".format(self._timecheck(), seqnum))
            else:
                print("[{0}] [\033[0;31;40mError\033[0m]: NAV_WP No.{1} uploaded failed.".format(self._timecheck(), seqnum))
        except:
            print("[{0}] [\033[0;31;40mError\033[0m]: Critical error occured while trying to uploade an NAV_WP.".format(self._timecheck(), seqnum))
    
    


    def setRC(self, values):
        """Sends new RC values to the device.
        Args:
            values (dict): 
                New RC values: containing values for one or more of the RC channels.
                Available channels are: "pitch", "yaw", "roll", "throttle", "aux1", "aux2",
                "aux3", and "aux4". Values not specified will be assumed to be zero.sdhsudhusdhusdhusadhiasudhliafuhsakudgaweukrbwuhwduihwiuwhedxliEUHCBASEIRUCbrli
                
        Returns:
            bool: True if successful, False otherwise
            
        Notes:
            This method does not get the current values for any channel before setting them.
            If you want to change a couple of values and keep the rest the same, call getRC()
            first, modify the values it returns, and pass that to setRC.
        """
        data = bytearray()
        throttle = 0
        pitch = 0
        yaw = 0
        roll = 0
        aux = [0,0,0,0]
        if isinstance(values, dict):
            if "pitch" in values:
                pitch = values["pitch"]
            if "roll" in values:
                roll = values["roll"]
            if "yaw" in values:
                yaw = values["yaw"]
            if "throttle" in values:
                throttle = values["throttle"]
            for i in range(1,4):
                if ("aux" + str(i)) in values:
                    aux[i-1] = values["aux" + str(i)]
            #end for
            r = self._fromInt16(roll)
            data.append(r[0])
            data.append(r[1])
            r = self._fromInt16(pitch)
            data.append(r[0])
            data.append(r[1])
            r = self._fromInt16(throttle)
            data.append(r[0])
            data.append(r[1])
            r = self._fromInt16(yaw)
            data.append(r[0])
            data.append(r[1])
            for i in range(0,3):
                r = self._fromInt16(aux[i])
                data.append(r[0])
                data.append(r[1])
            #end for
            return self._sendAndWait(self._MSPCommands.MSP_SET_RAW_RC, data)
        else:
            return False



# Keys Init

press_up = keyboard.KeyboardEvent("down", 38, "Up")             # 方向键"上" 按下
release_up = keyboard.KeyboardEvent("up", 38, "Up")
press_down = keyboard.KeyboardEvent("down", 40, "Down")         # 方向键"下" 按下
release_down = keyboard.KeyboardEvent("up", 38, "Down")

press_left = keyboard.KeyboardEvent("down", 37, "Left")
release_left = keyboard.KeyboardEvent("up", 37, "Left")
press_right = keyboard.KeyboardEvent("down", 39, "Right")
release_right = keyboard.KeyboardEvent("up", 39, "Right")

press_w = keyboard.KeyboardEvent("down", 87, "w")
release_w = keyboard.KeyboardEvent("up", 87, "w")
press_s = keyboard.KeyboardEvent("down", 83, "s")
release_s = keyboard.KeyboardEvent("up", 83, "s")

press_a = keyboard.KeyboardEvent("down", 65, "a")
release_a = keyboard.KeyboardEvent("up", 65, "a")
press_d = keyboard.KeyboardEvent("down", 68, "d")
release_d = keyboard.KeyboardEvent("up", 68, "d")

press_space = keyboard.KeyboardEvent("down", 65, "Space")
release_space = keyboard.KeyboardEvent("up", 65, "Space")
press_m = keyboard.KeyboardEvent("down", 68, "m")
release_m = keyboard.KeyboardEvent("up", 68, "m")

press_g = keyboard.KeyboardEvent("down", 65, "g")
press_b = keyboard.KeyboardEvent("down", 65, "b")
press_n = keyboard.KeyboardEvent("down", 65, "n")
press_q = keyboard.KeyboardEvent("down", 65, "q")

print("Press {0}/{1} to control pitch.".format(press_up.name, press_down.name))
print("Press {0}/{1} to control roll.".format(press_left.name, press_right.name))
print("Press {0}/{1} to control Throttle.".format(press_w.name, press_s.name))
print("Press {0}/{1} to control yaw.".format(press_a.name, press_d.name))
print("Press {0} to switch Arm/Disarm.".format(press_space.name))
print("Press {0} to switch Arm/Disarm.".format(press_m.name))



def pressCallback(x):
    """
        Simple keboard operating callback function: Dynamically modify the value of RCChannel.
        The following code is required:
        keyboard.hook(pressCallback)
        keyboard.wait()
    """

    # Axis Pitch控制: 方向键↑、↓
    if x.event_type == 'down' and x.name == press_up.name:
        MSP1.RCChannels["pitch"] = 1750
    elif x.event_type == 'up' and x.name == release_down.name:
        MSP1.RCChannels["pitch"] = 1500
    elif x.event_type == 'down' and x.name == press_down.name:
        MSP1.RCChannels["pitch"] = 1250
    elif x.event_type == 'up' and x.name == release_up.name:
        MSP1.RCChannels["pitch"] = 1500
        
    # Axis Roll控制: 方向键←、→
    elif x.event_type == 'down' and x.name == press_left.name:
        MSP1.RCChannels["roll"] = 1250
    elif x.event_type == 'up' and x.name == release_left.name:
        MSP1.RCChannels["roll"] = 1500
    elif x.event_type == 'down' and x.name == press_right.name:
        MSP1.RCChannels["roll"] = 1750
    elif x.event_type == 'up' and x.name == release_right.name:
        MSP1.RCChannels["roll"] = 1500
        
    # Axis Throttle: 字母键W、S
    elif x.event_type == 'down' and x.name == press_w.name:
        MSP1.RCChannels["throttle"] += 20
        print("[\033[0;32;40mInfo\033[0m]: Current throttle: {0}.".format(MSP1.RCChannels["throttle"]))
    elif x.event_type == 'up' and x.name == release_w.name:
        # MSP1.RCChannels["throttle"] += 10
        pass
    elif x.event_type == 'down' and x.name == press_s.name:
        MSP1.RCChannels["throttle"] -= 20
        print("[\033[0;32;40mInfo\033[0m]: Current throttle: {0}.".format(MSP1.RCChannels["throttle"]))
    elif x.event_type == 'up' and x.name == release_s.name:
        # MSP1.RCChannels["throttle"] = 1500
        pass
    
    # Axis yaw: 字母键A、D
    elif x.event_type == 'down' and x.name == press_a.name:
        MSP1.RCChannels["yaw"] = 1250
    elif x.event_type == 'up' and x.name == release_a.name:
        MSP1.RCChannels["yaw"] = 1500
    elif x.event_type == 'down' and x.name == press_d.name:
        MSP1.RCChannels["yaw"] = 1750
    elif x.event_type == 'up' and x.name == release_d.name:
        MSP1.RCChannels["yaw"] = 1500
    elif x.event_type == 'up' and x.name == press_n.name:
        MSP1.RCChannels["throttle"] = 1500
        
    # Mode switch: 模式切换M
    elif x.event_type == 'down' and x.name == press_space.name:
        if MSP1.RCChannels["aux1"] == 1350:
            MSP1.RCChannels["aux1"] = 1650
            print("[\033[0;32;40mInfo\033[0m]: Armmed.")
        else:
            MSP1.RCChannels["aux1"] = 1350
            print("[\033[0;32;40mInfo\033[0m]: Disarmmed.")
    elif x.event_type == 'up' and x.name == release_space.name:
        # MSP1.RCChannels["aux1"] = 1500
        pass
    elif x.event_type == 'down' and x.name == press_m.name:
        if MSP1.RCChannels["aux2"] == 1350:
            MSP1.RCChannels["aux2"] = 1650
            print("[\033[0;32;40mInfo\033[0m]: Mode set to NAV_Pos_Hold")
        else:
            MSP1.RCChannels["aux2"] = 1350
            print("[\033[0;32;40mInfo\033[0m]: Mode set to NAV_Alt_Hold")
    elif x.event_type == 'up' and x.name == release_m.name:
        # MSP1.RCChannels["aux2"] = 1500
        pass
    
    # GPS Info: 获取GPS信息G
    elif x.event_type == 'down' and x.name == press_g.name:
        # try:
        #     num = MSP1.getAltitude()
        #     print("[\033[0;32;40mInfo\033[0m]: Altitude got: {0}: ".format(num))
        # except:
        #     print("[\033[0;33;40mWarning\033[0m]: Get altitude failed.")
        # time.sleep(0.2)
        try:
            num = MSP1.getGPS()
            print("[\033[0;32;40mInfo\033[0m]: GPS got: {0}".format(num))
        except:
            print("[\033[0;33;40mWarning\033[0m]: Get GPS failed.")
            
    # Analog: 获取模拟量信息B
    elif x.event_type == 'down' and x.name == press_b.name:
        try:
            num = MSP1.getAnalog()
            print("[\033[0;32;40mInfo\033[0m]: Analog got: {0}".format(num))
        except:
            print("[\033[0;33;40mWarning\033[0m]: Get Analog failed.")

    elif x.event_type == 'down' and x.name == press_q.name:
        MSP1.stopControl()
        print("[\033[0;32;40mInfo\033[0m]: RC sender stopped. ")


 
if __name__ == "__main__":
    dbgmode = 0
    
    # with open('output.txt', 'w') as logfile:
    #     sys.stdout = logfile
        
    if dbgmode == 0:
        # 调试模式，请把debug置位
        port_input = str(input("MSP port(For windows is 'COMx', for linux is '/dev/ttySx'): "))
        MSP1 = MSPCtrl(debug=False, port=port_input if port_input else "COM3")
        # MSP1.connect(baudrate=57600)
        MSP1.startControl()
        keyboard.hook(pressCallback)
        keyboard.wait()
    
    elif dbgmode == 1:
        MSP1 = MSPCtrl(debug=False)
        MSP1.connect()
        num = MSP1.getGPS()
        print("GPS got: {0}".format(num))
    
    elif dbgmode == 2:
        MSP1 = MSPCtrl(debug=True)
        MSP1.connect()
        while(1):
            num = MSP1.getAltitude()
            print("Altitude got: {0}: ".format(num))
            time.sleep(1)
