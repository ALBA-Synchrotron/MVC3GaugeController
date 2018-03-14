#!/usr/bin/env python

#    "$Name:  $";
#    "$Header: $";
#=============================================================================
#
# file :        MVC3GaugeController.py
#
# description : Python source for the MVC3GaugeController and its commands. 
#                The class is derived from Device. It represents the
#                CORBA servant object which will be accessed from the
#                network. All commands which can be executed on the
#                MVC3GaugeController are implemented in this file.
#
# project :    VacuumController Device Server
#
# $Author: jmoldes@cells.es $
#
# $Revision: $
#
# $Log: $
#
#
# copyleft :    Cells / Alba Synchrotron
#               Bellaterra
#               Spain
#
############################################################################
#
# This file is part of Tango-ds.
#
# Tango-ds is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# Tango-ds is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
##########################################################################


#Alba imports
import PyTango

#standard python imports
import sys
import time
import threading
import traceback
import math
import re

#==================================================================
#
# Utility auxiliary classes
#
#==================================================================


class Channel(object):

    def __init__(self, parent=None):
        self.parent = parent
        object.__init__(self)
        self.number = -1
        self.value = float('NaN')
        self.unit = ''
        self.read_validity = 0 #always read from hardware by default
        self.last_read = 0


    def read(self):
        if self.parent == None:
            raise Exception('In %: no parent specified' % self.__class__.__name__)

        now = time.time()
        if (now - self.last_read) > self.read_validity:
            #read from the hardware
            try:
                self.last_read = now
                results = self.parent._communicate_raw(self.parent.CMD_RPV % self.number, output_expected=True)
            except Exception, e:
                msg = 'In %s::read() error reading channel %s: %s' % (self.__class__.__name__, self.number, str(e))
                self.parent.error_stream(msg)
                PyTango.Except.throw_exception('Communication error', msg, '%s::read()' % self.__class__.__name__)

            #communication worked, so try to parse the result
            ERROR_STATE = PyTango.DevState.FAULT if int(self.number)==1 else PyTango.DevState.ALARM
            try:
                if results.startswith(self.parent.ERR_CHARACTER):
                    match = self.parent.ERR_RE.match(results)
                    values = match.groups()
                    err_code = values[0]
                    if len(values) > 1:
                        err_param = values[1]
                    msg = self.parent.ERR_DICT[err_code] % err_param
                    msg += ' (detected %s)' % time.ctime()
                    self.parent.error_stream(msg)
                    self.parent._set_state(ERROR_STATE,msg)
                    read_press = 'NaN'
                else:
                    match = self.parent.CMD_RPV_RE.match(results)
                    values = match.groups()
                    err_code = int(values[0])
                    if err_code > 0:
                        read_press = 'NaN'
                        msg = 'Error reading channel %s: %s' % (self.number, self.parent.ERR_RPV_DICT[err_code])
                        self.parent.error_stream(msg)
                        self.parent._set_state(ERROR_STATE,msg)
                    else:
                        read_press = values[1]
                value = float(read_press)
            except Exception, e:
                value = float('NaN')
                msg = 'Unexpected exception while reading the value pressure: %s' % traceback.format_exc()
                self.parent.error_stream(msg)
                self.parent._set_state(ERROR_STATE, msg)
            finally:
                self.value = value


#==================================================================
# MVC3GaugeController Class Description:
#
# This device reads the pressure of the channels of a Vacom MVC-3
# vacuum gauge controller
#
#==================================================================


class MVC3GaugeController(PyTango.Device_4Impl):

#--------- Add you global variables here --------------------------

    #known serial device classes
    KNOWN_SERIAL_CLASSES = ['Serial', 'PySerial']

    #channels constants
    CHANNELS = ['1', '2', '3']

    #commands and answers (if applicable)
    CMD_RPV = 'RPV%0d' #read pressure value for channel %d
    CMD_RPV_RE = re.compile('([0-9]+)(?:,\t([0-9]+.[0-9]+E[+-][0-9]+))?')
    CMD_LOCK =  'SKL%0d' #set key lock ON/OFF (1/0)

    #error constants
    ERR_CHARACTER = '?'
    ERR_RE = re.compile('\?\t([XPCSK])(?:,\t([a-zA-Z0-9]+))?')
    ERR_DICT = {
        'X' : 'Incorrect command: %s',
        'P' : 'Incorrect parameter %s',
        'C' : 'Channel %s on device not available',
        'S' : 'No sensor on channel %s connected',
        'K' : 'No divider in the command available: %s' ,
    }
    ERR_RPV_DICT = {
        0  : 'Measurement value OK',
        1  : 'Measuring value < measuring range',
        2  : 'Measuring value > measuring range',
        3  : 'Measuring undershooting (Err Lo)',
        4  : 'Measuring overstepping (Err Hi)',
        5  : 'Sensor off (oFF)',
        6  : 'HV on (HU on)',
        7  : 'Sensor erro (Error S)',
        8  : 'BA error (Err bA)',
        9  : 'No Sensor (no Sen)',
        10 : 'No switch on or switch off point (notriG)',
        11 : 'Pressure value overstepping (Err P)',
        12 : 'Pirani error ATMION (Err Pi)',
        13 : 'Breakdown of operational voltage (Err 24)',
        14 : 'Filament defectively (FiLbr)'
    }


#------------------------------------------------------------------
# setup_standard_attributes
#------------------------------------------------------------------
    def _setup_standard_attributes(self):
        for attr in MVC3GaugeControllerClass.attr_list:
            method = 'is_' + attr + '_allowed'
            if not hasattr(MVC3GaugeController, method): #respect method if already defined
                setattr(MVC3GaugeController, method, MVC3GaugeController._standard_attr_state_machine)


#------------------------------------------------------------------
# standard_attribute_state_machine
#------------------------------------------------------------------
    def _standard_attr_state_machine(self, req_type = None):
        state = self.get_state()
        if state in [PyTango.DevState.ON, PyTango.DevState.ALARM, PyTango.DevState.OFF]:
            return True
        elif (req_type == PyTango.AttReqType.READ_REQ) and (state in [PyTango.DevState.UNKNOWN, PyTango.DevState.FAULT]):
            return True
        else:
            return False


#------------------------------------------------------------------
# serial communications functions. Serial and PySerial supported
#------------------------------------------------------------------

    def _init_comms(self):
        serial_class = self.serial.info().dev_class
        if not serial_class in self.KNOWN_SERIAL_CLASSES:
            msg = 'Unknown serial class %s. Valid ones are: %s' % (serial_class, self.KNOWN_SERIAL_CLASSES)
            self.error_stream(msg)
            PyTango.Except.throw_exception('Unknown serial class', msg, '%s::_init_comms()' % self.get_name())

        if serial_class == 'Serial':
            self.serial.command_inout('DevSerFlush',2)
            self._communicate_raw = self._communicate_raw_Serial
        elif serial_class == 'PySerial':
            if self.serial.State() != PyTango.DevState.ON:
                self.serial.command_inout('Open')
            self.serial.command_inout('FlushInput')
            self.serial.command_inout('FlushOutput')
            self._communicate_raw = self._communicate_raw_PySerial
        else: #should be impossible, but you never know
            msg = 'Unsupported serial device class: %s. Valid ones are: %s' % (serial_class, str(self.KNOWN_SERIAL_CLASSES))
            self.error_stream(msg)
            PyTango.Except.throw_exception('Communication init error', msg, '%s::_init_comms()' % self.get_name())


    def _communicate_raw_Serial(self, cmd, output_expected=False, strip_string=True):
        self.lock.acquire()
        try:
            self.debug_stream('In %s::_communicate_raw_Serial() command input: %s' % (self.get_name(), cmd) )
            
            # Force the buffer cleanup
            if output_expected: self.serial.command_inout('DevSerReadRaw')
            self.serial.command_inout('DevSerFlush', 2)
            
            #self.serial.command_inout('DevSerWriteString', cmd)
            self.serial.command_inout('DevSerWriteChar', map(ord,cmd)) #Modified to avoid string array memleaks in Tango8
            #@todo: Serial C++ device should allow to write its own newline character to avoid this hardcode
            self.serial.command_inout('DevSerWriteChar', bytearray('\r'))
            if output_expected:
                read_ = self.serial.command_inout('DevSerReadLine')
                if strip_string:
                    output = read_.strip()
                else:
                    output = filter(lambda x: x not in ('\r','\n'), read_) #preserve all characters except \n and \r
                self.debug_stream('In %s::_communicate_raw_Serial(). Command output: %r Return value: %r' % (self.get_name(), read_, output))
                return output
        except Exception, e:
            self.error_stream('In %s::_communicate_raw_Serial() unexpected exception: %s' % (self.get_name(), str(e)) )
            raise
        finally:
            self.lock.release()


    def _communicate_raw_PySerial(self, cmd, output_expected=False, strip_string=True):
        self.lock.acquire()
        try:
            self.debug_stream('In %s::_communicate_raw_PySerial() command input: %s' % (self.get_name(), cmd) )
            self.serial.command_inout('Write',bytearray(cmd))
            if output_expected:
                read_ = self.serial.command_inout('ReadLine')
                if read_ .size == 0:
                    msg = 'Got empty return value from PySerial device. This is probably a communication error. Please check'
                    self.error_stream(msg)
                    PyTango.Except.throw_exception('Communication error', msg, '%s::_communicate_raw_PySerial()' % self.get_name())
                read_ = read_.tostring()
                if strip_string:
                    output = read_.strip()
                else:
                    output = filter(lambda x: x not in ('\r','\n'), read_) #preserve all characters except \n and \r
                self.debug_stream('In %s::_communicate_raw_PySerial(). Command output: %r Return value: %r' % (self.get_name(), read_, output))
                return output
        except Exception, e:
            self.error_stream('In %s::_communicate_raw_PySerial() unexpected exception: %s' % (self.get_name(), str(e)))
            raise
        finally:
            self.lock.release()


#------------------------------------------------------------------
#   Set state
#------------------------------------------------------------------
    def _set_state(self, new_state, new_status=None):
        #preserve FAULT state if set
        if self.get_state() == PyTango.DevState.FAULT:
            return
        self.set_state(new_state)
        if new_status != None:
            self.set_status(new_status)


#------------------------------------------------------------------
# Device constructor
#------------------------------------------------------------------
    def __init__(self,cl, name):
        PyTango.Device_4Impl.__init__(self,cl,name)
        self._setup_standard_attributes()
        MVC3GaugeController.init_device(self)


#------------------------------------------------------------------
#    Device destructor
#------------------------------------------------------------------
    def delete_device(self):
        self.info_stream('[Device delete_device method] for device %s' % self.get_name())
        try:
            #let's unlock front panel
            if self.auto_lock:
                ok = self._communicate_raw(self.CMD_LOCK % int(False), output_expected=True)
                if ok != 'OK':
                    self.error_stream('Error while trying to unlock front panel')
        except Exception, e:
            msg = 'Error while trying to %s::delete_device(): %s' % (self.get_name(), str(e))
            self.error_stream(msg)


#------------------------------------------------------------------
# Device initialization
#------------------------------------------------------------------
    def init_device(self):
        self.info_stream('In %s::init_device()' % self.get_name())

        #get device properties
        self.get_device_properties(self.get_device_class())

        #lock for avoiding simultaneous serial port access
        self.lock = threading.Lock()

        #try to initialize communications (set FAULT state and return if goes wrong)
        try:
            self.serial = PyTango.DeviceProxy(self.SerialDevice)
            self._init_comms()
        except Exception, e:
            msg = 'In %s::init_device() error while initializing communication: %s' % (self.get_name(), repr(e))
            self.error_stream(msg)
            self._set_state(PyTango.DevState.FAULT, msg)
            return

        #at least we have access to the serial, so try to go on
        try:
            #let's be optimistic
            self.set_state(PyTango.DevState.ON)
            self.set_status('Device is working correctly')

            #initialize channels keys
            self.UsedChannels = list(self.UsedChannels)
            if not self.UsedChannels in ([],['']):
                if (not (set(self.UsedChannels).issubset(set(self.CHANNELS)))):
                    PyTango.Except.throw_exception('Bad parameter', 'Invalid UsedChannels: %s' % self.UsedChannels, '%s::init_device()' % self.get_name())
                self.channels_keys = self.UsedChannels
            else:
                self.channels_keys = self.CHANNELS

            #let's try to avoid unnecessary accesses to the hardware
            if self.ReadValidityPeriod == []:
                ch_read_validity = 0
            else:
                ch_read_validity = self.ReadValidityPeriod

            #channels initialization
            self.channels = {}
            for channel in self.channels_keys:
                self.channels[channel] = Channel(self)
                self.channels[channel].number = int(channel)
                self.channels[channel].read_validity = ch_read_validity

            #if autolock is on then try to lock front panel
            if (self.AutoLockFrontPanel != []) and (self.AutoLockFrontPanel):
                self.auto_lock = True
                ok = self._communicate_raw(self.CMD_LOCK % int(True), output_expected=True)
                if ok != 'OK':
                    msg = 'Error while trying to lock front panel'
                    self._set_state(PyTango.DevState.FAULT, msg)
            else:
                self.auto_lock = False

        except Exception, e:
            msg = 'In %s::init_device() unexpected exception: %s' % (self.get_name(), repr(e))
            self.error_stream(msg)
            self._set_state(PyTango.DevState.FAULT, msg)
            raise


#------------------------------------------------------------------
#    Always executed hook method
#------------------------------------------------------------------
    def always_executed_hook(self):
        self.info_stream('In %s::always_excuted_hook()' % self.get_name())


#==================================================================
#
# MVC3GaugeController read/write attribute methods
#
#==================================================================
#------------------------------------------------------------------
# Read Attribute Hardware
#------------------------------------------------------------------
    def read_attr_hardware(self,data):
        self.info_stream('In %s::read_attr_hardware()' % self.get_name())


#------------------------------------------------------------------
# Read P1 attribute
#------------------------------------------------------------------
    def read_P1(self, attr):
        self.info_stream('In %s::read_P1()' % self.get_name())
        self.read_Channels(attr)


#------------------------------------------------------------------
# Read P2 attribute
#------------------------------------------------------------------
    def read_P2(self, attr):
        self.info_stream('In %s::read_P2()' % self.get_name())
        self.read_Channels(attr)


#------------------------------------------------------------------
# Read P3 attribute
#------------------------------------------------------------------
    def read_P3(self, attr):
        self.info_stream('In %s::read_P3()' % self.get_name())
        self.read_Channels(attr)

#------------------------------------------------------------------
#    Read Channels
#------------------------------------------------------------------
    def read_Channels(self, attr):
        self.info_stream('In %s::read_Channels()' % self.get_name())

        channel_name = attr.get_name()
        ch = channel_name[-1]

        if ch not in self.channels_keys:
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
            return

        channel = self.channels[ch]

        #read from hardware
        channel.read()

        #return the requested channel
        if math.isnan(channel.value):
            attr.set_quality(PyTango.AttrQuality.ATTR_INVALID)
        attr.set_value(channel.value)


#==================================================================
#
#    MVC3GaugeControllerClass class definition
#
#==================================================================
class MVC3GaugeControllerClass(PyTango.DeviceClass):

    # Class Properties
    class_property_list = {
    }


    # Device Properties
    device_property_list = {
        'SerialDevice':
            [PyTango.DevString,
            'The serial device to connect to the instrument. Configuration must be 19200 8N1 no software nor hardware flow control. '
            'Baudrate can be configured in the instrument if necessary (accepts 9600, 19200 and 38400) 0x0D (CR) charactermus must'
            'be used as newline. '
            'If using a C++ Serial device, \'newline\' device property must be set to 13. '
            'If using a PySerial device, then the \'Terminator\' attribute must be set to CR. '
            'There is an important difference between C++ and python serial devices. The former does have any function to write its own'
            'newline character when calling DevSerWriteString (so we have to do it) but the latter writes its own newline character when calling its Write command',
            [] ],
        'ReadValidityPeriod':
            [PyTango.DevDouble,
            'Time in seconds (may include decimals or be 0) while the last read values from the hardware are consider to be valid. '
            'This is done to try to minimize the accesses to the hardware. '
            'If not specified, 0 be used.',
            [] ],
        'AutoLockFrontPanel':
            [PyTango.DevBoolean,
            'Front panel lock at init. '
            'If not specified False is assumed.',
            [] ],
        'UsedChannels':
            [PyTango.DevVarStringArray,
            'The channels we really want to read and manage (ignore the others). Channels may be discontinued (i.e 1 and 3). '
            'If not specified, all channels will be used',
            [] ],
    }


    # Command definitions
    cmd_list = {
    }


    # Attribute definitions
    attr_list = {
        'P1':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'unit':"mbar",
                'format':"%5.2e",
                'Polling period':500,
            } ],
        'P2':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'unit':"mbar",
                'format':"%5.2e",
                'Polling period':500,
            } ],
        'P3':
            [[PyTango.DevDouble,
            PyTango.SCALAR,
            PyTango.READ],
            {
                'unit':"mbar",
                'format':"%5.2e",
                'Polling period':500,
            } ],
    }


#------------------------------------------------------------------
#    MVC3GaugeControllerClass Constructor
#------------------------------------------------------------------
    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type(name)
        print "In MVC3GaugeControllerClass  constructor"



#==================================================================
#
#    MVC3GaugeController class main method
#
#==================================================================
def main(*args):
    try:
        py = PyTango.Util(*args)
        py.add_TgClass(MVC3GaugeControllerClass,MVC3GaugeController,"MVC3GaugeController")

        U = PyTango.Util.instance()
        U.server_init()
        U.server_run()

    except PyTango.DevFailed,e:
        print "-------> Received a DevFailed exception:",e
    except Exception,e:
        print "-------> An unforeseen exception occurred....",e


#==================================================================
#
#    MVC3GaugeController class main invocation
#
#==================================================================
if __name__ == "__main__":
    main(sys.argv)
