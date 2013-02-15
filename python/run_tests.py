#!/usr/bin/python
#
# Copyright (c) 2012, Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of the University of California, Berkeley nor the names
#   of its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#
# Run test suite
#
# by Fernando L. Garcia Bermudez
#
# v.0.1
#
# Revisions:
#  Fernando L. Garcia Bermudez      2012-8-20    Initial release
#

import sys, traceback
import test_suite


#RADIO_DEV_NAME  = '/dev/tty.usbserial-*' or 'COMx'
#RADIO_DEV_NAME = 'COM1'
RADIO_DEV_NAME = 'COM9'
BS_BAUDRATE = 230400

DEST_ADDR = '\x21\x02'

if __name__ == '__main__':
    try:
        ts = test_suite.TestSuite(RADIO_DEV_NAME,            \
                                  baud_rate=BS_BAUDRATE, \
                                  dest_addr=DEST_ADDR  )

        #print('\nI: Testing radio communication:')
        #ts.test_radio()
        
        while True:
            raw_input("Press any key to continue...")
            
            # print('\nI: Testing radio communication:')
            # ts.test_radio()

            print('\nI: Testing motors:\n')
            ts.test_motorop()

            # print('\nI: Testing MPU6000:\n')
            # ts.test_mpu()
    
        #print('\nI: Testing motor channel 1')
        #ts.test_motor_basic()
        #ts.test_motor()
        #ts.test_sma()

        ts.__del__()
        sys.exit(0)
    
    ### Exception handling
    except SystemExit as e:
        print('\nI: SystemExit: ' + str(e))
        sys.exit(1)
    except KeyboardInterrupt:
        print('\nI: KeyboardInterrupt')
        ts.__del__()
        sys.exit(1)
    except Exception as e:
        print('\nE: Unexpected exception!\n' + str(e))
        traceback.print_exc()
        sys.exit(1)
