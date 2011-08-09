#==============================================================================
# pc_chronos_gateway.py
# Python script that talks to a CC1111 RF Access Point with stock firmware
# from a Texas Instruments ez430 Chronos Development Kit, and gathers wireless
# data sent by a Chronos watch and forwards it onto Exosite's One Platform
# so the data can be viewed remotely.
#==============================================================================
## Tested with python 2.6.5
##
## Copyright (c) 2010, Exosite LLC
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without 
## modification, are permitted provided that the following conditions are met:
##
##    * Redistributions of source code must retain the above copyright notice,
##      this list of conditions and the following disclaimer.
##    * Redistributions in binary form must reproduce the above copyright 
##      notice, this list of conditions and the following disclaimer in the
##      documentation and/or other materials provided with the distribution.
##    * Neither the name of Exosite LLC nor the names of its contributors may
##      be used to endorse or promote products derived from this software 
##      without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.

import sys
import time
import xmpp
import ConfigParser
import serial
import threading
import os

from exompp.xmppchat import *

  # Following commands were gleaned from:
  # http://e2e.ti.com/support/microcontrollers/msp43016-bit_ultra-low_power_mcus/f/166/t/32714.aspx
  # https://github.com/wolfmankurd/eZ430-tools/blob/master/eZ430.py
  # http://processors.wiki.ti.com/index.php/EZ430-Chronos#eZ430_python_tools
CMD_RDACC = '\xFF\x08\x07\x00\x00\x00\x00' #Response: ff 06 07 tt xx yy zz (tt = ff: no data, 01: valid acc data, xx, yy, zz : acc data)
CMD_START = '\xFF\x07\x03' 
CMD_START_RESPONSE = '\xFF\x06\x03' 
CMD_STOP = '\xFF\x09\x03'
CMD_PING = '\xFF\x20\x07\x00\x00\x00\x00' #Response: ff 06 07 xx xx xx xx (4 byte watch address)
    
HEADER = '12345678\x0D\x0A'
FOOTER = '87654321\x0D\x0A'
kill_threads = False

#===============================================================================
def main():
#===============================================================================
  global kill_threads
  ## read node connection info from config file
  cfg_filepath = os.path.join(os.getcwd(),'options.cfg')
  serialport = getconfiguration(cfg_filepath,'Node_Connection',1)
  
  ##setup node communications
  try:
    node = GatewayNodeIO(serialport,CMD_START,CMD_START_RESPONSE)
  except:
    print "Problem with connecting to node. Check config file port_name"
    try:
      node.closeNode()
    except:
      pass
    return -1
  
  publish = PublishToExosite(cfg_filepath)
  publish.start()
  
  devices = getconfiguration(cfg_filepath,'Devices',1)
  # kind of a hack - just getting the last device listed in options.cfg
  for k, v in devices.iteritems():
    cik = v
  
  while False == kill_threads:
    node.writePort(CMD_RDACC)
    rxchar = node.readPort(7)
    if '' != rxchar:
      if '\x01' == rxchar[3]:
        #print "x - %s" % ord(rxchar[4])
        #print "y - %s" % ord(rxchar[5])
        #print "z - %s" % ord(rxchar[6])
        publish.addData(cik, "x_axis", ord(rxchar[4]))
        publish.addData(cik, "y_axis", ord(rxchar[5]))
        publish.addData(cik, "z_axis", ord(rxchar[6]))
        time.sleep(5)   #change this to report at a slower rate
    else: 
      print "No response to read accelerometer command!"
  
  node.closeNode()
  
  print "\n"
  print "Exiting Program"
  return

#===============================================================================
class GatewayNodeIO():
#===============================================================================
#-------------------------------------------------------------------------------
  def __init__(self, portsettings, passphrase, phraseresponse):
    self.passphrase = passphrase
    self.phraseresponse = phraseresponse
    self.portname = portsettings['port_name']
    self.portbaud = int(portsettings['baud_rate'])
    try:  
      s = serial.Serial(port=self.portname)
      s.close()   # explicit close
      if -1 == self.openNode():
        print "Problem connecting to valid Node - check serial port connection."
        self.closeNode()
        return -1  
    except serial.SerialException:
      print "\r\nDefault Port, %d, not available." % self.portname
      print "Available ports include:"
      #scan for available ports.
      for i in range(256):
        try:
          s = serial.Serial(i)
          print i, "(%s)" % s.portstr
          s.close()   # explicit close
        except serial.SerialException:
          pass
      return -1

#-------------------------------------------------------------------------------
  def openNode(self):
    self.s = serial.Serial(port=self.portname)
    self.s.baudrate = self.portbaud
    self.s.bytesize = 8
    self.s.parity = 'N'
    self.s.stopbits = 1
    self.s.timeout = 1 #return after 1 second if all bytes aren't read
    self.s.xonxoff = False
    self.s.rtscts = False
    self.s.writeTimeout = None
    self.s.dsrdtr = False
    self.s.interCharTimeout = None   
    #if we are talking to a valid node, it will recognize the passphrase
    # and will send back the phraseresponse.
    if '' != self.passphrase:
      self.writePort(self.passphrase)
      response = self.readPort(len(self.phraseresponse))
      if response != self.phraseresponse: 
        print "Non valid serial response received: %s" % repr(response)
        return -1

#-------------------------------------------------------------------------------
  def closeNode(self):
    self.s.close()

#-------------------------------------------------------------------------------
  def writePort(self,data):
    self.s.write(data)

#-------------------------------------------------------------------------------
  def readPort(self,size):
    global kill_threads
    try:
      ret_val = self.s.read(size)
    except (KeyboardInterrupt, SystemExit):
      kill_threads = True
      return -1  
    return ret_val

#-------------------------------------------------------------------------------
  def findHeader(self):
    global kill_threads
    headercount = 0
    #just spin forever waiting for a header
    while headercount < len(HEADER):
      if True == kill_threads: break
      if (self.readPort(1) != HEADER[headercount]):
        headercount = 0
      else:
        headercount += 1

#-------------------------------------------------------------------------------
  def findFooter(self):
    footercount = 0
    while footercount < len(FOOTER):
      if (self.readPort(1) != FOOTER[footercount]):
        footercount = 0
        return False
      else:
        footercount += 1
    return True

#-------------------------------------------------------------------------------
  def readLine(self):
    rxchar = ''
    linestr = ''
    while rxchar != '\x0A':
      rxchar = self.readPort(1) 
      if rxchar == '': #if timeout
        return '',False
      linestr = '%s%s' % (linestr,rxchar)
    return linestr,True

#===============================================================================        
if __name__ == '__main__':
  sys.exit(main())


