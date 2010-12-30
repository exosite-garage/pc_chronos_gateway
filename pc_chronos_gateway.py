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
  serialport = getconfiguration('Node_Connection',1)
  
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
  
  publish = PublishToExosite()
  publish.start()
  
  devices = getconfiguration('Devices',1)
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
        time.sleep(1)   #change this to report at a slower rate
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
def getconfiguration(section, printvalues):
#===============================================================================
  config = ConfigParser.RawConfigParser()
  config.read('options.cfg')
  config_list = {}
  if printvalues:
    print "======================"
    print "%s Settings:" % section
    print "======================"
  for option in config.options(section):
    config_list[option] = config.get(section, option)
    if printvalues: print "%s: %s" % (option,config_list[option])
  if printvalues:
    print "======================"
    print "\n"
  return config_list

#===============================================================================
def getsubscribers(list_cik, printvalues):
#===============================================================================
  datasource_info = []
  connection = getconfiguration ('Exosite_Connection',0)
  exompp = Exompp(connection)
  ## try to connect to Exosite
  if -1 == exompp.connect():
    print "Could not connect to Exosite - check your server settings"
    return -1
  if printvalues:
    print "======================"
    print "Subscribers in CIK list %s:" % list_cik
    print "======================"
  subscribers = -1
  while -1 == subscribers:
    subscribers = exompp.listdatasources(list_cik)
    if -1 == subscribers: 
      time.sleep(10)
      print "Couldn\'t list data sources (%s), retrying connection." % list_cik
      exompp.connect()
  #loop through subscriber information
  for k, v in subscribers.iteritems():
    #information for each subscriber is stored in json format
    parameters = json.loads(exompp.dsread(k,1))
    if -1 != parameters:
      #first load our subscriber device meta values
      cik = parameters['cik']
      location = parameters['location']
      #second, loop through our json and get the list of the data sources
      for pK, pV in parameters.iteritems():
        if 'datasource' == pK[:len('datasource')]:
          datasource_info.append({'name':k,'cik':cik,'type':pV['type'],'datasource':pV['name'],'units':pV['units'],'prefs':pV['prefs']})
          if printvalues: 
            print datasource_info[len(datasource_info) - 1]
  if printvalues:
    print "======================"
    print "\n"
  return datasource_info
  
#===============================================================================
class PublishToExosite ( threading.Thread ):
#===============================================================================
#-------------------------------------------------------------------------------
  def __init__ ( self ):
    threading.Thread.__init__ ( self )
    self.ringHead = 0
    self.ringTail = 0
    self.ringItems = 0
    self.ringSize = 256
    self.ring = {}
    connection = getconfiguration ('Exosite_Connection',1)
    self.exompp = Exompp(connection)
    ## try to connect to Exosite
    if -1 == self.exompp.connect():
      print "Could not connect to Exosite - check your server settings"
      return -1
    self.datasources = {}

#-------------------------------------------------------------------------------
  def addData ( self, device_cik, resource, value ):
    if self.ringItems + 1 > self.ringSize:
      #we just overran our buffer.  means we are getting data faster than we
      #can feed it to Exosite.  
      #wipe the buffer and just start doing the best we can with new data
      print "Warning - rx'ing data faster than publishing, wiping buffers."
      self.ringItems = 0
      self.ringHead = 0
      self.ringTail = 0
    # write values into buffer
    self.ring[self.ringHead] = {'device_cik':device_cik,'res_name':resource,'res_value':value}
    self.ringItems += 1
    if self.ringHead + 1 > self.ringSize:
      #wrap head
      self.ringHead = 0
    else: self.ringHead += 1

#-------------------------------------------------------------------------------
  def run ( self ):
    global kill_threads
    lastcik = ''
    lastringsize = 0
    message_output = True
    print "Monitoring Nodes, Publishing Data..."
    while False == kill_threads:
      if self.ringItems > 0:
        device_cik = self.ring[self.ringTail]['device_cik']
        try:
          self.datasources[device_cik]
        except:
          self.datasources[device_cik] = self.exompp.listdatasources(device_cik)
        res_name = self.ring[self.ringTail]['res_name']
        res_value = self.ring[self.ringTail]['res_value']
        if self.ringTail + 1 > self.ringSize:
          #wrap tail
          self.ringTail = 0
        else: self.ringTail += 1
        # note interlocked vulnerability here - TODO figure out python
        # interlocked calls/mutexes
        if self.ringItems > 0: #just in case we overflowed in the meantime
          self.ringItems -= 1
        #set up device channel
        if lastcik != device_cik: 
          lastcik = device_cik
          try:
            if -1 == self.exompp.setcik(device_cik):
              message_output = True
              print "Failed to set CIK %s" % device_cik
              lastcik = ''
              continue
          except:
            print "exception raised, continuing"
            message_output = True
            print "Failed to set CIK %s" % device_cik
            lastcik = ''
            continue
        #find resource number from resource name
        try:
          res_number = self.datasources[device_cik][res_name]
        except:
          message_output = True
          print "No datasource named %s, creating..." % res_name
          res_number = 0
          try: 
            for a,b in self.datasources[device_cik].iteritems():
              if int(b) > int(res_number): res_number = int(b)
            res_number += 1
          except:
            res_number = 1
          if -1 == self.exompp.createdatasource(res_name,res_number):
            print "Data source problem (%s) - check limits, check name & resource # pairing" % res_name
            continue
          else:
            self.datasources[device_cik] = self.exompp.listdatasources(device_cik)
        #write nfo. if write fails, try re-sending cik next go-around
        try:
          if -1 == self.exompp.write(res_number, res_value):
            message_output = True
            lastcik=''
            continue
        except:
          print "exception raised, continuing"
          message_output = True
          lastcik=''
          continue
        else: 
          #all of this is just to create a buffer size indicator on stdout
          if True == message_output:
            print "======================"
            sys.stdout.write('BUFFER: ')
            message_output = False
          currentringitems = self.ringItems
          if lastringsize != currentringitems:
            bufstring = ''
            rewstring = ''
            if lastringsize < currentringitems:
              sizediff = currentringitems - lastringsize
              while sizediff > 0:
                bufstring += '*'
                sizediff -= 1
              sys.stdout.write(bufstring)
            else:
              sizediff = lastringsize - currentringitems
              while sizediff > 0:
                bufstring += ' '
                rewstring += '\x08'
                sizediff -= 1
              sys.stdout.write(rewstring)
              sys.stdout.write(bufstring)
              sys.stdout.write(rewstring)
            lastringsize = currentringitems
          sys.stdout.flush()
      else: # if self.ringItems > 0
        time.sleep(1) # go to sleep for a second before looking again

#===============================================================================
class Exompp():
#===============================================================================
#-------------------------------------------------------------------------------
  def __init__(self, connection):
    self.connection = connection
    self.duplicate = False
    self.dsname = ''
    self.dsresource = ''

#-------------------------------------------------------------------------------
  def connect (self):
    retry = 1
    while 0 != retry:
      if retry > 1:
        time.sleep(10)
      try:
        jid = xmpp.protocol.JID(self.connection['user_id'])
      except:
        print "Unable to establish XMPP connection"
        retry += 1
        continue
      cl = xmpp.Client(jid.getDomain(), debug=0)
      self.messenger = Messenger(cl)
      try:
        con = cl.connect()
      except:
        print "Connection request was not reciprocated."
        retry += 1
        continue
      auth = 0
      try:
        auth = cl.auth(jid.getNode(), self.connection['password'])
      except:
        print "Authentication failed"
        retry += 1
        continue
      if not auth:
        print "Authentication failed"
        retry += 1
        continue
      cl.RegisterHandler('message', self.messenger.message_handler)
      #check for API version compatibility
      msg = xmpp.protocol.Message(to=self.connection['exosite_bot'],
                                  body='commanderid',
                                  typ='chat')
      self.messenger.send(msg, self.stdcallback, 'XMPP Commander 0.1')
      if self.messenger.wait() == -1: 
        print "Connection error or timed out. connect()"
        retry += 1
        continue
      else:
        if retry > 1: print "Connection re-established"
        retry = 0

#-------------------------------------------------------------------------------
  def setcik (self, cikvalue):
    retval = self.rawwrite('setcik %s\n' % cikvalue, 'ok')
    if -1 == retval: 
      print "Error in setcik (%s)" % cikvalue
    return retval

#-------------------------------------------------------------------------------
  def createdatasource (self, name, resource):
    self.dsname = name  
    self.dsresource = resource
    msg = xmpp.protocol.Message(to=self.connection['exosite_bot'],
                                body='dslist full',
                                typ='chat')
    self.messenger.send(msg, self.checkdsexistscallback)
    if self.messenger.wait() == -1:
      print "Connection error or timed out. createdatasource dslist(%s,%s)" % (name,resource)
      self.connect()
      return -1
    
    if self.duplicate:
      print "Data source %s is already setup, continuing..." % name
      self.duplicate = False
    else:
      msg = xmpp.protocol.Message(to=self.connection['exosite_bot'],
                                  body='dscreate %s %s na 0' % (name, resource),
                                  typ='chat')

      self.messenger.send(msg, self.cdscallback)
      if self.messenger.wait() == -1:
        print "Connection error or timed out. createdatasource dscreate(%s,%s)" % (name,resource)
        self.connect()
        return -1

#-------------------------------------------------------------------------------
  def rawwrite (self, messagebody, expected=''):
    msg = xmpp.protocol.Message(to=self.connection['exosite_bot'],
                                body=messagebody,
                                typ='chat')
    self.messenger.send(msg, self.stdcallback, expected)
    if self.messenger.wait() == -1:
      print "Connection error or timed out. rawwrite(%s)" % messagebody.strip()
      self.connect()
      return -1

#-------------------------------------------------------------------------------
  def write (self, resource, data):
    msg = xmpp.protocol.Message(to=self.connection['exosite_bot'],
                                body='write %s %s' % (resource, data),
                                typ='chat')
    self.messenger.send(msg, self.stdcallback, 'ok')
    if self.messenger.wait() == -1:
      print "Connection error or timed out.  write(%s)" % resource
      self.connect()
      return -1

#-------------------------------------------------------------------------------
  def dsread (self, ds_name, points):
    if 1 != points:
      print "Warning, dsread(): currently only supports reading one point."
      points = 1
    self.readmessage = {}
    msg = xmpp.protocol.Message(to=self.connection['exosite_bot'],
                                body='dsread %s %s\n' % (ds_name, points),
                                typ='chat')
    self.messenger.send(msg, self.readcallback)
    if self.messenger.wait() == -1:
      print "Connection error or timed out.  dsread(%s)" % ds_name
      self.connect()
      return -1
    return self.readmessage

#-------------------------------------------------------------------------------
  def read (self, resource):
    self.readmessage = {}
    msg = xmpp.protocol.Message(to=self.connection['exosite_bot'],
                                body='read %s\n' % resource,
                                typ='chat')
    self.messenger.send(msg, self.readcallback)
    if self.messenger.wait() == -1:
      print "Connection error or timed out.  read(%s)" % resource
      self.connect()
      return -1
    return self.readmessage

#-------------------------------------------------------------------------------
  def listdatasources (self, cik):
    self.dslist = {}
    self.setcik(cik)
    msg = xmpp.protocol.Message(to=self.connection['exosite_bot'],
                                body='dslist full',
                                typ='chat')
    self.messenger.send(msg, self.dslistcallback)
    if self.messenger.wait() == -1:
      print "Connection error or timed out. listdatasources(%s)" % cik
      self.connect()
      return -1
    return self.dslist

#-------------------------------------------------------------------------------
  def stdcallback (self, response, expected):
    if expected != '':
      if response != expected: return -1

#-------------------------------------------------------------------------------
  def cdscallback (self, response, expected):
    if response.find("error") != -1:
      print "CreateDataSource Error: response: %s" % response
      return -1

#-------------------------------------------------------------------------------
  def checkdsexistscallback (self, response, expected):
    start = response.find(self.dsname)
    if start != -1:
      end = response.find(',',start)
      #if the found name was just a subset of another name, it isn't duplicate
      if end - start > len(self.dsname): return 
      self.duplicate = True
      start = response.find(',',start) + 1
      end = response.find(',',start)
      if self.dsresource != response[start:end]:
        print "Error: Duplicate resource name, but resource # does not match."
        return -1

#-------------------------------------------------------------------------------
  def dslistcallback (self, response, expected):
    #possible that the device doesn't have any datasources setup yet
    if -1 == response.find('error'): 
      start = 0
      end = 1
      while -1 != start:
        end = response.find(',',start)
        dsname = response[start:end]
        start = end + 1
        end = response.find(',',start)
        self.dslist[dsname] = response[start:end]
        start = response.find('\x0A',end)
        if -1 != start: start += 1

#-------------------------------------------------------------------------------
  def readcallback (self, response, expected):
    if -1 == response.find('error'):
      start = 0
      #strip double quotes - CSV formatting inserts a second quote
      #to escape any quotes in the entry
      response = response.replace('\"\"','\"')
      length = len(response)
      start = response.find(',',start) + 1
      if '\"' == response[start]: start += 1
      end = length
      if '\"' == response[end - 1]: end -= 1
      self.readmessage = response[start:end]
    else:
      print "Error in read response"
      return -1
  

#===============================================================================
class Messenger(object):
#===============================================================================
#-------------------------------------------------------------------------------
  def __init__(self, client):
    self.wait_for_response = False
    self.callback = None
    self.client = client
    self.start = 0


#-------------------------------------------------------------------------------
  def wait(self):
    self.start = time.clock()
    while self.wait_for_response:
      if time.clock() - self.start > 10:
        self.wait_for_response = False 
        return -1
      if not self.client.Process(1):
        print 'Disconnected'
        return -1

#-------------------------------------------------------------------------------
  def message_handler(self, con, event):
    response = event.getBody()
    if self.callback:
      if -1 == self.callback(response,self.callbackexpected):
        print "WARNING: XMPP response: %s" % response
        self.start = time.clock() - 11
      else:
        self.wait_for_response = False
    else:
      if response.find("ok") == -1:
        print "ERROR: XMPP response: %s" % response
        self.start = time.clock() - 11
      else:
        self.wait_for_response = False

#-------------------------------------------------------------------------------
  def send(self, message, callback=None, callbackexpected=''):
    self.wait_for_response = True
    self.callback = callback
    self.callbackexpected = callbackexpected
    self.client.send(message)

#===============================================================================        
if __name__ == '__main__':
  sys.exit(main())


