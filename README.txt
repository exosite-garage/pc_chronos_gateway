========================================
About
========================================
pc_chronos_gateway.py is a python script that collects data from a Texas 
Instruments Chronos watch via a CC1111 RF Access Point attached to a USB port, 
and then publishes the data to Exosite via XMPP

License is BSD, Copyright 2010, Exosite LLC

Built/tested with Python 2.6.5

========================================
Quick Start
========================================
****************************************
1) install python
****************************************
http://www.python.org/download/
http://www.python.org/download/releases/2.6.5/
http://www.python.org/ftp/python/2.6.5/python-2.6.5.msi

****************************************
2) install xmpppy
****************************************
http://xmpppy.sourceforge.net/
http://sourceforge.net/projects/xmpppy/
http://sourceforge.net/projects/xmpppy/files/xmpppy/0.4.0/xmpppy-0.4.0.win32.exe/download

If running Debian Linux (or Ubuntu), you can > apt-get install python-xmpp

****************************************
3) install dnspython
****************************************
http://www.dnspython.org/

If running Debian Linux (or Ubuntu), you can > apt-get install python-dns
NOTE: this step is not always necessary, depends on xmpp server you use

****************************************
4) install serial
****************************************
http://pyserial.sourceforge.net/
http://pypi.python.org/pypi/pyserial

If running Debian Linux (or Ubuntu), you can > apt-get install python-serial

****************************************
5) install exompp
****************************************
https://github.com/exosite-labs/pyexompp

This is a library that supports an XMPP chat interface to Exosite's Platform.

****************************************
6) configure it
****************************************
Open the file "options.cfg"
--) update the serial port # in options.cfg to point to the port you are
	using (use device manager in windows or a terminal program to list ports)
--) update the default credentials in options.cfg to use your xmpp login
--) add device name and CIK pairing to options.cfg (get CIK from Exosite
	Portals device page -> add new device, device_name should match 
	whatever is sent by the device hooked to the serial port)

****************************************
7) test it out
****************************************
get python script "pc_chronos_gateway.py"
--) attach a CC1111 RF AP device to a/the serial port
--) run the script (> python pc_chronos_gateway.py)
--) verify the app connects to both the device and Exosite (no errors 
    should be generated)
--) log into portals.exosite.com and verify the data source is created and 
	that data was generated

****************************************
8) tweak it
****************************************
--) play around, use it, extend it!
