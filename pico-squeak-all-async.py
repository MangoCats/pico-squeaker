# pico-squeaker
# presents a webpage interface which controls two PWM outputs and an amplifier enable signal
# v1.0 deployed 2022-8-7
# adding an "easy" interface page in addition to the advanced controls
# disabling signal strength parser: seems problematic
# v1.1 deployed 2022-8-11
# adding battery voltage monitor (input to ADC1 midpoint of a 67.5k-150k divider network)
# adding input of the charging indicator from the bq24074
# v1.2 deployed 2022-8-20
# re-architecht web interface to async implementation
# add thread safety with semaphore locks on thread shared variables
# add NTP query to set internal RTC
# add IFTTT POST actions, manually triggered for now
# v1.2 deployed 2022-8-20
# capture start time for display on advanced interface
# add watchdog timer
# v1.3 deployed 2022-9-11
# remove watchdog timer, it's not helping and may be making debug/devel more miserable
# remove potential infinite loop
# add rest period after serving a webpage
# drain writer before closing IFTTT writer
# v1.4 deployed 2022-9-13
# dropping threads to put everything in the async event loop
#
# imports used everywhere
import gc
import uasyncio as asyncio
from machine import Pin, ADC, PWM
import network
import socket
import struct
import time

ssid = 'ImNot'
password = 'telling'
wlan = network.WLAN(network.STA_IF)
myIp = 'tempInit'
iftttSecret = 'secret'

# https://ifttt.com/maker_webhooks
ifttt = "\r\nPOST /trigger/{}/with/key/{} HTTP/1.1\r\nHost: maker.ifttt.com\r\nConnection: close\r\n\r\n"

easyHtml = """<!DOCTYPE html>
<html>
  <head><title>Squeaker {}</title></head>
  <body><center><font size="+6">
    <table width="95%%" style="text-align:center"><tr>
      <td></td>
      <td><h2><a href="/advanced">Squeaker</a> {}</h2></td>
      <td></td>
    </tr><tr>
      <td>{}F<br/>{}V</td>
      <td>{}<br/><a href="/vary">Vary</a><br/>{}</td>
      <td bgcolor="{}">{}s<br/>{}</td>
    </tr><tr>
      <td><br/><a href="/sound/01">1 sec</a></td>
      <td><br/><a href="/sound/05">5 secs</a></td>
      <td><br/><a href="/sound/30">30 secs</a></td>
    </tr></table>
    <br/>
    </font>
    <hr/>
    </center>
  </body>
</html>
"""

html = """<!DOCTYPE html>
<html>
  <head><title>Squeaker {}</title></head>
  <body><center><font size="+3">
    <table width="95%%" style="text-align:center"><tr>
      <td></td>
      <td><h2><a href="/">Squeaker</a> {}</h2></td>
      <td></td>
    </tr><tr>
      <td><a href="/light/on">LED ON</a><br/>
          <a href="/light/off">LED OFF</a><br/>
          <a href="/settime">Set Time</a><br/>
          <a href="/verbose">Verbose</a><br/>
          <a href="/quiet">Quiet</a></td>
      <td bgcolor="{}">{}<br/>{}F {} sec<br/>{}V {}<br/>
                   {} {}dBm<font size="+2"><br/>Started {}<br/></font>{}</td>
      <td><a href="/full">Full</a><br/><br/>
          <a href="/thirsty">Thirsty</a><br/><br/>
          <a href="/vary">Vary</a></td>
    </tr><tr>
      <td><br/><a href="/sound/01">1 sec</a></td>
      <td><br/><a href="/sound/05">5 secs</a></td>
      <td><br/><a href="/sound/30">30 secs</a></td>
    </tr><tr>
      <td>{}<br/><a href="/freq1/05000">5kHz</a></td>
      <td><br/><a href="/freq1/10000">10kHz</a></td>
      <td><br/><a href="/freq1/20000">20kHz</a></td>
    </tr><tr>
      <td>{}<br/><a href="/freq2/04500">4.5kHz</a></td>
      <td><br/><a href="/freq2/09900">9.9kHz</a></td>
      <td><br/><a href="/freq2/19000">19kHz</a></td>
    </tr><tr>
      <td>{}<br/><a href="/fRng1/0.000">Constant</a></td>
      <td><br/><a href="/fRng1/0.100">+/-0.1x</a></td>
      <td><br/><a href="/fRng1/0.400">+/-0.4x</a></td>
    </tr><tr>
      <td>{}<br/><a href="/fRng2/0.000">Constant</a></td>
      <td><br/><a href="/fRng2/0.150">+/-0.15x</a></td>
      <td><br/><a href="/fRng2/0.350">+/-0.35x</a></td>
    </tr><tr>
      <td>{}<br/><a href="/oscP1/00200">200ms</a></td>
      <td><br/><a href="/oscP1/01000">1000ms</a></td>
      <td><br/><a href="/oscP1/05000">5000ms</a></td>
    </tr><tr>
      <td>{}<br/><a href="/oscP2/00250">250ms</a></td>
      <td><br/><a href="/oscP2/01000">1000ms</a></td>
      <td><br/><a href="/oscP2/04900">4900ms</a></td>
    </tr><tr>
      <td>{}<br/><a href="/tSlic/0.020">20ms</a></td>
      <td><br/><a href="/tSlic/0.050">50ms</a></td>
      <td><br/><a href="/tSlic/0.250">250ms</a></td>
    </tr></table>
    <br/>
    </font>
    <hr/>
    <br/><br/><br/><br/><br/>
    <p><a href="/exit">Exit</a></p>
    </center>
  </body>
</html>
"""

sound0 = {
    "freq": 20000,
    "fRng": 0.194,
    "oscP": 1000
}

sound1 = {
    "freq": 18877,
    "fRng": 0.146,
    "oscP": 1100
}

sound2 = {
    "freq": 17818,
    "fRng": 0.0953,
    "oscP": 1320
}

sound3 = {
    "freq": 16818,
    "fRng": 0.0415,
    "oscP": 1716 
}

sound4 = {
    "freq": 17079,
    "fRng": 0.0562,
    "oscP": 2402
}

sound5 = {
    "freq": 18094,
    "fRng": 0.1091,
    "oscP": 3604
}

sound6 = {
    "freq": 19170,
    "fRng": 0.1591,
    "oscP": 5766
}

sound7 = {
    "freq": 20310,
    "fRng": 0.2063,
    "oscP": 9802
}

sound8 = {
    "freq": 20154,
    "fRng": 0.2002,
    "oscP": 875
}

sound9 = {
    "freq": 19023,
    "fRng": 0.1526,
    "oscP": 729
}

soundA = {
    "freq": 17956,
    "fRng": 0.1022,
    "oscP": 547
}

soundB = {
    "freq": 16948,
    "fRng": 0.0489,
    "oscP": 273
}

soundC = {
    "freq": 16120,
    "fRng": 0.0,
    "oscP": 1000
}

sounds = [sound0,sound1,sound2,sound3,sound4,sound6,sound6,sound7,
          sound8,sound9,soundA,soundB,soundC]

months = [ "Jan", "Feb", "Mar", "Apr", "May", "Jun",
           "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" ]

# GPIO assignments (GPIO #s, not pin #s)
GPIO_BATT_VOLTAGE=27
GPIO_PWM_A=22
GPIO_PWM_B=21
GPIO_CHARGING=20
GPIO_AMP_SHUTDOWN=19
GPIO_TEMP_SENSOR=4

# Select the Pico W onboard LED (which is apparently not PWM capable)
led = machine.Pin("LED", machine.Pin.OUT)

# Temperature sensor and Battery Voltage
sensor_temp = ADC(GPIO_TEMP_SENSOR)
batt_adc    = machine.ADC(GPIO_BATT_VOLTAGE)
conversion_factor = 3.3 / 65535.0

# Charging indicator from bq24074
chg_pin = machine.Pin(GPIO_CHARGING, machine.Pin.IN)

# NTP stuff Timezone set UTC -4
NTP_DELTA = 2208988800 + 3600*4
host = "us.pool.ntp.org" # "time.nist.gov" # "pool.ntp.org"
global startTime
startTime = "-"

global VERBOSE
VERBOSE = False

def set_time():
    import socket
    
    global startTime
    global VERBOSE
    if VERBOSE:
      print( 'set_time()' )
      
    NTP_QUERY = bytearray(48)
    NTP_QUERY[0] = 0x1B # 0x23
    gotReply = False
    
    try:
      if VERBOSE:
        print('set_time() blocking query of', host )
      addr = socket.getaddrinfo(host, 123)[0][-1]
      s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      s.settimeout(1)
      res = s.sendto(NTP_QUERY, addr)
      msg = s.recv(48)
      gotReply = True
      if VERBOSE:
        print('set_time() blocking msg received:',msg)
           
    except Exception as e:
      print( "Failed blocking NTP query of {}: {}".format( host, e ) )
        
    finally:
      try:
        s.close()
        if VERBOSE:
          print( 'set_time() blocking connection closed' )
      except Exception as e:
        print( "socket close attempt failed: {}".format( e ) )
               
    if gotReply:
      try:
        val = struct.unpack("!I", msg[40:44])[0]
        if VERBOSE:
          print( 'set_time() unpacked val',val )
        tm = time.gmtime( val - NTP_DELTA )
        machine.RTC().datetime((tm[0], tm[1], tm[2], tm[6] + 1, tm[3], tm[4], tm[5], 0))
        if ( len(startTime) < 4 ):
          startTime = time_string()
        if VERBOSE:
          print( 'set_time() time set:', time_string() )
      except Exception as e:
        print( "Exception in NTP reply interpretation {}".format( e ) )
# endof def set_time():

def time_string():
    t = time.localtime()
    if ( t[1] < 1 ):
      t[1] = 1
    if ( t[1] > 12 ):
      t[1] = 12
    ts = "{}-{}-{} {}:{t4:02d}:{t5:02d}".format( t[0], months[t[1]-1], t[2],
                                                 t[3], t4=t[4], t5=t[5] )
    return ts

async def pico_thirsty():
    global VERBOSE
    if VERBOSE:
      print( 'pico_thirsty()' )
    try:
      reader, writer = await asyncio.open_connection("maker.ifttt.com", 80)
      if VERBOSE:
        print( 'pico_thirsty() non blocking connection established' )
      req = ifttt.format( "Pico_thirsty", iftttSecret )
      writer.write(req.encode("utf8"))
      msg = await reader.read(1024)
      if VERBOSE:
        print( 'pico_thirsty() read reply:', msg )

    except Exception as e:
      print( "Failed thirsty connection: {}".format( e ) )
    
    finally:
      try:
        await writer.drain()
        await writer.wait_closed()
        if VERBOSE:
          print( 'pico_thirsty() connection closed' )
      except Exception as e:
        print( "writer close attempt failed: {}".format( e ) )
# endof async def pico_thirsty():

async def pico_full():
    global VERBOSE
    if VERBOSE:
      print( 'pico_full()' )
    try:
      reader, writer = await asyncio.open_connection( "maker.ifttt.com", 80 )
      if VERBOSE:
        print( 'pico_full() non blocking connection established' )
      req = ifttt.format( "Pico_full", iftttSecret )
      writer.write(req.encode("utf8"))
      msg = await reader.read(1024)
      if VERBOSE:
        print( 'pico_full() read reply:', msg )

    except Exception as e:
      print( "Failed full connection: {}".format( e ) )
    
    finally:
      try:
        await writer.drain()
        await writer.wait_closed()
        if VERBOSE:
          print( 'pico_full() connection closed' )
      except Exception as e:
        print( "writer close attempt failed: {}".format( e ) )
# endof async def pico_full():

def webserver_done():
    global ws_launched
    ws_launched = False
    print( "webserver done" )
    
def launch_webserver():
    global ws_launched
    global serverOb
    if wlan.status() != 3:
      print( "Will not launch webserver in wlan.status:", wlan.status() )
    else:  
      serverOb = asyncio.start_server(serve_client, "0.0.0.0", 80)
      asyncio.create_task( serverOb )
      ws_launched = True
      print( "webserver launched" )
# endof def launch_webserver():

# Check first for an existing connection before
# going into the main connect_to_network process
async def connect_wifi():
    global wifi_connected
    
    if wlan.status() == 3:
      print('already connected to', str(wlan.config('ssid')) )
      get_our_ip()
      wifi_connected = True
        
    while wlan.status() != 3:
      await connect_to_network()

      if wlan.status() != 3:
        print("Status:", wlan.status(), " Resting.")
        await asyncio.sleep(5.0)
# endof async def connect_wifi():
    
def get_our_ip():
    global myIp
    
    status = wlan.ifconfig()
    print( 'ip = ' + status[0] )
    try: 
      myIp = status[0].split('.')[3]
    except:
      myIp = status[0]
#endof def get_our_ip():

            
async def connect_to_network():
    global wifi_connected
    
# LED on while attempting connection
    led.value(1)

    if wlan.status() == 3:
      print( "wifi already connected" )
    else:
      print( "connecting to", ssid )
      
      wlan.active(True)
#      wlan.config(pm = 0xa11140) # disable low power mode default=0xa11142 aggressive_pm=0xa11c82 performance_pm=0x111022
      wlan.connect( ssid, password )

# Wait for connect or fail
      max_wait = 20
      while max_wait > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
          break
        max_wait -= 1
        print('waiting for connection...',max_wait)
        await asyncio.sleep(1)

# Handle connection error
    if wlan.status() != 3:
      print('connection timed out to',ssid)
      print('status code',wlan.status())
    else:
      print('connected to',ssid)
      get_our_ip()
      wifi_connected = True
      launch_webserver()

# Connection attempt is over, light out.
    await asyncio.sleep(1)
    led.value(0)

# endof async def connect_to_network():

# my_rssi( apl ) where apl is the output of wlan.scan()
def my_rssi( apl ):
  maxrssi = -300
  if isinstance(apl, (list, tuple)):
    for ap in apl:
      if isinstance(ap, tuple):
        if len(ap) > 3:
          if ap[0].decode("utf-8") == ssid:
            rssi = ap[3]
            if isinstance(rssi, (int, float)):
              if rssi > maxrssi:
                maxrssi = rssi
            else:
              print( "rssi not a number?", rssi )
        else:
          print( "ap tuple len is", len(ap) )
      else:
        print( "ap is not a tuple?", ap )
  else:
    print( "apl is not a tuple or list?", apl )
  return maxrssi
#endof def my_rssi

async def serve_client( reader, writer ):
    global counter
    global shutdown
    global freq1
    global freq2
    global fRng1
    global fRng2
    global oscP1
    global oscP2
    global tSlic
    global easyPage
    global si1
    global si2
    global VERBOSE
    global gcCycle
    global myIp
    global startTime

    # print("Client connected",si1,si2)
    
    request_line = await reader.readline()
    # print("Request:", request_line)
# We are not interested in HTTP request headers, skip them
    header = b""
    ticksStart = time.ticks_ms();
    while ( header != b"\r\n" ) and ( time.ticks_diff(time.ticks_ms(), ticksStart) < 250 ):
      header = await reader.readline()
      await asyncio.sleep(0.0001)
      # print('Header:', str(header) )
      
    request = str(request_line[4:17])
    request = request[2:]  
    stateis = "Hello"        
    if 0 == request.find('/favicon.ico'):
      writer.write('HTTP/1.0 200 OK\r\nContent-type: image/x-icon\r\nContent-length: 0\r\n\r\n')
    else:
      print('Request:',request)
      if 0 == request.find('/ '):
        easyPage = True  
            
      elif 0 == request.find('/sound/'):
        counter = int(request[7:9])
        stateis = "Sound On %s" % counter
        # easyPage is unchanged
            
      elif 0 == request.find('/freq1/'):
        freq1 = int(request[7:12])
        stateis = "Freq1 %sHz" % freq1
        easyPage = False
          
      elif 0 == request.find('/freq2/'):
        freq2 = int(request[7:12])
        stateis = "Freq2 %sHz" % freq2
        easyPage = False
          
      elif 0 == request.find('/fRng1/'):
        fRng1 = float( request[7:12] )
        stateis = "fRng1 %sx" % fRng1
        easyPage = False
          
      elif 0 == request.find('/fRng2/'):
        fRng2 = float( request[7:12] )
        stateis = "fRng2 %sx" % fRng2
        easyPage = False

      elif 0 == request.find('/oscP1/'):
        oscP1 = int(request[7:12])
        stateis = "oscP1 %sms" % oscP1
        easyPage = False
          
      elif 0 == request.find('/oscP2/'):
        oscP2 = int(request[7:12])
        stateis = "oscP2 %sms" % oscP2
        easyPage = False
          
      elif 0 == request.find('/tSlic/'):
        tSlic = float( request[7:12] )
        stateis = "tSlic %sms" % (tSlic * 1000.0)
        easyPage = False
          
      elif 0 == request.find('/settime'):
        set_time()
        stateis = "Time Set"
        
      elif 0 == request.find('/verbose'):
        VERBOSE = True
        stateis = "Verbose"
        
      elif 0 == request.find('/quiet'):
        VERBOSE = False
        stateis = "Quiet"
        
      elif 0 == request.find('/vary'):
        si1 = si1 + 3
        si2 = si2 + 5
        while ( si1 >= len(sounds) ):
          si1 = si1 - len(sounds)
        while ( si2 >= len(sounds) ):
          si2 = si2 - len(sounds)            
        if si1 == si2:
          si2 = si2 + 1
        while ( si2 >= len(sounds) ):
          si2 = si2 - len(sounds)            
        stateis = "Varied {} {}".format( si1, si2 )
        freq1 = sounds[si1]["freq"]
        freq2 = sounds[si2]["freq"]
        fRng1 = sounds[si1]["fRng"]
        fRng2 = sounds[si2]["fRng"]
        oscP1 = sounds[si1]["oscP"]
        oscP2 = sounds[si2]["oscP"]
        
      elif 0 == request.find('/exit'):
        stateis = "Exiting."
        shutdown = True
        easyPage = True
            
      elif 0 == request.find('/light/on'):
        led.value(1)
        # led.duty_u16( 65535 )
        stateis = "LED is ON"
        easyPage = False
 
      elif 0 == request.find('/light/off'):
        led.value(0)
        # led.duty_u16( 0 )
        stateis = "LED is OFF"
        easyPage = False
          
      elif 0 == request.find('/thirsty'):
        stateis = "Pico Thirsty"
        await pico_thirsty()
        
      elif 0 == request.find('/full'):
        stateis = "Pico Full"
        await pico_full()
        
      elif 0 == request.find('/advanced'):
        stateis = "Advanced"
        easyPage = False
            
      else:  
        stateis = "Unknown cmd"
        # easyPage is unchanged

# take a temperature reading
      reading     = sensor_temp.read_u16() * conversion_factor 
      temperature = 27.0 - (reading - 0.706)/0.001721
      farenheit   = temperature * 1.8 + 32.0
          
# read the battery voltage          
      reading = batt_adc.read_u16() * 3.3 / 32767.0
      battV   = reading * (150.0+68.0) / 150.0

# Read strongest connection to ssid, but only on advanced page
      maxrssi = -200
      if easyPage == False:
        maxrssi = my_rssi( wlan.scan() )
            
# Read the charging pin from the bq24074        
      if chg_pin.value() == True:
        cmsg = "Not Charging"
      else:
        cmsg = "Charging"
        
# Get the sound timer remaining count, safely        
      cnt = counter
      if ( cnt > 0 ):
        bgcolor = "#CCCCCC"
      else:
        bgcolor = "#FFFFFF"

# Send the updated GUI to the browser
      writer.write('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
      if easyPage == True:
        writer.write( easyHtml.format( myIp, myIp, farenheit,
                                       battV,
                                       stateis, time_string(),
                                       bgcolor, cnt,
                                       cmsg ) )
        # led.value(0) # LED is always off when using the easy interface
      else:
        writer.write( html.format( myIp, myIp, bgcolor, stateis,
                                   farenheit, counter,
                                   battV, cmsg, ssid, maxrssi,
                                   startTime, time_string(),
                                   freq1, freq2,
                                   fRng1, fRng2,
                                   oscP1, oscP2,
                                   tSlic ) )
    
    await writer.drain()
    await writer.wait_closed()
    await asyncio.sleep(0.75)
    gcCycle = gcCycle - 15
#    print("Disconnected", request )       
# endof async def serve_interface():

# The Web Server Thread - turns GET requests into actions onboard
# also issues queries to an NTP server and POSTs to IFTTT
async def main():
    print("main() starting")
    global counter
    global shutdown
    global freq1
    global freq2
    global fRng1
    global fRng2
    global oscP1
    global oscP2
    global tSlic
    global wifi_connected
    global ws_launched
    global serverOb
    global VERBOSE
    global easyPage
    global startTime
    global myIp
    global si1
    global si2
    global gcCycle
    
    wifi_connected = False
    ws_launched = False
    easyPage = True
    startTime = "-"
    myIp = "."
    si1 = 1
    si2 = 2

    counter  = 0
    shutdown = False
    freq1 = 20000
    freq2 = 19000
    fRng1 = 0.3
    fRng2 = 0.1
    oscP1 = 225
    oscP2 = 3500
    tSlic = 0.04

# Set country code, opens legal WiFi channels
    from rp2 import country
    country('US')
            
# Launch the sound maker
    asyncio.create_task( sound_player() )

    gcCycle = 2
    await asyncio.sleep(0.1)
    print("main() loop starting")
    while shutdown == False:
    
      if wifi_connected == False:
        await connect_wifi()
      
      if ws_launched == False:
        if wlan.status() == 3:
          launch_webserver()
            
      await asyncio.sleep(1.0)
      
      # Here: implement automatic POST calls to IFTTT when needed (to charge batteries)
      
      gcCycle = gcCycle - 1
      if ( gcCycle <= 0 ):
        if VERBOSE:
          print( 'doing gc0 Status:',wlan.status(),'RSSI:',my_rssi( wlan.scan() ) )
        if ( len( startTime ) < 4 ):
          print( 'getting startTime' )
          set_time()
          print( 'started',time_string() )
        # https://docs.micropython.org/en/latest/reference/constrained.html#control-of-garbage-collection
        gc.collect()
        gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
        gcCycle = 60 # 1 minute between cycles
    # while shutdown == False:
    
    try:
      serverOb.close()
    finally:
      print("main() exiting")
      await asyncio.sleep(2.0) # let core1 exit first
# endof async def main():

# The sound player
# anytime counter is > 0 sound will play
async def sound_player():
    print("sound_player() starting")
    from machine import Pin, PWM
    from time import sleep
    import math
    
    global counter
    global shutdown
    global freq1
    global freq2
    global fRng1
    global fRng2
    global oscP1
    global oscP2
    global tSlic
    global VERBOSE
        
    counter  = 0
    shutdown = False
    freq1 = 20000
    freq2 = 19000
    fRng1 = 0.3
    fRng2 = 0.1
    oscP1 = 225
    oscP2 = 3500
    tSlic = 0.04

    pwmOff = True
    pwmA = PWM( Pin ( GPIO_PWM_A ) ) # GP22, pin 29
    pwmA.freq( freq1 ) 
    pwmA.duty_u16( 0 )
    pwmB = PWM( Pin ( GPIO_PWM_B ) ) # GP21, pin 27
    pwmB.freq( freq2 ) 
    pwmB.duty_u16( 0 )
    shdn = machine.Pin( GPIO_AMP_SHUTDOWN, machine.Pin.OUT ) # GP19, pin 25
    shdn.value(0)            # default to amplifier off

    await asyncio.sleep(1)

#    gcCycle1 = 75
    targ = math.pi * -30.0
    print("sound_player() loop starting")
    while shutdown == False:
        countRunning = ( counter > 0 )
        if countRunning:
          if pwmOff == True:
            pwmA.duty_u16( 32768 )   # duty 50% (65535/2)
            pwmB.duty_u16( 32768 )
            shdn.on()                # amplifier on
            if VERBOSE:
              print("Sound on")
            pwmOff = False
            
          if VERBOSE:
            print( counter )  
          counter -= 1
          oscC1 = 1000.0 / float(oscP1)  # convert the modulation periods (in ms) to coefficients
          oscC2 = 1000.0 / float(oscP2)
          fMod1 = float(freq1) * fRng1   # frequency modulation depths
          fMod2 = float(freq2) * fRng2
          tDone = targ + math.pi * 2.0
          slicI = math.pi * 2.0 * tSlic;
          fc1   = freq1
          fc2   = freq2
          tSlp  = tSlic
          
          while targ < tDone:            # work through the modulators for the next second
            pwmA.freq( fc1 + int( fMod1 * math.sin( targ * oscC1 ) ) )
            pwmB.freq( fc2 + int( fMod2 * math.sin( targ * oscC2 ) ) )
            targ += slicI;
            await asyncio.sleep( tSlp )
            
        else: # counter is not > 0
          if pwmOff == False:
            pwmA.duty_u16( 0 )     # stop oscillation
            pwmB.duty_u16( 0 )
            shdn.off()             # amplifier off
            if VERBOSE:
              print("Sound off")
            targ = math.pi * -30.0 # keep targ in a reasonable range
            pwmOff = True
            
          await asyncio.sleep( 0.2 )             # save a bit of power
                  
        #gcCycle1 = gcCycle1 - 1
        #if ( gcCycle1 <= 0 ):
        #  if VERBOSE:
        #    print( 'doing gc1' )
        #  gc.collect()
        #  gcCycle1 = 300
    # while shutdown == False:
    
    pwmA.deinit()
    pwmB.deinit()
    shdn.off()    # amplifier off
    print("sound_player() exiting")
# def sound_player():

# Launch the program using asyncio 
try:
  asyncio.run(main())

finally:
  asyncio.new_event_loop()
