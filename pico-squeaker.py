# pico-squeaker
# presents a webpage interface which controls two PWM outputs and an amplifier enable signal
# v1.0 deployed 2022-8-7
# adding an "easy" interface page in addition to the advanced controls
# disabling signal strength parser: seems problematic
# v1.1 deployed 2022-8-11
# adding battery voltage monitor (input to ADC1 midpoint of a 67.5k-150k divider network)
# adding input of the charging indicator from the bq24074
# v1.2 deployed 2022-8-20
#
# imports used in both threads
from time import sleep
import _thread
import gc


# The Web Server Thread - turns GET requests into actions onboard
def core0_thread():
    import network
    import socket
    from rp2 import country
    from machine import Pin, ADC

    global counter
    global shutdown
    global freq1
    global freq2
    global fRng1
    global fRng2
    global oscP1
    global oscP2
    global tSlic

    counter  = 0
    shutdown = False
    freq1 = 20000
    freq2 = 19000
    fRng1 = 0.3
    fRng2 = 0.1
    oscP1 = 225
    oscP2 = 3500
    tSlic = 0.04
# Temperature sensor and Battery Voltage
    sensor_temp = ADC(4)
    batt_adc    = ADC(1)
    conversion_factor = 3.3 / 65535.0   
# Select the onboard LED
    led = machine.Pin("LED", machine.Pin.OUT)
    led.value(1)
# Charging indicator from bq24074
    chg_pin = machine.Pin(20, machine.Pin.IN)
# Set country code, opens legal WiFi channels
    country('US')
# GUI framework
    html = """<!DOCTYPE html>
<html>
  <head><title>MangoCats Pico W</title></head>
  <body><center><font size="+6"><h2>MangoCats <a href="/">Pico</a> W</h2>
    <table width="95%%" style="text-align:center"><tr>
      <td><a href="/light/on">LED ON</a></td>
      <td>{}<br/>{}F<br/>{}V<br/>{}<br/>{}dBm</td>
      <td><a href="/light/off">LED OFF</a></td>
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
    
    easyHtml = """<!DOCTYPE html>
<html>
  <head><title>MangoCats Pico W</title></head>
  <body><center><font size="+6"><h2>MangoCats <a href="/advanced">Pico</a> W</h2>
    <table width="95%%" style="text-align:center"><tr>
      <td>{}F<br/>{}V</td>
      <td>{}</td>
      <td>{}s<br/>{}</td>
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
    
    print("core0_thread starting")

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect('ImNot','Telling')

# Wait for connect or fail
    max_wait = 25
    while max_wait > 0:
      if wlan.status() < 0 or wlan.status() >= 3:
        break
      max_wait -= 1
      print('waiting for connection...',max_wait)
      sleep(1)
  
# Handle connection error
    if wlan.status() != 3:
      raise RuntimeError('network connection failed')
    else:
      print('connected')
      status = wlan.ifconfig()
      print( 'ip = ' + status[0] )
      led.value(0)
   
# Open socket
    addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)
 
    print('listening on', addr)
 
# Listen for connections
    easyPage = True
    while True:
      try:
        stateis = "Hello"
        cl, addr = s.accept()
        request = str( cl.recv(2048) )
        request = request[6:19]
        print('client connected from', addr, request )
        
        if 0 == request.find('/favicon.ico'):
          cl.send('HTTP/1.0 200 OK\r\nContent-type: image/x-icon\r\nContent-length: 0\r\n\r\n')
          cl.close()
        else:
          if 0 == request.find('/ '):
            easyPage = True  
            
          elif 0 == request.find('/sound/'):
            counter = int(request[7:9])
            stateis = "Sound On %s" % counter
            # easyPage is unchanged
            
          elif 0 == request.find('/light/on'):
            led.value(1)
            stateis = "LED is ON"
            easyPage = False
 
          elif 0 == request.find('/light/off'):
            led.value(0)
            stateis = "LED is OFF"
            easyPage = False
          
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
          
          elif 0 == request.find('/exit'):
            stateis = "Exiting."
            shutdown = True
            easyPage = True
            
          elif 0 == request.find('/advanced'):
            stateis = "Advanced"
            easyPage = False
            
          else:  
            stateis = "Unknown cmd"
            # easyPage is unchanged

# take a temperature reading
          reading = sensor_temp.read_u16() * conversion_factor 
          temperature = 27.0 - (reading - 0.706)/0.001721
          farenheit = temperature * 9.0 / 5.0 + 32.0
          
# read the battery voltage          
          reading = batt_adc.read_u16() * conversion_factor
          battV = reading * (150.0+67.5) / 150.0

# Read strongest connection to ssid
          maxrssi = -200
          if easyPage == False:
            apl = wlan.scan()
            if isinstance(apl, tuple):
              for ap in apl:
                if isinstance(ap, tuple):
                  if len(ap) > 3:
                    if ap[0].decode("utf-8") == ssid:
                      if isinstance(ap[3], int):
                        if ap[3] > maxrssi:
                          maxrssi = ap[3] 

# Send the updated GUI to the browser
          cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
          if easyPage == True:
            if chg_pin.value() == True:
              cmsg = "Not Charging"
            else:
              cmsg = "Charging"
            cl.send( easyHtml.format( farenheit, battV, stateis, counter, cmsg ) )
            led.value(0) # LED is always off when using the easy interface
          else:
            cl.send( html.format( stateis, farenheit, battV, cmsg, maxrssi, freq1, freq2, fRng1, fRng2, oscP1, oscP2, tSlic ) )
          cl.close()
    
          if shutdown == True:
            led.value(1) # light on to indicate shutting down
            s.close()
            print("core0_thread exiting")
            sleep(2)     # giving core1_thread time to exit first
            led.value(0) # light off when shutdown is complete
            break
        
        gc.collect()
        
      except OSError as e:
        cl.close()
        print('connection closed')
        
#     try:
#   while True:
# def core0_thread():

# The sound player thread
# anytime counter is > 0 sound will play
def core1_thread():
    from machine import Pin, PWM
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
    pwmA = PWM( Pin ( 22 ) ) # GP22, pin 29
    pwmA.freq( freq1 ) 
    pwmA.duty_u16( 0 )
    pwmB = PWM( Pin ( 21 ) ) # GP21, pin 27
    pwmB.freq( freq2 ) 
    pwmB.duty_u16( 0 )
    shdn = machine.Pin( 19, machine.Pin.OUT ) # GP19, pin 25
    shdn.value(0)            # default to amplifier off
    
    print("core1_thread starting")
    targ = math.pi * -30.0
    while True:
        if counter > 0:
          if pwmOff == True:
            pwmA.duty_u16( 32768 )   # duty 50% (65535/2)
            pwmB.duty_u16( 32768 )
            shdn.on()                # amplifier on
#            print("Sound on")
            pwmOff = False
#          print( counter )  
          counter -= 1
          
          oscC1 = 1000.0 / float(oscP1)  # convert the modulation periods (in ms) to coefficients
          oscC2 = 1000.0 / float(oscP2)
          fMod1 = float(freq1) * fRng1   # frequency modulation depths
          fMod2 = float(freq2) * fRng2
          tDone = targ + math.pi * 2.0
          slicI = math.pi * 2.0 * tSlic;
          while targ < tDone:            # work through the modulators for the next second
            pwmA.freq( freq1 + int( fMod1 * math.sin( targ * oscC1 ) ) )
            pwmB.freq( freq2 + int( fMod2 * math.sin( targ * oscC2 ) ) )
            targ += slicI;
            sleep( tSlic )
            
        else: # counter is not > 0
          if pwmOff == False:
            pwmA.duty_u16( 0 )     # stop oscillation
            pwmB.duty_u16( 0 )
            shdn.off()             # amplifier off
#            print("Sound off")
            targ = math.pi * -30.0 # keep targ in a reasonable range
            pwmOff = True        
          sleep( 0.2 )             # save a bit of power
          
        if shutdown == True:
          pwmA.deinit()
          pwmB.deinit()
          shdn.off()    # amplifier off
          print("core1_thread exiting")
          break
        
        gc.collect()
    # while True:
# def core1_thread():
    
# Launch both threads
second_thread = _thread.start_new_thread(core1_thread, ())
core0_thread()