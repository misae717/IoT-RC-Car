import time
from machine import Pin, I2C, PWM, Timer, UART, ADC
from pico_car import SSD1306_I2C, pico_car, ws2812b, ultrasonic

Motor = pico_car()  # Initialize pico_car with automated calibration
Motor.Car_Stop()
num_leds = 8
pixels = ws2812b(num_leds, 0)
pixels.fill(0, 0, 0)
pixels.show()

BZ = PWM(Pin(22))
BZ.freq(1000)

CM = [0, 330, 350, 393, 441, 495, 556, 624]
ultrasonic = ultrasonic()
i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=100000)
oled = SSD1306_I2C(128, 32, i2c)
oled = SSD1306_I2C(128, 32, i2c)
oled.fill(0)  
oled.text('IoT-SE322', 0, 0)  
oled.text('Saleh Alghannam', 0, 10)  
oled.show()  
uart = UART(0, 9600, bits=8, parity=None, stop=1, tx=Pin(16), rx=Pin(17))
dat = 0

Quantity_of_electricity = ADC(28)
Sound = ADC(27)
tim = Timer()


def tick(timer):
    w_power = int(Quantity_of_electricity.read_u16() / 65535 * 240)
    if w_power > 100:
        w_power = 100
    w_distance = ultrasonic.Distance_accurate()
    w_sounds = int(Sound.read_u16() / 65535 * 200)
    uart.write('$DAT')
    uart.write(str(w_distance))
    uart.write(',')
    uart.write(str(w_sounds))
    uart.write(',')
    uart.write(str(w_power))
    uart.write('#')


tim.init(freq=0.1, mode=Timer.PERIODIC, callback=tick)


def water():
    global i,dat
    i = 0
    while dat != b'M#':
        while uart.any() > 0:
            dat = uart.read(2)
        if i == 0:
            pixels.set_pixel(7,0,0,0)
            pixels.set_pixel(0,150,0,150)
            i = i + 1
        else:
            pixels.set_pixel(i-1,0,0,0)
            pixels.set_pixel(i,150,0,150)
            i = i + 1
            if i == 8:
                i = 0
        pixels.show()
        time.sleep(0.1)
    i = 0

def breathing():
    global i,dat
    i = 0
    brightness = 0
    fadeAmount = 1
    while dat != b'M#':
        while uart.any() > 0:
            dat = uart.read(2)
        for i in range(num_leds):
            pixels.set_pixel(i,0,brightness,brightness)
        pixels.show()
        brightness = brightness + fadeAmount
        if brightness <= 0 or brightness >= 200:
            fadeAmount = -fadeAmount
        time.sleep(0.005)
    i = 0


def horse():
    global dat
    while dat != b'M#':
        while uart.any() > 0:
            dat = uart.read(2)
        for i in range(num_leds):
            for j in range(num_leds):
                #pixel_num, red, green, blue
                pixels.set_pixel(j,abs(i+j)%10,abs(i-(j+3))%10,abs(i-(j+6))%10)
            pixels.show()
            time.sleep(0.05)

#Define the tracking sensor, 1-4 from left to right
#recognize that black is 0 and white is 1
#Tracing_1 Tracing_2 Tracing_3 Tracing_4
#    2         3        4          5     
Tracing_1 = machine.Pin(2, machine.Pin.IN)
Tracing_2 = machine.Pin(3, machine.Pin.IN)
Tracing_3 = machine.Pin(4, machine.Pin.IN)
Tracing_4 = machine.Pin(5, machine.Pin.IN)

def line():
    global dat
    oled.fill(0)
    while dat != b'V#':
        while uart.any() > 0:
            dat = uart.read(2)
                

        if (Tracing_1.value() == 0 or Tracing_2.value() == 0) and Tracing_4.value() == 0:
            Motor.Car_Right(120,120)
            for i in range(num_leds):
                pixels.set_pixel(i,0,255,0)
            oled.text('Turn Right', 0, 0)
            time.sleep(0.08)

        elif Tracing_1.value() == 0 and (Tracing_3.value() == 0 or Tracing_4.value() == 0):
            Motor.Car_Left(120,120)
            for i in range(num_leds):
                pixels.set_pixel(i,0,0,255)
            oled.text('Turn Left', 0, 0)
            time.sleep(0.08)
            

        elif Tracing_1.value() == 0:
            Motor.Car_Run(0,130)
            for i in range(num_leds):
                pixels.set_pixel(i,0,0,255)
            oled.text('Turn Left', 0, 0)
        

        elif Tracing_4.value() == 0:
            Motor.Car_Run(130,0)
            for i in range(num_leds):
                pixels.set_pixel(i,0,255,0)
            oled.text('Turn Right', 0, 0)

        elif Tracing_2.value() == 0 and Tracing_3.value() == 0:
            Motor.Car_Run(110,110)
            for i in range(num_leds):
                pixels.set_pixel(i,255,255,255)
            oled.text('Run', 0, 0)
            
        pixels.show()
        oled.show()
        oled.fill(0)

    pixels.fill(0,0,0)
    Motor.Car_Stop()
    BZ.duty_u16(0)

 
def avoid():
    global dat
    oled.fill(0)
    while dat != b'V#':
        while uart.any() > 0:
            dat = uart.read(2)
       
        distance = ultrasonic.Distance_accurate()
        print("distance is %d cm"%(distance) )
       
        oled.text('distance:', 0, 0)
        oled.text(str(distance), 75, 0)
        oled.show()
        oled.fill(0)
        
        if distance < 10:
            for i in range(num_leds):
                pixels.set_pixel(i,255,0,0)
            pixels.show()
            Motor.Car_Back(150,150)
            BZ.duty_u16(500)
            BZ.freq(CM[7])
            time.sleep(0.2)
            Motor.Car_Right(150,150)
            BZ.duty_u16(500)
            BZ.freq(CM[5])
            time.sleep(0.2)
            BZ.duty_u16(0)
        elif distance >= 10 and distance < 30:
            for i in range(num_leds):
                pixels.set_pixel(i,255,255,0)
            pixels.show()
            Motor.Car_Run(100,100)
        else:
            for i in range(num_leds):
                pixels.set_pixel(i,0,255,0)
            pixels.show()
            Motor.Car_Run(100,100)
        time.sleep(0.1)
    pixels.fill(0,0,0)
    Motor.Car_Stop()
    BZ.duty_u16(0)


def voice():
    global dat
    oled.fill(0)
    while dat != b'V#':
        while uart.any() > 0:
            dat = uart.read(2)
        
        sounds = Sound.read_u16()
        print(sounds)
        oled.text('Sound:', 0, 0)
        oled.text(str(sounds), 50, 0)
       
        if sounds > 22000:
            while sounds > 10000:
                Motor.Car_Stop()
                sounds = Sound.read_u16()
                print(sounds)
                time.sleep(0.001)
            Motor.Car_Run(255,255)
            BZ.duty_u16(500)
            BZ.freq(CM[1])
            pixels.set_pixel(2,150,0,150)
            pixels.set_pixel(3,150,0,150)
            pixels.show()
            time.sleep(0.03)
            BZ.duty_u16(500)
            BZ.freq(CM[2])
            pixels.set_pixel(2,0,0,0)
            pixels.set_pixel(3,0,0,0)
            pixels.set_pixel(1,150,0,150)
            pixels.set_pixel(4,150,0,150)
            pixels.show()
            time.sleep(0.03)
            BZ.duty_u16(500)
            BZ.freq(CM[3])
            pixels.set_pixel(1,0,0,0)
            pixels.set_pixel(4,0,0,0)
            pixels.set_pixel(0,150,0,150)
            pixels.set_pixel(5,150,0,150)
            pixels.show()
            time.sleep(0.03)
            BZ.duty_u16(500)
            BZ.freq(CM[4])
            pixels.set_pixel(0,0,0,0)
            pixels.set_pixel(5,0,0,0)
            pixels.set_pixel(6,150,0,150)
            pixels.set_pixel(7,150,0,150)
            pixels.show()
            time.sleep(0.03)
            BZ.duty_u16(500)
            BZ.freq(CM[5])
            pixels.set_pixel(0,0,0,0)
            pixels.set_pixel(5,0,0,0)
            pixels.set_pixel(6,150,0,150)
            pixels.set_pixel(7,150,0,150)
            pixels.show()
            time.sleep(0.03)
            BZ.duty_u16(500)
            BZ.freq(CM[6])
            pixels.set_pixel(6,0,0,0)
            pixels.set_pixel(7,0,0,0)
            pixels.show()
            BZ.duty_u16(0)
            sounds = 0
            oled.show()
            oled.fill(0)
            while sounds > 10000:
                Motor.Car_Stop()
                sounds = Sound.read_u16()
                print(sounds)
                time.sleep(0.001)
        else:
            Motor.Car_Stop()
            oled.show()
            oled.fill(0)
        time.sleep(0.01)
    pixels.fill(0,0,0)
    Motor.Car_Stop()
    BZ.duty_u16(0)

while True:
    
    while uart.any() > 0:
        dat = uart.read(2)
        
        if dat == b'X#':
            for oledi in range(128):
                for oledj in range(10):
                    oled.pixel(oledi, oledj,0)
            datoled_1 = uart.read(16)
            stroled_1 = str(datoled_1)
            stroled_1 = stroled_1.replace("b'", "")
            stroled_1 = stroled_1.replace("'", "")
            stroled_1 = stroled_1.replace("$", "")
            oled.text(stroled_1, 0, 0)
            oled.show()
            print(stroled_1)
        elif dat == b'Y#':
            for oledi in range(128):
                for oledj in range(10,20):
                    oled.pixel(oledi, oledj,0)
            datoled_2 = uart.read(16)
            stroled_2 = str(datoled_2)
            stroled_2 = stroled_2.replace("b'", "")
            stroled_2 = stroled_2.replace("'", "")
            stroled_2 = stroled_2.replace("$", "")
            oled.text(stroled_2, 0, 10)
            oled.show()
            print(stroled_2)
        elif dat == b'Z#':
            for oledi in range(128):
                for oledj in range(20,30):
                    oled.pixel(oledi, oledj,0)
            datoled_3 = uart.read(16)
            stroled_3 = str(datoled_3)
            stroled_3 = stroled_3.replace("b'", "")
            stroled_3 = stroled_3.replace("'", "")
            stroled_3 = stroled_3.replace("$", "")
            oled.text(stroled_3, 0, 20)
            oled.show()
            print(stroled_3)
        elif dat == b'W#':
            BBuzzer = uart.read(1)
            if BBuzzer == b'1':
                BZ.duty_u16(500)
                BZ.freq(277)
            elif BBuzzer == b'2':
                BZ.duty_u16(500)
                BZ.freq(311)
            elif BBuzzer == b'3':
                BZ.duty_u16(500)
                BZ.freq(370)
            elif BBuzzer == b'4':
                BZ.duty_u16(500)
                BZ.freq(415)
            elif BBuzzer == b'5':
                BZ.duty_u16(500)
                BZ.freq(466)

   
    if dat == b'A#':
        Motor.Car_Run(255,255)
    elif dat == b'B#':
        Motor.Car_Back(255,255)
    elif dat == b'C#':
        Motor.Car_Run(0,255)
    elif dat == b'D#':
        Motor.Car_Run(255,0)
    elif dat == b'E#':
        Motor.Car_Left(255,255)
    elif dat == b'F#':
        Motor.Car_Right(255,255)
    elif dat == b'0#':
        Motor.Car_Stop()
  
    elif dat == b'1#':
        BZ.duty_u16(500)
        BZ.freq(262)
    elif dat == b'2#':
        BZ.duty_u16(500)
        BZ.freq(294)
    elif dat == b'3#':
        BZ.duty_u16(500)
        BZ.freq(330)
    elif dat == b'4#':
        BZ.duty_u16(500)
        BZ.freq(349)
    elif dat == b'5#':
        BZ.duty_u16(500)
        BZ.freq(392)
    elif dat == b'6#':
        BZ.duty_u16(500)
        BZ.freq(440)
    elif dat == b'7#':
        BZ.duty_u16(500)
        BZ.freq(494)
    elif dat == b'8#':
        BZ.duty_u16(500)
        BZ.freq(523)
    elif dat == b'O#':
        BZ.duty_u16(0)
  
    elif dat == b'G#':
        pixels.fill(255,0,0)
        pixels.show()
    elif dat == b'H#':
        pixels.fill(0,255,0)
        pixels.show()
    elif dat == b'I#':
        pixels.fill(0,0,255)
        pixels.show()
    elif dat == b'J#':
        pixels.fill(255,255,0)
        pixels.show()
    elif dat == b'K#':
        pixels.fill(0,255,255)
        pixels.show()
    elif dat == b'L#':
        pixels.fill(255,0,255)
        pixels.show()
    elif dat == b'N#':
        water()
    elif dat == b'P#':
        horse()
    elif dat == b'Q#':
        breathing()
    elif dat == b'M#':
        pixels.fill(0,0,0)
        pixels.show()
        i = 0
        brightness = 0
        fadeAmount = 1
   
    elif dat == b'S#':
        line()
    elif dat == b'T#':
        avoid()
    elif dat == b'U#':
        voice()
    elif dat == b'V#':
        oled.fill(0)
        oled.show()
        pixels.fill(0,0,0)
        pixels.show()
        Motor.Car_Stop()
        BZ.duty_u16(0)
    time.sleep(0.01)

