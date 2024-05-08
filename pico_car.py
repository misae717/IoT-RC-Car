import array, time
from machine import Pin, PWM
import rp2
import framebuf
from onewire import OneWire

WHEEL_DIAMETER_MM = 44  
WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_MM * 3.14159265 / 1000  

class pico_car:
    def __init__(self):
        
        self.R_B = PWM(Pin(11))
        self.R_A = PWM(Pin(10))
        self.L_B = PWM(Pin(13))
        self.L_A = PWM(Pin(12))

        
        self.R_B.freq(1000)
        self.R_A.freq(1000)
        self.L_B.freq(1000)
        self.L_A.freq(1000)

        
        self.motor_calibration = {
            'left': [],
            'right': []
        }

        
        self.calibrate_motors()

    def measure_rotation_time(self, pwm_a, pwm_b, duty_cycle):
        """Measure the time it takes for a wheel to complete one rotation."""
       
        pwm_a.duty_u16(0)
        pwm_b.duty_u16(duty_cycle)
        start_time = time.ticks_us()

        
        rotation_time_us = 800000 / duty_cycle  # Estimat
        time.sleep_us(rotation_time_us)

        
        elapsed_time_us = time.ticks_diff(time.ticks_us(), start_time)
        elapsed_time_s = elapsed_time_us / 1e6

        return elapsed_time_s

    def estimate_rpm(self, elapsed_time_s):
        """Estimate RPM based on the elapsed time for one wheel rotation."""
        rotations_per_second = 1 / elapsed_time_s
        return rotations_per_second * 60

    def calibrate_motor(self, motor_side):
        """Calibrate a single motor and store calibration data."""
        calibration_data = []
        pwm_a, pwm_b = (self.R_A, self.R_B) if motor_side == 'right' else (self.L_A, self.L_B)

        for duty_cycle in range(1000, 65536, 1000):
            elapsed_time_s = self.measure_rotation_time(pwm_a, pwm_b, duty_cycle)
            rpm = self.estimate_rpm(elapsed_time_s)
            calibration_data.append((duty_cycle, rpm))

        pwm_a.duty_u16(0)
        pwm_b.duty_u16(0)
        return calibration_data

    def calibrate_motors(self):
        """Automatically calibrate both motors."""
        self.motor_calibration['left'] = self.calibrate_motor('left')
        self.motor_calibration['right'] = self.calibrate_motor('right')

    def get_duty_for_rpm(self, target_rpm, motor_side='left'):
        calibration_data = self.motor_calibration[motor_side]
        for i in range(len(calibration_data) - 1):
            if calibration_data[i][1] <= target_rpm <= calibration_data[i + 1][1]:
                dc1, rpm1 = calibration_data[i]
                dc2, rpm2 = calibration_data[i + 1]
                return int(dc1 + (dc2 - dc1) * (target_rpm - rpm1) / (rpm2 - rpm1))
        return calibration_data[-1][0] if target_rpm > calibration_data[-1][1] else calibration_data[0][0]

    def set_motor_rpm(self, left_rpm, right_rpm):
        left_dc = self.get_duty_for_rpm(left_rpm, 'left')
        right_dc = self.get_duty_for_rpm(right_rpm, 'right')

        self.R_B.duty_u16(right_dc)
        self.R_A.duty_u16(0)
        self.L_B.duty_u16(left_dc)
        self.L_A.duty_u16(0)

    def Car_Run(self, left_rpm, right_rpm):
        self.set_motor_rpm(left_rpm, right_rpm)

    def Car_Stop(self):
        self.R_B.duty_u16(0)
        self.R_A.duty_u16(0)
        self.L_B.duty_u16(0)
        self.L_A.duty_u16(0)

    def Car_Back(self, left_rpm, right_rpm):
        left_dc = self.get_duty_for_rpm(left_rpm, 'left')
        right_dc = self.get_duty_for_rpm(right_rpm, 'right')

        self.R_B.duty_u16(right_dc)
        self.R_A.duty_u16(0)
        self.L_B.duty_u16(0)
        self.L_A.duty_u16(left_dc)

    def Car_Left(self, left_rpm, right_rpm):
        left_dc = self.get_duty_for_rpm(left_rpm, 'left')
        right_dc = self.get_duty_for_rpm(right_rpm, 'right')

        self.R_B.duty_u16(0)
        self.R_A.duty_u16(right_dc)
        self.L_B.duty_u16(0)
        self.L_A.duty_u16(left_dc)

    def Car_Right(self, left_rpm, right_rpm):
        left_dc = self.get_duty_for_rpm(left_rpm, 'left')
        right_dc = self.get_duty_for_rpm(right_rpm, 'right')

        self.R_B.duty_u16(right_dc)
        self.R_A.duty_u16(0)
        self.L_B.duty_u16(left_dc)
        self.L_A.duty_u16(0)

    def servo180(self, num, angle):
        angle = angle * 72.2222 + 3535
        if num == 1:
            S1.duty_u16(int(angle))
        elif num == 2:
            S2.duty_u16(int(angle))
        elif num == 3:
            S3.duty_u16(int(angle))
        elif num == 4:
            S4.duty_u16(int(angle))

    def servo270(self, num, angle):
        angle = angle * 48.1481 + 3535
        if num == 1:
            S1.duty_u16(int(angle))
        elif num == 2:
            S2.duty_u16(int(angle))
        elif num == 3:
            S3.duty_u16(int(angle))
        elif num == 4:
            S4.duty_u16(int(angle))

    def servo360(self, num, angle):
        angle = angle * 36.1111 + 3535
        if num == 1:
            S1.duty_u16(int(angle))
        elif num == 2:
            S2.duty_u16(int(angle))
        elif num == 3:
            S3.duty_u16(int(angle))
        elif num == 4:
            S4.duty_u16(int(angle))

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()

class ws2812b:
    def __init__(self, num_leds, state_machine, delay=0.001):
        self.pixels = array.array("I", [0 for _ in range(num_leds)])
        self.sm = rp2.StateMachine(state_machine, ws2812, freq=8000000, sideset_base=Pin(6))
        self.sm.active(1)
        self.num_leds = num_leds
        self.delay = delay
        self.brightnessvalue = 255

    def brightness(self, brightness=None):
        if brightness is None:
            return self.brightnessvalue
        if brightness < 1:
            brightness = 1
        if brightness > 255:
            brightness = 255
        self.brightnessvalue = brightness

    def set_pixel(self, pixel_num, red, green, blue):
        blue = round(blue * (self.brightness() / 255))
        red = round(red * (self.brightness() / 255))
        green = round(green * (self.brightness() / 255))

        self.pixels[pixel_num] = blue | red << 8 | green << 16

    def show(self):
        for i in range(self.num_leds):
            self.sm.put(self.pixels[i], 8)
        time.sleep(self.delay)

    def fill(self, red, green, blue):
        for i in range(self.num_leds):
            self.set_pixel(i, red, green, blue)
        time.sleep(self.delay)

class ultrasonic:
    def __init__(self):
        self.Trig = Pin(0, Pin.OUT)
        self.Echo = Pin(1, Pin.IN)

    def Distance(self):
        self.Trig.value(0)
        time.sleep(0.000002)
        self.Trig.value(1)
        time.sleep(0.000015)
        self.Trig.value(0)
        t2 = 0
        while not self.Echo.value():
            t1 = 0
        t1 = 0
        while self.Echo.value():
            t2 += 1

        time.sleep(0.001)
        return ((t2 - t1) * 2.0192 / 10)

    def Distance_accurate(self):
        num = 0
        ultrasonic = []
        while num < 5:
            distance = self.Distance()
            while int(distance) == -1:
                distance = self.Distance()
                return int(999)
            while int(distance) >= 500 or int(distance) == 0:
                distance = self.Distance()
                return int(999)
            ultrasonic.append(distance)
            num = num + 1
            time.sleep(0.01)
        distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3]) / 3
        return int(distance)

class SSD1306:
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height
        self.poweron()
        self.init_display()

    def init_display(self):
        for cmd in (
                0xae, 
                0x20, 0x00, 
                0x40, 
                0xa0,  
                0xa8, self.height - 1,  
                0xc0, 
                0xd3, 0x00,  
                0xda, 0x02 if self.height == 32 else 0x12,  
                0xd5, 0x80,  
                0xd9, 0x22 if self.external_vcc else 0xf1,  
                0xdb, 0x30,  
                0x81, 0xff,  
                0xa4,  
                0xa6,  
                0x8d, 0x10 if self.external_vcc else 0x14,  
                0xaf): 
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(0xae)

    def contrast(self, contrast):
        self.write_cmd(0x81)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(0xa6 | (invert & 1))

    def show(self):
        x0 = 0
        x1 = self.width - 1
        if self.width == 64:
            x0 += 32
            x1 += 32
        self.write_cmd(0x21)
        self.write_cmd(x0)
        self.write_cmd(x1)
        self.write_cmd(0x22)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_framebuf()

    def fill(self, col):
        self.framebuf.fill(col)

    def pixel(self, x, y, col):
        self.framebuf.pixel(x, y, col)

    def scroll(self, dx, dy):
        self.framebuf.scroll(dx, dy)

    def text(self, string, x, y, col=1):
        self.framebuf.text(string, x, y, col)

class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3c, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        self.buffer = bytearray(((height // 8) * width) + 1)
        self.buffer[0] = 0x40
        self.framebuf = framebuf.FrameBuffer1(memoryview(self.buffer)[1:], width, height)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_framebuf(self):
        self.i2c.writeto(self.addr, self.buffer)

    def poweron(self):
        pass

class ir:
    def __init__(self):
        self.Pin = Pin(7)
        self.Pin.value(1)
        self.ir_repeat_cnt = 0
        self.irdata = 0xfe

    def Getir(self):
        if self.Pin.value() == 0:
            self.ir_repeat_cnt = 0
            count = 0
            while self.Pin.value() == 0:
                count += 1
                time.sleep(0.00003)
            count = 0
            while self.Pin.value() == 1 and count < 160:
                count += 1
                time.sleep(0.00003)
            idx = 0
            cnt = 0
            data = [0, 0, 0, 0]
            for i in range(0, 32):
                count = 0
                while self.Pin.value() == 0 and count < 30:
                    count += 1
                    time.sleep(0.00003)
                count = 0
                while self.Pin.value() == 1 and count < 80:
                    count += 1
                    time.sleep(0.00003)
                if count > 35:
                    data[idx] |= 1 << cnt
                if cnt == 7:
                    cnt = 0
                    idx += 1
                else:
                    cnt += 1
            if data[0] + data[1] == 0xFF and data[2] + data[3] == 0xFF:
                self.irdata = data[2]
        else:
            if self.ir_repeat_cnt > 110:
                self.ir_repeat_cnt = 0
                self.irdata = 0xfe
            else:
                time.sleep(0.001)
                self.ir_repeat_cnt += 1
        if self.irdata != None:
            if self.irdata != 254:
                return self.irdata

class ds:
    def __init__(self, unit='c', resolution=12):
        self.pin = 7
        self.no_addr = 0
        self.addr = self.getaddr()
        self.unit = unit
        self.res = resolution

    def getaddr(self):
        ow = OneWire(Pin(self.pin))
        a = ow.scan()
        for i in a:
            self.no_addr += 1
        return a

    def read(self):
        if self.no_addr == 0:
            print("no sensors detected")
        if self.no_addr >= 1:
            temp_array = []
            for i in range(1, self.no_addr + 1):
                temp_array.append(self._request(self.addr[i - 1]))
                return temp_array

    def _request(self, addr):
        self._res(addr)
        ow = OneWire(Pin(self.pin))
        ow.reset()
        ow.select_rom(addr)
        ow.writebyte(0x44)
        if self.res == 12:
            time.sleep_ms(1000)
        if self.res == 11:
            time.sleep_ms(400)
        if self.res == 10:
            time.sleep_ms(200)
        if self.res == 9:
            time.sleep_ms(100)
        ow.reset()
        ow.select_rom(addr)
        ow.writebyte(0xbe)
    
        LSB = ow.readbyte()  
        MSB = ow.readbyte() 
        ow.readbyte()
        ow.readbyte()
        ow.readbyte()  
        ow.readbyte()
        ow.readbyte()
        ow.readbyte()
        ow.readbyte()
        ow.reset() 


        d_LSB = float(0)
        d_MSB = float(0)
        count = 0
        b = bin(LSB)
        b2 = bin(MSB)
        b3 = ""
        l = 10 - len(b2)
        for i in range(l):
            if len(b2) < 10:
                b3 += "0"
        b2 = b3 + b2
        b4 = ""
        l = 10 - len(b)
        for i in range(l):
            if len(b) < 10:
                b4 += "0"
        b5 = b4 + b
        for i in b5:
            if count == 2:
                if i == '1':
                    d_LSB += 2**3
            if count == 3:
                if i == '1':
                    d_LSB += 2**2
            if count == 4:
                if i == '1':
                    d_LSB += 2**1
            if count == 5:
                if i == '1':
                    d_LSB += 2**0
            if count == 6:
                if i == '1':
                    d_LSB += 2**-1
            if count == 7:
                if i == '1':
                    d_LSB += 2**-2
            if count == 8:
                if i == '1':
                    d_LSB += 2**-3
            if count == 9:
                if i == '1':
                    d_LSB += 2**-4
            count += 1
        count = 0
        sign = 1
        for i in b2:
            if count == 6:
                if i == '1':
                    sign = -1
            if count == 7:
                if i == '1':
                    d_MSB += 2**6
            if count == 8:
                if i == '1':
                    d_MSB += 2**5
            if count == 9:
                if i == '1':
                    d_MSB += 2**4
            count += 1
        temp = (d_LSB + d_MSB) * sign

        return temp

    def _res(self, addr):
        ow = OneWire(Pin(self.pin))
        ow.reset()
        ow.select_rom(addr)
        ow.writebyte(0x4e)
        if self.res == 12:
            ow.writebyte(0x7f)
            ow.writebyte(0x7f)
            ow.writebyte(0x7f)
        elif self.res == 11:
            ow.writebyte(0x5f)
            ow.writebyte(0x5f)
            ow.writebyte(0x5f)
        elif self.res == 10:
            ow.writebyte(0x3f)
            ow.writebyte(0x3f)
            ow.writebyte(0x3f)
        elif self.res == 9:
            ow.writebyte(0x1f)
            ow.writebyte(0x1f)
            ow.writebyte(0x1f)
        ow.reset()
