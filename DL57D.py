"""
Коли процесор обчислює еквівалент імпульсу,
використовуйте параметр мікрокроку × 200,
об отримати значення мікрокроку в одиницях імп / об.
ENA -- 5 ms --> DIR -- 100 micro s --> PUL
PUL width > 5 micro s
LOW lvl width > 5 micro s
1/20 < EGR < 20 Electronic gear ratio
Number_of_wires = 1000
Encoder resolution ratio indicates the count of pulses output by the encoder during one motor rotation.
gear_ratio = P006 / P007
gear_ratio = Number_of_wires * 4 / Displacement_per_load_shaft_revolution * m / n
"""
import sys

import pigpio
from time import sleep

BAD_CODE: int = -1


class DL57D:
    """
    Class for Raspberry Pi control of DL57D driver and NEMA servo-motor
    Connections of Encoder:
        Black -> EB+ -> Yellow | Brown  -> EB-  -> Green
        Gray  -> EA+ -> Black  | White  -> EA-  -> Blue
        Blue  -> VCC -> Red    | Purple -> EGND -> White
    """
    EGR_MIN: float = 0.05  # Min electronical gear ratio
    EGR_MAX: int = 20  # Max electronical gear ratio
    STEPS_RATIO: int = 200  # Steps for motor (without microsteps)

    SLEEP_AFTER_ENA: float = 5e-6  # 5 microseconds after ENA before DIR
    SLEEP_AFTER_DIR: float = 100e-6  # 100 microseconds after DIR change (Before PUL)
    PULL_MIN_PERIOD: float = 5e-6  # 5 microseconds for signal period
    PULL_MAX_FREQ: int = 200e3  # 1 / MIN_PERIOD  Max freq of PUL signals
    LVL_MIN_DURATION: float = 2.5e-6  # 2.5 microseconds for signal duration (LOW or HIGH lvl)
    NORMAL_SPEED_RPM: float = 1000  # 1000 r/min rotation per minute is 166 rotation per second
    MAX_SPEED_RPM: float = 2000  # 2000 r/min  rotation per minute is 332 rotation per second

    ENCODER_RESOLUTION: int = 1000  # full rotation / min
    """
                                   Hertz

           1: 40000 20000 10000 8000 5000 4000 2500 2000 1600
               1250  1000   800  500  400  250  200  100   50

           2: 20000 10000  5000 4000 2500 2000 1250 1000  800
                625   500   400  250  200  125  100   50   25

           4: 10000  5000  2500 2000 1250 1000  625  500  400
                313   250   200  125  100   63   50   25   13
    sample
     rate
     (us)  5:  8000  4000  2000 1600 1000  800  500  400  320
                250   200   160  100   80   50   40   20   10

           8:  5000  2500  1250 1000  625  500  313  250  200
                156   125   100   63   50   31   25   13    6

          10:  4000  2000  1000  800  500  400  250  200  160
                125   100    80   50   40   25   20   10    5
    """
    PWM_FREQ_DICT: dict[int: tuple[int | float]] = {1: (40e3, 20e3, 10e3, 8e3, 5e3, 4e3, 2500, 2000, 1600,
                                                        1250, 1000, 800, 500, 400, 250, 200, 100, 50),
                                                    2: (20e3, 10e3, 5e3, 4e3, 2500, 2000, 1250, 1000, 800,
                                                        625, 500, 400, 250, 200, 125, 100, 50, 25),
                                                    4: (10e3, 5e3, 2500, 2e3, 1250, 1000, 625, 500, 400,
                                                        313, 250, 200, 125, 100, 63, 50, 25, 13),
                                                    5: (8e3, 4e3, 2e3, 1600, 1000, 800, 500, 400, 320,
                                                        250, 200, 160, 100, 80, 50, 40, 20, 10),
                                                    8: (5e3, 2500, 1250, 1000, 625, 500, 313, 250, 200,
                                                        156, 125, 100, 63, 50, 31, 25, 13, 6),
                                                    10: (4e3, 2e3, 1000, 800, 500, 400, 250, 200, 160,
                                                         125, 100, 80, 50, 40, 25, 20, 10, 5)}

    # PWM_FREQ_DEFAULT_RANGE = PWM_FREQ_DICT[5]

    def __init__(self,
                 pul_gpio: int = 18,  # PIN 12  # Must-have for sending move command
                 ena_gpio: int = 13,  # PIN 33  # For enabling driver
                 dir_gpio: int = 23,  # PIN 16  # Direction command
                 pend_gpio: int | None = None,  # Could be not connected, 12 gpio PIN 32, For receiving when in position
                 alm_gpio: int | None = None,  # Could be not connected, For Receiving alarm

                 microstep: int = 10,  # Pulses per step (200 steps -> full rotation)
                 reductor_ratio: int = 100,  # Amount of driver rotation needs for 1 system rotation
                 pigpiod_sample_rate: int = 5,  # 5 microseconds

                 sectors: int = 400):
        """
        :param pul_gpio: raspberry pi GPIO num of PUL + connection to pin
        :param ena_gpio: raspberry pi GPIO num of ENA + connection to pin
        :param dir_gpio: raspberry pi GPIO num of DIR + connection to pin
        :param pend_gpio: raspberry pi GPIO num of PEND + connection to pin
        :param alm_gpio: raspberry pi GPIO num of ALM + connection to pin
        :param microstep:  driver setted P001 parameter
        :param reductor_ratio:  reductor ratio of connected to NEMA motor
        :param pigpiod_sample_rate:  sample rate of tunned pigpio daemon (pigpiod)
        """
        print('Init driver')
        self.pul_gpio: int = pul_gpio  # Clockwise mode OUTPUT GPIO
        self.ena_gpio: int = ena_gpio  # OUTPUT GPIO (ON / OFF) 1 / 0
        self.dir_gpio: int = dir_gpio  # OUTPUT GPIO (CW / CCW) Directions
        self.pend_gpio: int | None = pend_gpio  # INPUT GPIO When in position
        self.alm_gpio: int | None = alm_gpio  # INPUT GPIO Alarm message from driver
        self.gpios: dict = {'PULL': self.pul_gpio,
                            'ENA': self.ena_gpio,
                            'DIR': self.dir_gpio,
                            'PEND': self.pend_gpio,
                            'ALM': self.alm_gpio}

        self.sectors = sectors
        self.microstep: int = microstep  # Pulses (microsteps) per one step (200 steps -> full rotation)
        print(f'Microstep is {self.microstep}')
        self.reductor_ratio: int = reductor_ratio  # Reduction in degrees
        self.full_rotate_steps: int = self.microstep * self.STEPS_RATIO  # Pulses per full rotation
        print(f'Full rotation steps is {self.full_rotate_steps}')
        self.seconds_for_rotate: float = 60 / self.NORMAL_SPEED_RPM  # Time in seconds for 1 rotation with normal speed
        print(f'Seconds for rotate is {self.seconds_for_rotate}')

        # self.lvl_duration: float = self.seconds_for_rotate / self.full_rotate_steps / 2  # Period indeed 2 durations
        self.max_speed: float = self.convert_lvl_duration_to_speed(lvl_duration=self.LVL_MIN_DURATION)
        print(f'Max speed is {self.max_speed} for microstep {self.microstep}')
        if self.max_speed > self.MAX_SPEED_RPM:
            self.max_speed = self.MAX_SPEED_RPM
        self.lv_min_duration: float = self.convert_speed_to_lvl_duration(speed=self.MAX_SPEED_RPM)
        if self.lv_min_duration < self.LVL_MIN_DURATION:  # lvl duration couldn't be less 2.5 us
            print(f'For microstep {microstep} and lvl duration {self.LVL_MIN_DURATION} speed {self.MAX_SPEED_RPM}'
                  f'Unavailable')
            print(f'Steps duration set to {self.LVL_MIN_DURATION}')
            self.lv_min_duration = self.LVL_MIN_DURATION
        print(f'Lvl min duration set to {self.lv_min_duration}')
        if self.full_rotate_steps % self.sectors != 0:
            print(f'Bad sectors {self.sectors} for full circle of {self.full_rotate_steps} steps')
        self.sector_steps: int = int(self.full_rotate_steps / self.sectors)
        # microsteps per degree ( 5/9 of microsteps ) (if sectors 360)
        print(f'Degree steps is {self.sector_steps}')

        self.pigpiod_sample_rate: int = pigpiod_sample_rate  # 5 microseconds of sample rate by default
        print(f'Pigpiod sample rate is {self.pigpiod_sample_rate}')

        if self.pigpiod_sample_rate in self.PWM_FREQ_DICT:
            self.pwm_freqs: tuple = self.PWM_FREQ_DICT[self.pigpiod_sample_rate]  # Range of values is constant
            print(f'PWM freqs is {self.pwm_freqs}')
        else:
            print(f'Not available pidpiod sample rate {self.pigpiod_sample_rate} available: '
                  f'{[_ for _ in self.PWM_FREQ_DICT]}')  # Key freq from dict
            sys.exit(BAD_CODE)

        print('Init gpio')
        self.pi: pigpio.pi = pigpio.pi()
        if not self.pi.connected:
            print('PI not connected\n'
                  'EXIT . . .')
            exit(BAD_CODE)  # Exiting program
        # SETUP pins GPIO modes (I/O)
        print('Set up pins MODES')
        self.pi.set_mode(self.pul_gpio, pigpio.OUTPUT)  # OUTPUT PULL signal mode
        if self.ena_gpio is not None:
            self.pi.set_mode(self.ena_gpio, pigpio.OUTPUT)  # OUTPUT ENA mode
            print('ENA output connected')
        if self.dir_gpio is not None:  # If connected
            self.pi.set_mode(self.dir_gpio, pigpio.OUTPUT)  # OUTPUT direction
            print('DIR output connected')
        if self.alm_gpio is not None:  # If connected
            self.pi.set_mode(self.alm_gpio, pigpio.INPUT)  # INPUT ALM error
            print('ALM input connected')
        if self.pend_gpio is not None:
            self.pi.set_mode(self.pend_gpio, pigpio.INPUT)  # INPUT PEND when in position
            print('PEND input connected')

        print('State before turn ON:')
        self.print_state()
        print('Turn ON driver (ENA LOW)')
        if self.ena_gpio is not None:
            self.change_lvl(gpio_name='ENA', lvl=pigpio.LOW)  # Turn ON (0) by default
        if self.dir_gpio is not None:
            self.change_lvl(gpio_name='DIR', lvl=pigpio.HIGH)  # Turn ON (1) by default

    def change_lvl(self, gpio_name: str, lvl: int | None = None) -> None:  # Turn off PULL
        """
        Change lvl of gpio channel, safe to use due to block PULL while changing
        And have required sleeps after ENA and DIR changes
        :param gpio_name: Name of gpio channel
        :param lvl: 1 or 0 lvl
        :return: None
        """
        # self.pull_off()  # Block PULL signal
        if gpio_name in self.gpios:
            gpio: int | None = self.gpios[gpio_name]  # GPIO of gpio name channel (GPIO of PUL exmpl)
            if gpio is None:
                print(f'No gpio connection of {gpio_name} channel')
            else:
                if lvl is None:  # Default value change
                    if self.pi.read(gpio=gpio) == pigpio.HIGH:
                        self.pi.write(gpio=gpio, level=pigpio.LOW)
                        print(f'{gpio_name} lvl {pigpio.LOW}')
                        if gpio_name == 'ENA':  # After ENA needs to sleep 5 us Before DIR
                            sleep(self.SLEEP_AFTER_ENA)
                        elif gpio_name == 'DIR':  # After DIR needs to sleep 100us Before ENA
                            sleep(self.SLEEP_AFTER_DIR)
                    else:  # Was LOW
                        self.pi.write(gpio=gpio, level=pigpio.HIGH)  # Now HIGH
                        print(f'{gpio_name} lvl {pigpio.HIGH}')
                        if gpio_name == 'ENA':  # After ENA needs to sleep 5 us Before DIR
                            sleep(self.SLEEP_AFTER_ENA)
                        elif gpio_name == 'DIR':  # After DIR needs to sleep 100us Before ENA
                            sleep(self.SLEEP_AFTER_DIR)
                elif lvl == pigpio.LOW or lvl == pigpio.HIGH:
                    self.pi.write(gpio=gpio, level=lvl)
                    print(f'{gpio_name} lvl {lvl}')
                    if gpio_name == 'ENA':  # After ENA needs to sleep 5 us Before DIR
                        sleep(self.SLEEP_AFTER_ENA)
                    elif gpio_name == 'DIR':  # After DIR needs to sleep 100us Before ENA
                        sleep(self.SLEEP_AFTER_DIR)
                else:
                    print(f'Incorrect lvl {lvl} for {gpio_name}')
                self.print_state()
        else:
            print(f'No name {gpio_name} in gpios')

    def convert_speed_to_lvl_duration(self, speed: float | int) -> float | None:
        """
        UNSAFE for big speeds
        Take speed in r/min and gives lvl duration
        :param speed: float orint speed r/min
        :return: float lvl duration
        """
        if speed > self.MAX_SPEED_RPM:
            print(f'Speed {speed} more than maximum {self.MAX_SPEED_RPM}')
            # return None
        lvl_duration: float = 30 / (speed * self.full_rotate_steps)
        print(f'Speed {speed} -> duration {lvl_duration}')
        if lvl_duration < self.LVL_MIN_DURATION:
            print(f'Duration {lvl_duration}less than min 2.5e-6')
            # return None
        return lvl_duration

    def convert_lvl_duration_to_speed(self, lvl_duration: float | int) -> float | None:
        """
        Unsafe for too small lvl durations
        Takes float or int duration and gives speed r/min of this duration
        :param lvl_duration: lvl duration
        :return: speed float r/min
        """
        if lvl_duration < self.LVL_MIN_DURATION:
            print(f'Duration {lvl_duration} less than min {self.LVL_MIN_DURATION}')
            # return None
        speed: float = 30 / (lvl_duration * self.full_rotate_steps)
        print(f'Duration {lvl_duration} -> speed {speed}')
        if speed > self.MAX_SPEED_RPM:
            print(f'Speed {speed} more than maximum {self.MAX_SPEED_RPM}')
            # return None
        return speed

    def rotate_sectors(self, sector: float, speed: float | int | None = None) -> None:
        """
        UNSAFE for big speeds
        :param sector: degree of rotation ( - sign mean DIR change)
        :param speed: float speed of rotation r/min
        :return: None
        """
        if sector < 0 and (self.pi.read(gpio=self.dir_gpio) != pigpio.LOW):  # CCW DIR
            self.change_lvl(gpio_name='DIR', lvl=pigpio.LOW)  # Needs to change
        elif sector > 0 and (self.pi.read(gpio=self.dir_gpio) != pigpio.HIGH):  # CW DIR
            self.change_lvl(gpio_name='DIR', lvl=pigpio.HIGH)  # Needs to change
        if speed is not None:
            current_lvl_duration: float = self.convert_speed_to_lvl_duration(speed=speed)
            if current_lvl_duration < self.lv_min_duration:
                current_lvl_duration = self.lv_min_duration
                print(f'Lvl duration {self.lv_min_duration}')
        else:
            current_lvl_duration: float = self.lv_min_duration
        print(f'Speed {speed} Lvl duration: {current_lvl_duration}')
        print('Start moving ...')
        for _ in range(int(sector * self.sector_steps)):
            self.pi.write(gpio=self.pul_gpio, level=pigpio.HIGH)  # LVL HIGH (1)
            sleep(current_lvl_duration)
            self.pi.write(gpio=self.pul_gpio, level=pigpio.LOW)  # LVL LOW (0)
            sleep(current_lvl_duration)

    def rotate_speed(self, speed: float | int = 5, duration: float | int = 6) -> None | bool:
        """
        Makes speed rotates/min for t seconds
        Converts speed (rotations/min) into PWM freqency
        :param speed: rotates / min
        :param duration: time in seconds
        :return: None when don't move or True after Move
        """
        if speed > self.MAX_SPEED_RPM:
            print(f'Speed {speed} r/min more than max speed {self.MAX_SPEED_RPM}')
            return  # Don't move
        elif speed > self.max_speed:
            print(f'Speed {speed} r/min more than max speed {self.max_speed} for microstep {self.microstep}')
            return  # Don't move
        if speed < 0 and (self.pi.read(gpio=self.dir_gpio) != pigpio.LOW):  # CCW DIR
            self.change_lvl(gpio_name='DIR', lvl=pigpio.LOW)  # Needs to change
        elif speed > 0 and (self.pi.read(gpio=self.dir_gpio) != pigpio.HIGH):  # CW DIR
            self.change_lvl(gpio_name='DIR', lvl=pigpio.HIGH)  # Needs to change
        impulses_per_second: int = int(speed / 60 * self.full_rotate_steps)  # Calculate need amount of steps for second
        current_lvl_duration = self.convert_speed_to_lvl_duration(speed=speed)
        print(f'Speed {speed} Lvl duration: {current_lvl_duration}')
        if current_lvl_duration < self.lv_min_duration:
            current_lvl_duration = self.lv_min_duration
            print(f'Lvl duration {self.lv_min_duration}')
        self.pi.write(gpio=self.pul_gpio, level=pigpio.LOW)  # LVL LOW (0)
        for _ in range(int(impulses_per_second * duration)):
            self.pi.write(gpio=self.pul_gpio, level=pigpio.HIGH)  # LVL HIGH (1)
            sleep(current_lvl_duration)
            self.pi.write(gpio=self.pul_gpio, level=pigpio.LOW)  # LVL LOW (0)
            sleep(current_lvl_duration)
        else:
            return True

    def stop_driver(self) -> None:
        """
        Turn off driver (Turn off PULL PWM and TUEN OFF ENA of driver)
        :return: None
        """
        try:
            self.pi.write(gpio=self.pul_gpio, level=pigpio.LOW)
            print('PULL off (0)')
            self.pi.write(gpio=self.ena_gpio, level=pigpio.HIGH)
            print('ENA off (1)')
        except Exception as e:
            print('While disabling:')
            print(e)
        finally:
            self.pi.stop()
            print('GPIO stops')

    def print_mode(self) -> None:
        if self.ena_gpio is not None:
            print(f"ENA pin MODE : {self.pi.get_mode(gpio=self.ena_gpio)} (IN - 0 OUT - 1)")
        if self.dir_gpio is not None:
            print(f"DIR pin MODE : {self.pi.get_mode(gpio=self.dir_gpio)} (IN - 0 OUT - 1)")
        print(f"PUL pin MODE : {self.pi.get_mode(gpio=self.pul_gpio)} (IN - 0 OUT - 1)")
        if self.pend_gpio is not None:
            print(f"PEND pin read : {self.pi.read(gpio=self.pend_gpio)}")

    def print_state(self) -> None:
        if self.ena_gpio is not None:
            print(f"ENA gpio state : {self.pi.read(gpio=self.ena_gpio)}")
        if self.dir_gpio is not None:
            print(f"DIR gpio state : {self.pi.read(gpio=self.dir_gpio)}")
        print(f"PUL pin read : {self.pi.read(gpio=self.pul_gpio)}")
        if self.pend_gpio is not None:
            print(f"PEND gpio state : {self.pi.read(gpio=self.pend_gpio)}")
        print('#' * 40)


def setup_driver() -> DL57D:
    """
    Setup Driver from keyboard
    :return: Driver
    """
    details: str = input('Input driver details -> y ( n - for default)\n')
    try:
        if details.lower() == 'y' or details.lower() == 'yes':
            m: int = int(input('Input microsteps -> int\n'))
            g: str = input('Input gpios?     -> y ( n - for default)\n')
            if g.lower() == 'y' or g.lower() == 'yes':
                pull: int = int(input('Input PUL gpio\n'))
                ena: int = int(input('Input ENA gpio\n'))
                d: int = int(input('Input DIR gpio\n'))
                pend: int = int(input('Input PEND gpio\n'))
                driver: DL57D = DL57D(pul_gpio=pull,  # PIN 12
                                      ena_gpio=ena,  # PIN 33
                                      dir_gpio=d,  # PIN 16
                                      pend_gpio=pend,  # PIN 32
                                      microstep=m,
                                      sectors=360)
            else:
                driver: DL57D = DL57D(pul_gpio=18,  # PIN 12
                                      ena_gpio=13,  # PIN 33
                                      dir_gpio=23,  # PIN 16
                                      pend_gpio=12,  # PIN 32
                                      microstep=m,
                                      sectors=360)
        else:
            driver: DL57D = DL57D(pul_gpio=18,  # PIN 12
                                  ena_gpio=13,  # PIN 33
                                  dir_gpio=23,  # PIN 16
                                  pend_gpio=12,  # PIN 32
                                  microstep=10,
                                  sectors=360)
        return driver
    except Exception as e:
        print(e)
        sys.exit(BAD_CODE)


def run_driver(driver: DL57D) -> None:
    run: bool = True
    while run:
        try:
            driver.print_state()
            m: str = '0'  # At start rotate to 180 degree
            while m != 'exit' and m != 'e':
                driver.print_state()
                m: str = input('Chose mode:\n'
                               'd --> degree control\n'
                               's --> speed control\n'
                               'ena 1 --> ENA HIGH\n'
                               'ena 0 --> ENA LOW\n'
                               'ena c --> ENA Change LVL\n'
                               'dir 1 --> DIR HIGH\n'
                               'dir 0 -- DIR LOW\n'
                               'dir c --> DIR Change lvl\n'
                               'e for exit\n').lower()
                if m == 'd':
                    a: str = input('Chose angle (int) in degree:\n')
                    if a.replace(".", "").isnumeric():
                        d: float = float(a)
                        driver.rotate_sectors(sector=d)
                    else:
                        print(f'{a} is not int')
                elif m == 's':
                    s: str = input('Chose speed (float) in rotation/min:\n')
                    if s.replace(".", "").isnumeric():
                        s: float = float(s)
                    else:
                        print(f'{s} is not float')
                    t: str = input('Chose time (float) in seconds:\n')
                    if t.replace(".", "").isnumeric():
                        t: float = float(t)
                        driver.rotate_speed(speed=s, duration=t)
                    else:
                        print(f'{t} is not float')
                elif m == 'ena 1':
                    driver.change_lvl(gpio_name='ENA', lvl=pigpio.HIGH)
                elif m == 'ena 0':
                    driver.change_lvl(gpio_name='ENA', lvl=pigpio.LOW)
                elif m == 'ena c':
                    driver.change_lvl(gpio_name='ENA')
                elif m == 'dir 1':
                    driver.change_lvl(gpio_name='DIR', lvl=pigpio.HIGH)
                elif m == 'dir 0':
                    driver.change_lvl(gpio_name='DIR', lvl=pigpio.LOW)
                elif m == 'dir c':
                    driver.change_lvl(gpio_name='DIR')
                elif m == 'e':
                    driver.stop_driver()

        except Exception as e:
            print(e)
        finally:
            driver.stop_driver()
            run = False
    print('Driver closed')


def main() -> None:
    driver: DL57D = setup_driver()
    run_driver(driver=driver)


if __name__ == "__main__":
    main()
