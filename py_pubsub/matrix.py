from apa102_pi.driver import apa102
import time
#set_pixel
#seet_pixel_rgb
#get_pixel
#get_pixel_rgb
#rotate(positions=1)
#combine_color(red, green, blue)

class LEDMatrix:
    # Colors
    GREEN = 0x00FF00
    BLUE = 0x0000FF
    ORANGE = 0xFFA500
    RED = 0xFF0000
    WHITE = 0xFFFFFF

    num_leds = 64

    def __init__(self, logger, owningPub):
        self.logger = logger
        self.owningPub = owningPub
        self.strip = apa102.APA102(num_led=self.num_leds, order='rgb')
        self.max_brightness = 64
        self.blink_success_time = 0.1 # sec
        self.stopExecution = False
        self.blinkLoopActive = False
        self.dt = 0.01  # sec - for pseudosleep loop

        self.isTurnedOn = False
        self.turn_off()

    def turn_on(self, color, brightness):
        self.isTurnedOn = True

        for i in range(self.num_leds):
            self.strip.set_pixel_rgb(i, color, bright_percent=brightness)

        self.strip.show()

    def turn_off(self):
        self.clear()
        self.isTurnedOn = False

    def blink(self, color=-1, initial_on_time=2, max_brightness=-1, rate=8, num_blinks=3):
        if self.isTurnedOn:
            self.stop_execution()

        if color == -1:
            color = self.GREEN
        if max_brightness == -1:
            max_brightness = self.max_brightness

        delta_bright = max_brightness/2
        brightness_list = [x*delta_bright for x in range(int(max_brightness/delta_bright)+1)]

        'Wait for previous blink loop to terminate'
        while self.blinkLoopActive:
            time.sleep(self.dt)

        self.blinkLoopActive = True
        self.executeBlinkLoop(color, initial_on_time, num_blinks, max_brightness, brightness_list, rate)
        self.blinkLoopActive = False

    def executeBlinkLoop(self, color, initial_on_time, num_blinks, max_brightness, brightness_list, rate):
        self.stopExecution = False
        self.turn_on(color, max_brightness)

        'Initial TOn phase'
        completed = self.sleep(initial_on_time)

        if not completed:
            return

        'Blink phase'
        for i in range(num_blinks):
            for brightness in brightness_list[::-1]:

                self.turn_on(color, brightness)
                completed = self.sleep(1/rate)

                if not completed:
                    return

        self.turn_off() 

    def sleep(self, T: float):
        t = 0
        while t <= T:
            time.sleep(self.dt)
            t += self.dt

            if self.stopExecution:
                self.logger.info(f'Robot clicked - turning off prematurely')
                self.turn_off()
                return False

        return True

    def stop_execution(self):
        self.logger.info('--------------------------stopping matrix execution--------------------------------------')
        self.stopExecution = True


    def test(self):
        self.blink(self.RED)
        self.blink(self.BLUE)
        self.blink(self.GREEN)
        self.clear()

    def clear(self):
        self.strip.clear_strip()


if __name__ == '__main__':
    mat = LEDMatrix(None, None)
    mat.blink(LEDMatrix.GREEN)
    mat.clear()
