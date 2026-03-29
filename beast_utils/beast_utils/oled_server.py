#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from beast_msgs.srv import UpdateOLED
import smbus2
import time
from PIL import Image, ImageDraw, ImageFont


class OLEDServer(Node):
    def __init__(self):
        super().__init__('oled_server')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x3C)
        self.declare_parameter('width', 128)
        self.declare_parameter('height', 32)

        self.i2c_bus     = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.addr        = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.width       = self.get_parameter('width').get_parameter_value().integer_value
        self.height      = self.get_parameter('height').get_parameter_value().integer_value
        self.line_height = self.height // 3

        self.bus  = smbus2.SMBus(self.i2c_bus)
        self.font = ImageFont.load_default()
        self.lines = ['  UGV Beast', '  Starting...', '']

        self._init_display()
        self.get_logger().info(
            f'OLED initialized: {self.width}x{self.height} '
            f'on I2C bus {self.i2c_bus}, addr 0x{self.addr:02X}')

        self.srv = self.create_service(UpdateOLED, 'update_oled', self.handle_update)
        self.get_logger().info('update_oled service ready')

        # Delay first render 3 seconds to let bus settle
        self.render_timer = self.create_timer(3.0, self.render_tick)

    def _write(self, data, retries=50):
        for attempt in range(retries):
            try:
                msg = smbus2.i2c_msg.write(self.addr, data)
                self.bus.i2c_rdwr(msg)
                return True
            except Exception:
                time.sleep(0.02)
                # Every 10 failed attempts, reopen the bus handle
                if attempt % 10 == 9:
                    try:
                        self.bus.close()
                        time.sleep(0.05)
                        self.bus = smbus2.SMBus(self.i2c_bus)
                    except Exception:
                        pass
        self.get_logger().warn(f'I2C write failed: 0x{data[0]:02X}')
        return False

    def _cmd(self, c):
        return self._write([0x00, c])

    def _data(self, pixels):
        return self._write([0x40] + pixels)

    def _init_display(self):
        cmds = [
            0xAE,       # display off
            0x20, 0x00, # horizontal addressing
            0x40,       # start line 0
            0xA1,       # segment remap
            0xA8, 0x1F, # mux ratio for 32px
            0xC8,       # com scan direction
            0xD3, 0x00, # display offset
            0xDA, 0x02, # com pins
            0xD5, 0x80, # clock
            0xD9, 0xF1, # precharge
            0xDB, 0x40, # vcomh
            0x81, 0xCF, # contrast
            0xA4,       # pixels from ram
            0xA6,       # normal display
            0x8D, 0x14, # charge pump on
            0xAF,       # display on
        ]
        for c in cmds:
            self._cmd(c)

    def _render(self):
        image = Image.new('1', (self.width, self.height), 0)
        draw  = ImageDraw.Draw(image)
        for i, text in enumerate(self.lines):
            draw.text((0, i * self.line_height), text, font=self.font, fill=1)

        pixels = list(image.getdata())
        for page in range(4):
            if not self._cmd(0xB0 + page): return False
            if not self._cmd(0x00):        return False
            if not self._cmd(0x10):        return False
            row = []
            for col in range(self.width):
                byte = 0
                for bit in range(8):
                    y = page * 8 + bit
                    if y < self.height:
                        if pixels[y * self.width + col]:
                            byte |= (1 << bit)
                row.append(byte)
            if not self._data(row): return False
        return True

    def render_tick(self):
        success = self._render()
        if success:
            self.get_logger().info('Render OK')
        else:
            self.get_logger().warn('Render failed, re-initializing display')
            self._init_display()
            success2 = self._render()
            if success2:
                self.get_logger().info('Render OK after re-init')
            else:
                self.get_logger().warn('Render still failing after re-init')

    def handle_update(self, request, response):
        try:
            line = int(request.line_num)
            if not (0 <= line <= 3):
                response.success = False
                response.message = f'Invalid line {line}, must be 0-2'
                return response
            self.lines[line] = request.text[:21]
            response.success = True
            response.message = 'OK'
        except Exception as e:
            self.get_logger().error(f'OLED update failed: {e}')
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OLEDServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()