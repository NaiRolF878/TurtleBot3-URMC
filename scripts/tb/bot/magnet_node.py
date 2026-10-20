#!/usr/bin/env python3
"""Elektromagnet an GPIO 17 des Raspberry Pi schalten -- laeuft AUF DEM BOT.

Wird von bot-up.sh mit auf den Bot kopiert und gestartet (Log: ~/logs/magnet.log).

ROS-Schnittstelle:
    Service  /magnet          std_srvs/SetBool   data:true = an, false = aus
    Topic    /magnet/cmd      std_msgs/Bool      dasselbe als Topic (z.B. aus einer anderen Node)
    Topic    /magnet/state    std_msgs/Bool      aktueller Zustand (latched -- neue Abonnenten
                                                 bekommen sofort den letzten Wert)

    ros2 service call /magnet std_srvs/srv/SetBool "{data: true}"
    ros2 service call /magnet std_srvs/srv/SetBool "{data: false}"

Parameter:
    gpio_pin        17      BCM-Nummer (= physischer Pin 11 auf der Leiste)
    gpio_chip       0       /dev/gpiochip<N>; beim Pi 4 ist GPIO 0-53 auf gpiochip0
    active_high     True    False, falls das Modul bei LOW anzieht
    max_on_time_s   0.0     > 0: schaltet nach so vielen Sekunden selbst ab (Schutz gegen
                            Ueberhitzen, falls ein "aus" verloren geht). 0 = kein Limit.

Sicherheit: Beim Start und beim Beenden (Strg+C, SIGTERM, Absturz) wird der Pin auf AUS gesetzt.

HARDWARE: Der GPIO liefert nur 3,3 V und wenige mA -- er darf den Magneten NICHT direkt
treiben. Immer ueber ein Modul mit Transistor/Relais (z.B. Grove Electromagnet: Signal an
GPIO 17, Versorgung an 5 V, GND an GND).
"""
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class GpioOut:
    """Ein Ausgangspin -- nimmt die erste GPIO-Bibliothek, die auf dem Pi vorhanden ist."""

    def __init__(self, chip, pin, logger):
        self.pin = pin
        self.backend = None
        errors = []
        for name in ('gpiod', 'lgpio', 'RPi.GPIO'):
            try:
                getattr(self, '_open_' + name.replace('.', '_'))(chip, pin)
                self.backend = name
                logger.info(f'GPIO {pin} ueber {name} geoeffnet')
                return
            except Exception as e:                      # naechste Bibliothek probieren
                errors.append(f'{name}: {e}')
        raise RuntimeError('Keine GPIO-Bibliothek nutzbar -- ' + ' | '.join(errors) +
                           ' | Rechte? ls -l /dev/gpiochip* ; Nutzer in Gruppe gpio/dialout?')

    # --- libgpiod, Python-Bindings v2 (gpiod.request_lines) oder v1 (Chip.get_line) ---
    def _open_gpiod(self, chip, pin):
        import gpiod
        if hasattr(gpiod, 'request_lines'):
            from gpiod.line import Direction, Value
            self._v_on, self._v_off = Value.ACTIVE, Value.INACTIVE
            self._req = gpiod.request_lines(
                f'/dev/gpiochip{chip}', consumer='urmc_magnet',
                config={pin: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)})
            self._set = lambda v: self._req.set_value(pin, self._v_on if v else self._v_off)
            self._close = self._req.release
        else:
            c = gpiod.Chip(f'gpiochip{chip}')
            line = c.get_line(pin)
            line.request(consumer='urmc_magnet', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
            self._set = lambda v: line.set_value(1 if v else 0)
            self._close = line.release

    def _open_lgpio(self, chip, pin):
        import lgpio
        h = lgpio.gpiochip_open(chip)
        lgpio.gpio_claim_output(h, pin, 0)
        self._set = lambda v: lgpio.gpio_write(h, pin, 1 if v else 0)
        self._close = lambda: (lgpio.gpio_free(h, pin), lgpio.gpiochip_close(h))

    def _open_RPi_GPIO(self, chip, pin):
        import RPi.GPIO as GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        self._set = lambda v: GPIO.output(pin, GPIO.HIGH if v else GPIO.LOW)
        self._close = lambda: GPIO.cleanup(pin)

    def write(self, level):
        self._set(level)

    def close(self):
        try:
            self._close()
        except Exception:
            pass


class MagnetNode(Node):
    def __init__(self):
        super().__init__('magnet')
        self.declare_parameter('gpio_pin', 17)
        self.declare_parameter('gpio_chip', 0)
        self.declare_parameter('active_high', True)
        self.declare_parameter('max_on_time_s', 0.0)

        pin = self.get_parameter('gpio_pin').value
        self.active_high = self.get_parameter('active_high').value
        self.gpio = GpioOut(self.get_parameter('gpio_chip').value, pin, self.get_logger())
        self.on = False
        self._timer = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_state = self.create_publisher(Bool, '/magnet/state', latched)
        self.create_service(SetBool, '/magnet', self.cb_service)
        self.create_subscription(Bool, '/magnet/cmd', lambda m: self.set_magnet(m.data), 10)

        self.set_magnet(False)
        self.get_logger().info(f'Magnet bereit an GPIO {pin} ({self.gpio.backend}) -- '
                               'ros2 service call /magnet std_srvs/srv/SetBool "{data: true}"')

    def set_magnet(self, on):
        on = bool(on)
        self.gpio.write(on if self.active_high else not on)
        if on != self.on:
            self.get_logger().info('Magnet AN' if on else 'Magnet AUS')
        self.on = on
        self.pub_state.publish(Bool(data=on))

        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        limit = float(self.get_parameter('max_on_time_s').value)
        if on and limit > 0:
            self._timer = self.create_timer(limit, self._auto_off)

    def _auto_off(self):
        self.get_logger().warn('max_on_time_s erreicht -- Magnet automatisch AUS')
        self.set_magnet(False)

    def cb_service(self, req, res):
        try:
            self.set_magnet(req.data)
            res.success = True
            res.message = 'an' if req.data else 'aus'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def shutdown(self):
        # Immer AUS hinterlassen -- egal wie die Node endet.
        try:
            self.gpio.write(False if self.active_high else True)
        finally:
            self.gpio.close()


def main():
    rclpy.init()
    node = None
    # SIGTERM (pkill aus bot-down.sh) wie Strg+C behandeln, damit der Pin sauber auf AUS geht.
    def on_sigterm(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sigterm)
    try:
        node = MagnetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'magnet_node: {e}', file=sys.stderr)
        sys.exit(1)
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
