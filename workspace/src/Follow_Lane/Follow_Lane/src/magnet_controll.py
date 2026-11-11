#!/usr/bin/env python3
"""Elektromagnet an GPIO 17 des Raspberry Pi schalten -- laeuft AUF DEM BOT.

bot-up.sh kopiert die Datei nach ~/magnet_controll.py auf den Bot und startet sie dort
(Log: ~/logs/magnet.log), bot-down.sh beendet sie per SIGTERM.

Zwei Startarten, beide gleichwertig:
    ros2 run Follow_Lane magnet_controll        (Paket auf dem Pi gebaut)
    python3 magnet_controll.py                  (Datei per scp kopiert, kein Build noetig --
                                                 die Datei importiert nichts aus dem Paket)

ROS-Schnittstelle:
    Topic    /lokal/magnet_control  std_msgs/Bool     Befehl, true = an (sendet storage_sequence)
    Topic    /lokal/magnet_status   std_msgs/Bool     aktueller Zustand, latched: neue Abonnenten
                                                      bekommen sofort den letzten Wert
    Service  /lokal/magnet          std_srvs/SetBool  dasselbe von Hand, mit Rueckmeldung

    ros2 topic pub --once /lokal/magnet_control std_msgs/msg/Bool "{data: true}"
    ros2 service call /lokal/magnet std_srvs/srv/SetBool "{data: false}"

Parameter:
    gpio_pin      17     BCM-Nummer (= physischer Pin 11 auf der Leiste)
    gpio_chip     0      /dev/gpiochip<N>, nur fuer gpiod/lgpio; Pi 4: GPIO 0-53 auf gpiochip0
    active_high   True   False, falls das Modul bei LOW anzieht

Sicherheit: Beim Start und beim Beenden (Strg+C, SIGTERM von pkill, SIGHUP, Absturz) geht der
Pin auf AUS. Nur kill -9 kann das nicht abfangen.

HARDWARE: Der GPIO liefert nur 3,3 V und wenige mA, er darf den Magneten NICHT direkt treiben.
Immer ueber ein Modul mit Transistor/Relais (Signal an GPIO 17, Versorgung an 5 V, GND an GND).
"""
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

CONTROL_TOPIC = '/lokal/magnet_control'
STATUS_TOPIC = '/lokal/magnet_status'
SERVICE = '/lokal/magnet'


class GpioOut:
    """Ein Ausgangspin. Nimmt die erste GPIO-Bibliothek, die auf dem Pi funktioniert.
    gpiozero zuerst: damit ist der Magnet auf dem Bot getestet."""

    BACKENDS = ('gpiozero', 'gpiod', 'lgpio', 'RPi.GPIO')

    def __init__(self, chip, pin, active_high, logger):
        self.backend = None
        errors = []
        for name in self.BACKENDS:
            try:
                getattr(self, '_open_' + name.replace('.', '_'))(chip, pin, active_high)
                self.backend = name
                logger.info(f'GPIO {pin} ueber {name} geoeffnet')
                return
            except Exception as e:  # naechste Bibliothek probieren
                errors.append(f'{name}: {e}')
        raise RuntimeError('Keine GPIO-Bibliothek nutzbar -- ' + ' | '.join(errors) +
                           ' | Rechte? ls -l /dev/gpiochip* ; Nutzer in Gruppe gpio?')

    # Jede _open_* setzt self._set(on) (on = Magnet an, Polaritaet schon beruecksichtigt)
    # und self._close(). Der Pin startet AUS.

    def _open_gpiozero(self, chip, pin, active_high):
        from gpiozero import OutputDevice
        dev = OutputDevice(pin, active_high=active_high, initial_value=False)
        self._set = lambda on: dev.on() if on else dev.off()
        self._close = dev.close

    def _open_gpiod(self, chip, pin, active_high):
        import gpiod
        lvl = (lambda on: on == active_high)  # Magnet an -> physischer Pegel
        if hasattr(gpiod, 'request_lines'):   # Python-Bindings v2
            from gpiod.line import Direction, Value
            val = (lambda on: Value.ACTIVE if lvl(on) else Value.INACTIVE)
            req = gpiod.request_lines(
                f'/dev/gpiochip{chip}', consumer='urmc_magnet',
                config={pin: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=val(False))})
            self._set = lambda on: req.set_value(pin, val(on))
            self._close = req.release
        else:                                 # Python-Bindings v1
            line = gpiod.Chip(f'gpiochip{chip}').get_line(pin)
            line.request(consumer='urmc_magnet', type=gpiod.LINE_REQ_DIR_OUT,
                         default_vals=[int(lvl(False))])
            self._set = lambda on: line.set_value(int(lvl(on)))
            self._close = line.release

    def _open_lgpio(self, chip, pin, active_high):
        import lgpio
        lvl = (lambda on: int(on == active_high))
        h = lgpio.gpiochip_open(chip)
        lgpio.gpio_claim_output(h, pin, lvl(False))
        self._set = lambda on: lgpio.gpio_write(h, pin, lvl(on))
        self._close = lambda: (lgpio.gpio_free(h, pin), lgpio.gpiochip_close(h))

    def _open_RPi_GPIO(self, chip, pin, active_high):
        import RPi.GPIO as GPIO
        lvl = (lambda on: GPIO.HIGH if on == active_high else GPIO.LOW)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT, initial=lvl(False))
        self._set = lambda on: GPIO.output(pin, lvl(on))
        self._close = lambda: GPIO.cleanup(pin)

    def write(self, on):
        self._set(bool(on))

    def close(self):
        try:
            self._close()
        except Exception:
            pass


class MagnetControll(Node):
    def __init__(self):
        super().__init__('magnet_controll')
        self.declare_parameter('gpio_pin', 17)
        self.declare_parameter('gpio_chip', 0)
        self.declare_parameter('active_high', True)

        pin = self.get_parameter('gpio_pin').value
        self.gpio = GpioOut(self.get_parameter('gpio_chip').value, pin,
                            self.get_parameter('active_high').value, self.get_logger())
        self.on = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(Bool, STATUS_TOPIC, latched)
        self.create_subscription(Bool, CONTROL_TOPIC, lambda m: self.set_magnet(m.data), 10)
        self.create_service(SetBool, SERVICE, self.cb_service)

        self.set_magnet(False)
        self.get_logger().info(f'Magnet bereit an GPIO {pin} ({self.gpio.backend}), AUS. '
                               f'Hoert auf {CONTROL_TOPIC}, meldet auf {STATUS_TOPIC}')

    def set_magnet(self, on):
        on = bool(on)
        self.gpio.write(on)
        if on != self.on:
            self.get_logger().info('Magnet AN' if on else 'Magnet AUS')
        self.on = on
        self.pub_status.publish(Bool(data=on))

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
        """Immer AUS hinterlassen, egal wie die Node endet."""
        try:
            self.gpio.write(False)
        finally:
            self.gpio.close()


def main(args=None):
    # SIGTERM (pkill) und SIGHUP (SSH-Verbindung weg) wie Strg+C behandeln. Ohne das stirbt
    # der Prozess sofort, der finally-Block laeuft nicht und der Magnet bleibt an.
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGHUP, stop)

    rclpy.init(args=args)
    node = None
    try:
        node = MagnetControll()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            print('magnet_controll: Magnet AUS, GPIO freigegeben', flush=True)  # Context ist schon weg
            node.destroy_node()
        # rclpy raeumt den Context bei Strg+C schon selbst ab, sonst wirft der zweite Aufruf
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
