# An example of 6TiSCH Minimal Scheduling Function (MSF)

You will have two firmwares:

* `msf-root`: for the root node (TSCH coordinator and RPL DODAG root)
* `msf-node`: for an ordinary node (RPL router)

This example is used in `tests/16-6tisch-msf`.

## Supported platforms

As of writing, we confirm this example runs on the following
platforms:

* `cooja-mote`
* `openmote`

## Quick start

Connect two OpenMote devices to your machine. Then, go down to
`examples/6tisch/msf`, make the devices wait for flashing (reset a
device connecting `ON/SLEEP` and `GND`), and follow steps shown
below. Here we assume one device has `/dev/ttyUSB0` and the other has
`/dev/ttyUSB1`:

In a terminal:
```
$ make TARGET=openmote PORT=/dev/ttyUSB0 msf-root.upload
(reset the device)
$ make TARGET=openmote PORT=/dev/ttyUSB0 login
#0012.4b00.060d.9edf> msf
MSF commands
'> msf help': show this message
'> msf cnst': show constants (configuration parameters)
'> msf stat': show current status
'> msf nego': show negotiated cells
'> msf auto': show autonomous cells
#0012.4b00.060d.9edf> msf auto
MSF Autonomous Cells:
type, slot, chan, addr
o  RX,   77,    4, 0012.4b00.060d.9edf
```

In another terminal:
```
$ make TARGET=openmote PORT=/dev/ttyUSB1 msf-node.upload
(reset the device)
$ make TARGET=openmote PORT=/dev/ttyUSB1 login
#0012.4b00.060d.9edf> msf
MSF commands
'> msf help': show this message
'> msf cnst': show constants (configuration parameters)
'> msf stat': show current status
'> msf nego': show negotiated cells
'> msf auto': show autonomous cells
```

If you can see the help message, this device is ready. Otherwise, you
might have failed to flash a firmware. The `msf-root` device should
have an autonomous RX cell just after booting up.

After the `msf-node` device gets synchronized and joins the network,
it starts sending application packets. Then, you should see that
negotiated TX cells are scheduled.

```
#0012.4b00.060d.9ee1> msf nego
MSF Negotiated Cells:
 type, sl_off, ch_off,  PDR, addr
o  TX,     35,      1, 100%, 0012.4b00.060d.9edf
o  TX,     50,     10, 100%, 0012.4b00.060d.9edf
```

If you hit `msf nego` on the terminal of `msf-root`, you will see
corresponding negotiated RX cells:
```
#0012.4b00.060d.9edf> msf nego
MSF Negotiated Cells:
 type, sl_off, ch_off,  PDR, addr
o  RX,     35,      1,  N/A, 0012.4b00.060d.9ee1
o  RX,     50,     10,  N/A, 0012.4b00.060d.9ee1
```

If log messages of MSF are annoying, you can disable it by `log`
command:

```
#0012.4b00.060d.9ee1> log msf 0
```
