# WiringPi

WiringPi is a _performant_ GPIO access library written in C.


To compile programs with wiringPi, you need to include `wiringPi.h` as well as link against `wiringPi`:

```c
#include <wiringPi.h> // Include WiringPi library!

int main(void)
{
  // uses BCM numbering of the GPIOs and directly accesses the GPIO registers.
  wiringPiSetupGpio();

  // pin mode ..(INPUT, OUTPUT, PWM_OUTPUT)
  // set pin 17 to input
  pinMode(17, INPUT);

  // pull up/down mode (PUD_OFF, PUD_UP, PUD_DOWN) => down
  pullUpDnControl(17, PUD_DOWN);

  // get state of pin 17
  int value = digitalRead(17);

  if (HIGH == value)
  {
    // your code
  }
}
```

To compile this code, link against wiringPi:

```sh
gcc -o myapp myapp.c -l wiringPi
```

Be sure to check out the [examples](./examples/), build them using Make:

```sh
cd examples
make <example-name | really-all>
```

The tool `gpio` can be used to set single pins as well as get the state of everything at once:

```
pi@wiringdemo:~ $ gpio readall
 +-----+-----+---------+------+---+- M4Zero -+---+------+---------+-----+-----+
 | I/O | wPi |   Name  | Mode | V | Physical | V | Mode |  Name   | wPi | I/O |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 |     |     |    3.3V |      |   |  1 || 2  |   |      | 5V      |     |     |
 | 262 |   8 |   SDA.0 |  OUT | 1 |  3 || 4  |   |      | 5V      |     |     |
 | 261 |   9 |   SCL.0 |  OUT | 1 |  5 || 6  |   |      | 0V      |     |     |
 | 268 |   7 |  IO.268 |  OUT | 1 |  7 || 8  | 1 | OUT  | TxD1    | 15  | 269 |
 |     |     |      0V |      |   |  9 || 10 | 1 | OUT  | RxD1    | 16  | 270 |
 | 226 |   0 |  IO.226 |  OUT | 1 | 11 || 12 | 1 | OUT  | IO.257  | 1   | 257 |
 | 227 |   2 |  IO.227 |  OUT | 1 | 13 || 14 |   |      | 0V      |     |     |
 | 267 |   3 |  IO.267 |  OUT | 1 | 15 || 16 | 1 | OUT  | IO.271  | 4   | 271 |
 |     |     |    3.3V |      |   | 17 || 18 | 1 | OUT  | IO.272  | 5   | 272 |
 | 231 |  12 |    MOSI |  OUT | 1 | 19 || 20 |   |      | 0V      |     |     |
 | 232 |  13 |    MISO |  OUT | 1 | 21 || 22 | 1 | OUT  | IO.66   | 6   | 66  |
 | 230 |  14 |    SLCK |  OUT | 1 | 23 || 24 | 1 | OUT  | SS      | 10  | 229 |
 |     |     |      0V |      |   | 25 || 26 | 1 | OUT  | IO.233  | 11  | 233 |
 | 264 |  30 |   SDA.1 |  OUT | 1 | 27 || 28 | 1 | OUT  | SCL.1   | 31  | 263 |
 | 266 |  21 |  IO.266 |  OUT | 1 | 29 || 30 |   |      | 0V      |     |     |
 | 265 |  22 |  IO.265 |  OUT | 1 | 31 || 32 | 1 | OUT  | IO.228  | 26  | 228 |
 | 234 |  23 |  IO.234 |  OUT | 1 | 33 || 34 |   |      | 0V      |     |     |
 | 258 |  24 |  IO.258 |  OUT | 1 | 35 || 36 | 1 | OUT  | IO.71   | 27  | 71  |
 | 256 |  25 |  IO.256 |  OUT | 1 | 37 || 38 | 1 | OUT  | IO.260  | 28  | 260 |
 |     |     |      0V |      |   | 39 || 40 | 1 | OUT  | IO.259  | 29  | 259 |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 | I/O | wPi |   Name  | Mode | V | Physical | V | Mode |  Name   | wPi | I/O |
 +-----+-----+---------+------+---+- M4Zero -+---+------+---------+-----+-----+
```


## Installing

You can either build it yourself or use the prebuilt binaries:

### From Source

1. create debian-package

```sh
# fetch the source
sudo apt install gettext-base fakeroot git
git clone https://github.com/Dangku/WiringPi.git
cd WiringPi

# build the package
./build debian
mv debian-template/wiringpi_3.6_arm64.deb /tmp/

# install it
sudo apt install /tmp/wiringpi_3.6_arm64.deb
```


### Prebuilt Binaries

Grab the latest release from [here](https://github.com/Dangku/WiringPi/releases).


Unzip/use the portable prebuilt verison:

```sh
# unzip the archive
tar -xfv wiringpi_3.6.tar.gz
```

Install the debian package:

```sh
# install a dpkg
sudo apt install ./wiringpi_3.6_arm64.deb
```


## Support

Please use the [issue system](https://github.com/WiringPi/WiringPi/issues) of GitHub.


## History

This repository is the continuation of 'Gordon's wiringPi' which has been [deprecated](https://web.archive.org/web/20220405225008/http://wiringpi.com/wiringpi-deprecated/), a while ago.

* The last "old wiringPi" source of Gordon's release can be found at the
  [`final_source_2.50`](https://github.com/WiringPi/WiringPi/tree/final_official_2.50) tag.
* The default `master` branch contains code that has been written since version 2.5
  to provide support for newer hardware as well as new features.

:information_source:️ Since 2024, [GC2](https://github.com/GrazerComputerClub) has taken over maintenance of the project, supporting new OS versions as well as current hardware generations. We are dedicated to keeping the arguably best-performing GPIO Library for Raspberry Pi running smoothly. We strive to do our best, but please note that this is a community effort, and we cannot provide any guarantees or take responsibility for implementing specific features you may need.

## Debug

WIRINGPI_DEBUG=1 ./my_wiringpi_program

WIRINGPI_DEBUG=1 gpio readall
