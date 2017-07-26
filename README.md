Interrupt based bus sync with EasyCAT
-------------------------------------

This example uses wiringPi for gpio interrupt handling. Run `sudo apt-get install wiringpi`.

This example uses the bcm2835 library to comunicate with the SPI.
The reference web site is: http://www.airspayce.com/mikem/bcm2835/

Download the library: http://www.airspayce.com/mikem/bcm2835/bcm2835-1.50.tar.gz, then:

```
tar zxvf bcm2835-1.50.tar.gz
cd bcm2835-1.xx
./configure
make
sudo make check
sudo make install
cd ..
```

Now compile the project from inside the project directory:

```
git clone https://github.com/glowbuzzer/easycat.git
cd easycat
gcc -o ecat ecat.cpp EasyCAT.cpp -l bcm2835 -lwiringPi
```

You need to compile/deploy the custom ESI `EasyCAT_DC_modified.xml`.

To run the example:

```
sudo ./ecat
```

The example simply increments a 32-bit counter. An app in codesys reads this value and compares with its own 
internal counter to detect any jitter. On an unmodified Raspbian we see this is around 5ms dropped every 10000ms, 
or 0.05% lost.
