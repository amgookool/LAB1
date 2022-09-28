# _I2C Example_

* This example will show you how to use I2C module:
 
    * to read external i2c sensor, ADS1115..

## Pin assignment

* master:
    * GPIO0 is assigned as the data signal of i2c master port
    * GPIO2 is assigned as the clock signal of i2c master port

* Connection:
    * connect sda/scl of sensor with GPIO0/GPIO2
    * no need to add external pull-up resistors, driver will enable internal pull-up resistors.


