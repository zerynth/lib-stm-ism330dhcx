"""
.. module:: ism330dhcx

*****************
ISM330DHCX Module
*****************

This module contains the driver for STMicroelectronics ISM330DHCX 3-axis accelerometer and gyroscope.

In the ISM330DHCX, the sensing elements of the accelerometer and of the gyroscope are implemented on the same silicon die, thus guaranteeing superior stability and robustness. (`datasheet <https://www.st.com/resource/en/datasheet/ism330dhcx.pdf>`_).
    """

import struct
import spi

ISM330DHCX_ID                      = 0x6B
ISM330DHCX_WHO_AM_I                = 0x0F

ISM330DHCX_COUNTER_BDR_REG1        = 0x0B

ISM330DHCX_INT1_CTRL               = 0x0D
ISM330DHCX_INT2_CTRL               = 0x0E

ISM330DHCX_CTRL1_XL                = 0X10
ISM330DHCX_CTRL2_G                 = 0x11
ISM330DHCX_CTRL3_C                 = 0x12

ISM330DHCX_OUT_TEMP_L              = 0x20
ISM330DHCX_OUT_TEMP_H              = 0x21
ISM330DHCX_REG_DATA_OUT_X_L_G      = 0x22
ISM330DHCX_REG_DATA_OUT_X_H_G      = 0x23
ISM330DHCX_REG_DATA_OUT_Y_L_G      = 0x24
ISM330DHCX_REG_DATA_OUT_Y_H_G      = 0x25
ISM330DHCX_REG_DATA_OUT_Z_L_G      = 0x26
ISM330DHCX_REG_DATA_OUT_Z_H_G      = 0x27
ISM330DHCX_REG_DATA_OUT_X_L_A      = 0x28
ISM330DHCX_REG_DATA_OUT_X_H_A      = 0x29
ISM330DHCX_REG_DATA_OUT_Y_L_A      = 0x2A
ISM330DHCX_REG_DATA_OUT_Y_H_A      = 0x2B
ISM330DHCX_REG_DATA_OUT_Z_L_A      = 0x2C
ISM330DHCX_REG_DATA_OUT_Z_H_A      = 0x2D

TIMEOUT_DURATION                   = 1000 

ISM330DHCX_CTRLx_ODR_OFF           = 0

ISM330DHCX_ACC_SENS_FS_2G          = 0.061 # Sensitivity value for 2g full scale [mg/LSB] 
ISM330DHCX_ACC_SENS_FS_4G          = 0.122
ISM330DHCX_ACC_SENS_FS_8G          = 0.244
ISM330DHCX_ACC_SENS_FS_16G         = 0.488

ISM330DHCX_GYRO_SENS_FS_125DPS     = 4.375 # Sensitivity value for 125dps full scale [mdps/LSB] 
ISM330DHCX_GYRO_SENS_FS_250DPS     = 8.750
ISM330DHCX_GYRO_SENS_FS_500DPS     = 17.500
ISM330DHCX_GYRO_SENS_FS_1000DPS    = 35.000
ISM330DHCX_GYRO_SENS_FS_2000DPS    = 70.000
ISM330DHCX_GYRO_SENS_FS_4000DPS    = 140.000

ISM330DHCX_ACC_FS_REG_VALUE = [
    0,4,8,12
]

ISM330DHCX_ACC_SENS_VALUE = [
    ISM330DHCX_ACC_SENS_FS_2G,
    ISM330DHCX_ACC_SENS_FS_16G,
    ISM330DHCX_ACC_SENS_FS_4G,
    ISM330DHCX_ACC_SENS_FS_8G
]

ISM330DHCX_GYRO_FS_REG_VALUE = [
    2,0,4,8,12,1
]

ISM330DHCX_GYRO_SENS_VALUE = [
    ISM330DHCX_GYRO_SENS_FS_125DPS,
    ISM330DHCX_GYRO_SENS_FS_250DPS,
    ISM330DHCX_GYRO_SENS_FS_500DPS,
    ISM330DHCX_GYRO_SENS_FS_1000DPS,
    ISM330DHCX_GYRO_SENS_FS_2000DPS,
    ISM330DHCX_GYRO_SENS_FS_4000DPS
]

ISM330DHCX_ODR_OFF    = 0
ISM330DHCX_ODR_12Hz5  = 1
ISM330DHCX_ODR_26Hz   = 2
ISM330DHCX_ODR_52Hz   = 3
ISM330DHCX_ODR_104Hz  = 4
ISM330DHCX_ODR_208Hz  = 5
ISM330DHCX_ODR_417Hz  = 6
ISM330DHCX_ODR_833Hz  = 7
ISM330DHCX_ODR_1667Hz = 8
ISM330DHCX_ODR_3333Hz = 9
ISM330DHCX_ODR_6667Hz = 10
ISM330DHCX_ODR_6Hz5   = 11

_ODR_MASK = 0b00001111
_FS_MASK  = 0b11110000

@c_native("_ism330dhcx_getfast", ["csrc/ism330dhcx.c"])
def _ism330dhcx_getfast(spi, sens_acc, sens_gyro):
    pass

@c_native("_ism330dhcx_read_reg8",["csrc/ism330dhcx.c"])
def _ism330dhcx_read_reg8(spi,reg):
    pass

@c_native("_ism330dhcx_read_reg16",["csrc/ism330dhcx.c"])
def _ism330dhcx_read_reg16(spi,reg):
    pass

@c_native("_ism330dhcx_read_reg16x3",["csrc/ism330dhcx.c"])
def _ism330dhcx_read_reg16x3(spi,reg):
    pass

@c_native("_ism330dhcx_write_reg8",["csrc/ism330dhcx.c"])
def _ism330dhcx_write_reg8(spi,reg,value):
    pass

@c_native("_ism330dhcx_write_reg16",["csrc/ism330dhcx.c"])
def _ism330dhcx_write_reg16(spi,reg,value):
    pass

class ISM330DHCX(spi.Spi):
    """

.. class:: ISM330DHCX(spidrv, pin_cs, clk=5000000)

    Class which provides a simple interface to ISM330DHCX features.
    
    Creates an instance of ISM330DHCX class, using the specified SPI settings
    and initial device configuration

    :param spidrv: the *SPI* driver to use (SPI0, ...)
    :param pin_cs: Chip select pin to access the ISM330DHCX chip
    :param clk: Clock speed, default 5 MHz

    Example: ::

        from stm.ism330dhcx import ism330dhcx

        ...

        accgyro = ism330dhcx.ISM330DHCX(SPI0, D10)
        acc = accgyro.get_acc_data()
        gyro = accgyro.get_gyro_data()

    """

    def __init__(self, spidrv, pin_cs, clk=5000000):
        spi.Spi.__init__(self,pin_cs,spidrv,clock=clk)
        # for native functions
        self.spi       = spidrv & 0xFF
        self.odr_acc   = ISM330DHCX_ODR_417Hz
        self.odr_gyro  = ISM330DHCX_ODR_417Hz
        self.fs_acc    = 0
        self.fs_gyro   = 0
        self.sens_acc  = ISM330DHCX_ACC_SENS_FS_2G
        self.sens_gyro = ISM330DHCX_GYRO_SENS_FS_250DPS
        
        self.reset()
        sleep(100)

        if ISM330DHCX_ID != self.whoami():
            raise RuntimeError #("ISM330DHCX not detected")

        self.enable_acc(self.odr_acc, self.fs_acc)
        self.enable_gyro(self.odr_gyro, self.fs_gyro)

    def _register_word(self, register, value=None):
        self.lock()
        self.select()
        ex = None
        ret = None
        try:
            if value is None:
                ret = _ism330dhcx_read_reg16(self.spi, register)
            else:
                ret = _ism330dhcx_write_reg16(self.spi, register, value)
        except Exception as e:
            ex = e
        self.unselect()
        self.unlock()
        if ex is not None:
            raise ex
        return ret

    def _register_char(self, register, value=None):
        self.lock()
        self.select()
        ex = None
        ret = None
        try:
            if value is None:
                ret = _ism330dhcx_read_reg8(self.spi, register)
            else:
                ret = _ism330dhcx_write_reg8(self.spi, register, value)
        except Exception as e:
            ex = e
        self.unselect()
        self.unlock()
        if ex is not None:
            raise ex
        return ret

    def _fs(self, reg, value):
        char = self._register_char(reg)
        char &= _FS_MASK # clear FS bits
        char |= value
        self._register_char(reg, char)

    def _odr(self, reg, value):
        char = self._register_char(reg)
        char &= _ODR_MASK # clear ODR bits
        char |= value<<4
        self._register_char(reg, char)
    
    def reset(self):
        """
    .. method:: reset()

        Reset the device using the internal register flag.

        """
        ctr3 = self._register_char(ISM330DHCX_CTRL3_C)
        ctr3 |= 0x01
        self._register_char(ISM330DHCX_CTRL3_C, ctr3)

    def enable_acc(self, odr=ISM330DHCX_ODR_417Hz, fs=0):
        """

.. method:: enable_acc(odr=ISM330DHCX_ODR_417Hz, fs=0)

        Sets the device's configuration registers for accelerometer.
    
        **Parameters:**
    
        * **odr** : sets the Output Data Rate of the device. Available values are:
    
            ====== ================= =======================
            Value  Output Data Rate  Constant Name
            ====== ================= =======================
            0x00   OFF               ISM330DHCX_ODR_OFF
            0x01   12.5 Hz           ISM330DHCX_ODR_12Hz5
            0x02   26 Hz             ISM330DHCX_ODR_26Hz
            0x03   52 Hz             ISM330DHCX_ODR_52Hz
            0x04   104 Hz            ISM330DHCX_ODR_104Hz
            0x05   208 Hz            ISM330DHCX_ODR_208Hz
            0x06   417 Hz            ISM330DHCX_ODR_417Hz
            0x07   833 Hz            ISM330DHCX_ODR_833Hz
            0x08   1.667 KHz         ISM330DHCX_ODR_1667Hz
            0x09   3.333 KHz         ISM330DHCX_ODR_3333Hz
            0x0A   6.667 KHz         ISM330DHCX_ODR_6667Hz
            0x0B   6.5 Hz            ISM330DHCX_ODR_6Hz5
            ====== ================= =======================
        
        * **fs** : sets the Device Full Scale. Available values are:
    
            ====== =========== ========================== =============
            Value  Full Scale  Costant Name               in mg/LSB
            ====== =========== ========================== =============
            0x00   ±2g         ISM330DHCX_ACC_SENS_FS_2G  0.061 mg/LSB
            0x01   ±4g         ISM330DHCX_ACC_SENS_FS_4G  0.122 mg/LSB
            0x02   ±8g         ISM330DHCX_ACC_SENS_FS_8G  0.244 mg/LSB
            0x03   ±16g        ISM330DHCX_ACC_SENS_FS_16G 0.488 mg/LSB
            ====== =========== ========================== =============
    
        Returns True if configuration is successful, False otherwise.

        """
        if fs not in range(0,4):
            raise ValueError
        if odr not in range(0,12):
            raise ValueError
        try:
            self._fs(ISM330DHCX_CTRL1_XL, ISM330DHCX_ACC_FS_REG_VALUE[fs])
            self._odr(ISM330DHCX_CTRL1_XL, odr)
        except Exception as e:
            return False
        self.odr_acc = odr
        self.fs_acc = fs
        self.sens_acc = ISM330DHCX_ACC_SENS_VALUE[fs]
        return True

    def disable_acc(self):
        """

.. method:: disable_acc()

        Disables the accelerator sensor.

        Returns True if configuration is successful, False otherwise.

        """
        if self.odr_acc == 0:
            return True
        try:
            res = self._odr(ISM330DHCX_CTRL1_XL, ISM330DHCX_ODR_OFF)
        except Exception as e:
            return False
        self.odr_acc = ISM330DHCX_ODR_OFF
        return True

    def enable_gyro(self, odr=ISM330DHCX_ODR_417Hz, fs=0):
        """

.. method:: enable_gyro(odr=ISM330DHCX_ODR_417Hz, fs=0)

        Sets the device's configuration registers for gyroscope.
    
        **Parameters:**
    
        * **odr** : sets the Output Data Rate of the device. Available values are:
    
            ====== ================= =======================
            Value  Output Data Rate  Constant Name
            ====== ================= =======================
            0x00   OFF               ISM330DHCX_ODR_OFF
            0x01   12.5 Hz           ISM330DHCX_ODR_12Hz5
            0x02   26 Hz             ISM330DHCX_ODR_26Hz
            0x03   52 Hz             ISM330DHCX_ODR_52Hz
            0x04   104 Hz            ISM330DHCX_ODR_104Hz
            0x05   208 Hz            ISM330DHCX_ODR_208Hz
            0x06   417 Hz            ISM330DHCX_ODR_417Hz
            0x07   833 Hz            ISM330DHCX_ODR_833Hz
            0x08   1.667 KHz         ISM330DHCX_ODR_1667Hz
            0x09   3.333 KHz         ISM330DHCX_ODR_3333Hz
            0x0A   6.667 KHz         ISM330DHCX_ODR_6667Hz
            ====== ================= =======================
        
        * **fs** : sets the Device Full Scale. Available values are:

            ====== =========== ================================ =================
            Value  Full Scale  Costant Name                     in mdps/LSB
            ====== =========== ================================ =================
            0x00   ±125 dps    ISM330DHCX_GYRO_SENS_FS_125DPS   4.375 mdps/LSB
            0x01   ±250 dps    ISM330DHCX_GYRO_SENS_FS_250DPS   8.750 mdps/LSB
            0x02   ±500 dps    ISM330DHCX_GYRO_SENS_FS_500DPS   17.500 mdps/LSB
            0x03   ±1000 dps   ISM330DHCX_GYRO_SENS_FS_1000DPS  35.000 mdps/LSB
            0x04   ±2000 dps   ISM330DHCX_GYRO_SENS_FS_2000DPS  70.000 mdps/LSB
            0x05   ±4000 dps   ISM330DHCX_GYRO_SENS_FS_4000DPS  140.000 mdps/LSB
            ====== =========== ================================ =================
    
        Returns True if configuration is successful, False otherwise.

        """
        if fs not in range(0,6):
            raise ValueError
        if odr not in range(0,11):
            raise ValueError
        try:
            self._fs(ISM330DHCX_CTRL2_G, ISM330DHCX_GYRO_FS_REG_VALUE[fs])
            self._odr(ISM330DHCX_CTRL2_G, odr)
        except Exception as e:
            return False
        self.odr_gyro = odr
        self.fs_gyro = fs
        self.sens_gyro = ISM330DHCX_GYRO_SENS_VALUE[fs]
        return True

    def disable_gyro(self):
        """

.. method:: disable_gyro()

        Disables the gyroscope sensor.

        Returns True if configuration is successful, False otherwise.

        """
        if self.odr_gyro == 0:
            return True
        try:
            self._odr(ISM330DHCX_CTRL2_G, ISM330DHCX_ODR_OFF)
        except Exception as e:
            return False
        self.odr_gyro = ISM330DHCX_ODR_OFF
        return True

    def whoami(self):
        """

.. method:: whoami()

        Value of the *ISM330DHCX_WHO_AM_I* register (0x6B).
        
        """
        return self._register_char(ISM330DHCX_WHO_AM_I)

    def get_acc_data(self, ms2=True, raw=False):
        """

.. method:: get_acc_data(ms2=True, raw=False)

        Retrieves accelerometer data in one call.

        :param ms2: If ms2 flag is True, returns data converted in m/s^2; otherwise in mg (default True)
        :param raw: If raw flag is True, returns raw register values (default False)

        Returns acc_x, acc_y, acc_z

        """

        s = self.sens_acc
        # x = self._register_word(ISM330DHCX_REG_DATA_OUT_X_L_A) * s
        # y = self._register_word(ISM330DHCX_REG_DATA_OUT_y_L_A) * s
        # z = self._register_word(ISM330DHCX_REG_DATA_OUT_Z_L_A) * s
        # return (x,y,z)
        self.lock()
        self.select()

        ex = None
        ret = None
        try:
            ret = _ism330dhcx_read_reg16x3(self.spi, ISM330DHCX_REG_DATA_OUT_X_L_A)
            # (x,y,z) = _ism330dhcx_read_reg16x3(self.spi, ISM330DHCX_REG_DATA_OUT_X_L_A)
        except Exception as e:
            ex = e

        self.unselect()
        self.unlock()

        if ex is not None:
            raise ex
        if raw:
            return ret

        ret = (ret[0] * s, ret[1] * s, ret[2] * s)
        # ret = (x * s, y * s, z * s)

        if ms2 == False:
            return ret
        ret = (ret[0] * 0.00981, ret[1] * 0.00981, ret[2] * 0.00981)
        
        return ret


    def get_gyro_data(self, dps=True, raw=False):
        """

.. method:: get_gyro_data(dps=True, raw=False)

        Retrieves gyroscope data in one call.

        :param dps: If dps flag is True, returns data converted in dps; otherwise in mdps (default True)
        :param raw: If raw flag is True, returns raw register values (default False)

        Returns gyro_x, gyro_y, gyro_z

        """
        s = self.sens_gyro

        # x = self._register_word(ISM330DHCX_REG_DATA_OUT_X_L_G) * s
        # y = self._register_word(ISM330DHCX_REG_DATA_OUT_Y_L_G) * s
        # z = self._register_word(ISM330DHCX_REG_DATA_OUT_Z_L_G) * s
        # return (x,y,z)

        self.lock()
        self.select()
        ex = None
        ret = None
        try:
            ret = _ism330dhcx_read_reg16x3(self.spi, ISM330DHCX_REG_DATA_OUT_X_L_G)
            # (x,y,z) = _ism330dhcx_read_reg16x3(self.spi, ISM330DHCX_REG_DATA_OUT_X_L_G)
        except Exception as e:
            ex = e
        self.unselect()
        self.unlock()
        if ex is not None:
            raise ex
        if raw:
            return ret
        ret = (ret[0] * s, ret[1] * s, ret[2] * s)
        # ret = (x * s, y * s, z * s)
        if dps == False:
            return ret
        ret = (ret[0] / 1000.0, ret[1] / 1000.0, ret[2] / 1000.0)
        return ret

    def get_temp_data(self, raw=False):
        """

.. method:: get_temp()

        Retrieves temperature in one call; if raw flag is enabled, returns raw register values.

        Returns temp

        """
        traw = self._register_word(ISM330DHCX_OUT_TEMP_L)
        if raw:
            return traw
        temp = (traw / 256.0) + 25.0
        return temp

    def get_fast(self):
        """

.. method:: get_fast()

        Retrieves all data sensors in one call in fast way (c-code acquisition).
        
        * Temperature in °C
        * Acceleration in m/s^2
        * Angular Velocity in dps (degrees per second)

        Returns temp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z

        """
        return _ism330dhcx_getfast(self.spi, self.sens_acc, self.sens_gyro)

    def set_event_interrupt(self, pin_int, sensor, enable):
        """

.. method:: set_event_interrupt(pin_int, sensor, enable)

        Enables the interrupt pins. When data from sensor selected will be ready, the related interrupt pin configured will be set to high.

        :param pin_int: ID of the interrupt pin to be enabled/disabled. Available values are 1 o 2.
        :param sensor: Data Ready flag selector. Available values are "acc" or "gyro".
        :param enable: If enable flag is True, interrupt pin will be enabled, otherwise will be disabled.

        Returns True if configuration is successful, False otherwise.

        """
        if pin_int not in [1,2]:
            raise ValueError
        if sensor not in ["acc","gyro"]:
            raise ValueError
        if enable not in [True, False]:
            raise ValueError

        if sensor == "acc":
            value = 0b00000001
        elif sensor == "gyro":
            value = 0b00000010

        if pin_int == 1:
            reg = ISM330DHCX_INT1_CTRL
        elif pin_int == 2:
            reg = ISM330DHCX_INT1_CTRL

        try:
            char = self._register_char(reg)
            if enable == False:
                char &= ~value
            else:
                char |= value
            self._register_char(reg, char)
        except Exception as e:
            return False
        return True
