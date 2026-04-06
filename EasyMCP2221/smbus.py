from struct import pack, unpack
import EasyMCP2221
import random

I2C_SMBUS_BLOCK_MAX = 255  # len is one byte only

CRC_POLYNOMIAL=0x07
CRC_WIDTH=8
CRC_TABLE_SIZE=(1 << CRC_WIDTH)
CRC_MSB_MASK=(1 << (CRC_WIDTH - 1))
CRC_MAX_VALUE=(CRC_TABLE_SIZE - 1)


crc_lookup_table = [
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,   0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,   0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,   0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,   0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,

    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,   0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,   0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,   0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,   0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,

    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,   0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,   0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,   0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,   0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,

    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,   0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,   0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,   0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,   0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
]


def crc_calc_smbus (i2c_addr, value, register, size, data_type, endianness="little", write_cmd=False):
    crc = 0x00
    write_sa = (i2c_addr << 1) | 0
    read_sa = (i2c_addr << 1) | 1

    pec_data = [write_sa, register] if write_cmd else [write_sa, register, read_sa]
    endian = "big" if endianness=="big_endian" else "little"

    value_bytes = value.to_bytes(size, byteorder=endian, signed=(data_type=="int"))
    pec_data.extend(value_bytes)

    for byte in pec_data:
        index = (crc ^ byte) & CRC_MAX_VALUE
        crc = crc_lookup_table[index]

    return crc

class SMBus(object):

    """ Initialize and open an I2C bus connection. See :class:`EasyMCP2221.Device` initialization for details.

    Parameters:
        bus (int, optional): Device index **starting by 1**. Default is to use the first device (index 1).
        force (bool, optional): For compatibility only, not used.
        VID (int, optional): Vendor Id (default is ``0x04D8``)
        PID (int, optional): Product Id (default is ``0x00DD``)
        usbserial (str, optional): Device's USB serial ID to open.
        clock (int, optional): I2C clock frequency (default to ``100kHz``).
        mcp (EasyMCP2221 object, optional): An already initialized :class:`EasyMCP2221.Device` object to use.

    Return:
        SMBus object.

    Example:

        .. code-block:: python

            from EasyMCP2221 import SMBus

            bus = SMBus()

        or

        .. code-block:: python

            from EasyMCP2221 import smbus

            bus = smbus.SMBus()

    """

    def __init__(self, bus=1, force=False, VID=0x04D8, PID=0x00DD, usbserial=None, clock=100_000, mcp=None):

        if mcp:
            self.mcp = mcp

        else:
            bus = bus - 1  # first device should be 0
            self.mcp = EasyMCP2221.Device(VID, PID, devnum=bus, usbserial=usbserial)
            self.mcp.I2C_speed(clock)


    def _read_register(self, addr, register, length = 1, reg_bytes = 1, reg_byteorder = 'big'):
        """ Generic read from a register. See description in I2C_Slave class. """
        self.mcp.I2C_write(
            addr,
            register.to_bytes(reg_bytes, byteorder = reg_byteorder),
            kind = 'nonstop')

        data = self.mcp.I2C_read(
            addr,
            length,
            kind = 'restart')

        return data


    def _read(self, addr, length = 1):
        """ Generic read from an I2C slave. See description in I2C_Slave class. """
        return self.mcp.I2C_read(addr, length)


    def _write_register(self, addr, register, data, reg_bytes = 1, reg_byteorder = 'big'):
        """ Generic write to a specific register, position or command. See description in I2C_Slave class. """

        if type(data) == int:
            data = bytes([data])
        elif type(data) == list:
            data = bytes(data)

        self.mcp.I2C_write(
            addr,
            register.to_bytes(reg_bytes, byteorder=reg_byteorder) + data)


    def _write(self, addr, data):
        """ Generic write to I2C slave. See description in I2C_Slave class. """
        if type(data) == int:
            data = bytes([data])
        elif type(data) == list:
            data = bytes(data)

        self.mcp.I2C_write(addr, data)


    #### SMBUS interface ####

    def open(self, bus):
        """
        (For compatibility only, no effects)
        Open a given i2c bus.

        :param bus: i2c bus number (e.g. 0 or 1)
            or an absolute file path (e.g. '/dev/i2c-42').
        :type bus: int or str
        :raise TypeError: if type(bus) is not in (int, str)
        """
        pass


    def close(self):
        """
        (For compatibility only, no effects)
        Close the i2c connection.
        """
        pass


    def read_byte(self, i2c_addr, force=None):
        """
        Read a single byte from a device.

        :rtype: int
        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param force:
        :type force: Boolean
        :return: Read byte value
        """
        out = self._read(i2c_addr)
        return unpack("B", out)[0]


    def write_byte(self, i2c_addr, value, force=None):
        """
        Write a single byte to a device.

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param value: value to write
        :type value: int
        :param force:
        :type force: Boolean
        """
        self._write(i2c_addr, value)


    def read_byte_data(self, i2c_addr, register, force=None, data_type="uint", enable_pec=False):
        """
        Read a single byte from a designated register.

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param register: Register to read
        :type register: int
        :param force:
        :type force: Boolean
        :return: Read byte value
        :rtype: int
        """
        length = 2 if enable_pec else 1
        out = self._read_register(i2c_addr, register, length=length)

        data_bytes = out[:1]  # first byte - actual data
        pec_byte = out[1] if enable_pec else None  # second byte - PEC

        format = "b" if data_type == "int" else "B"
        data = unpack(format, data_bytes)[0]

        return data, pec_byte


    def write_byte_data(self, i2c_addr, register, value, force=None, data_type="uint", enable_pec=True, endianness="little", override_CRC=False):
        """
        Write a byte to a given register.

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param register: Register to write to
        :type register: int
        :param value: Byte value to transmit
        :type value: int
        :param force:
        :type force: Boolean
        :rtype: None
        """
        format = "b" if data_type == "int" else "B"
        data = pack(format, value)
        pec_value = None
        if enable_pec:
            pec_value = crc_calc_smbus(
                i2c_addr=i2c_addr,
                value=value,
                register=register,
                size=1,
                data_type=data_type,
                endianness=endianness,
                write_cmd=True,
            )
            if override_CRC:
                # generate a number different from pec_value
                pec_value = (pec_value + random.randint(1, 255)) % 255

            data += pack("B", pec_value)
        self._write_register(i2c_addr, register, data)
        return pec_value

    def read_word_data(self, i2c_addr, register, force=None, endianness="little", data_type="uint", enable_pec=False):
        """
        Read a single word (2 bytes) from a given register.

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param register: Register to read
        :type register: int
        :param force:
        :type force: Boolean
        :return: 2-byte word
        :rtype: int
        """
        length = 3 if enable_pec else 2
        out = self._read_register(i2c_addr, register, length=length)



        data_bytes = out[:2]  # first 2 bytes - actual data
        pec_byte = out[2] if enable_pec else None  # third byte - PEC

        endianness_format = ">" if endianness == "big_endian" else "<"
        data_format = "h" if data_type == "int" else "H"
        format = f"{endianness_format}{data_format}"

        data = unpack(format, data_bytes)[0]
        return data, pec_byte


    def write_word_data(self, i2c_addr, register, value, force=None, endianness="little", data_type="uint", enable_pec=False, override_CRC=False):
        """
        Write a single word (2 bytes) to a given register.

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param register: Register to write to
        :type register: int
        :param value: Word value to transmit
        :type value: int
        :param force:
        :type force: Boolean
        :rtype: None
        """
        endianness_format = ">" if endianness == "big_endian" else "<"
        data_format = "h" if data_type == "int" else "H"

        format = f"{endianness_format}{data_format}"
        data = pack(format, value)

        pec_value = None
        if enable_pec:
            pec_value = crc_calc_smbus(
                i2c_addr=i2c_addr,
                value=value,
                register=register,
                size=2,
                data_type=data_type,
                endianness=endianness,
                write_cmd=True,
            )
            if override_CRC:
                pec_value = (pec_value + random.randint(1, 255)) % 255

            data += pack("B", pec_value)

        self._write_register(i2c_addr, register, data)

        return pec_value

    def process_call(self, i2c_addr, register, value, force=None):
        """
        Executes a SMBus Process Call, sending a 16-bit value and receiving a 16-bit response

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param register: Register to read/write to
        :type register: int
        :param value: Word value to transmit
        :type value: int
        :param force:
        :type force: Boolean
        :rtype: int
        """
        data = pack("h", value)
        self.mcp.I2C_write(i2c_addr, register + data, kind = 'nonstop')

        data = self.mcp.I2C_read(i2c_addr, length = 2, kind = 'restart')
        return unpack("h", data)[0]


    def read_block_data(self, i2c_addr, register, force=None):
        """
        Read a block of up to 32-bytes from a given register.

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param register: Start register
        :type register: int
        :param force:
        :type force: Boolean
        :return: List of bytes
        :rtype: list
        """
        out = self._read_register(i2c_addr, register, length = I2C_SMBUS_BLOCK_MAX)
        length = out[0]
        return out[1:1+length]


    def write_block_data(self, i2c_addr, register, data, force=None):
        """
        Write a block of byte data to a given register.

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param register: Start register
        :type register: int
        :param data: List of bytes
        :type data: list
        :param force:
        :type force: Boolean
        :rtype: None
        """
        length = len(data)
        if length > I2C_SMBUS_BLOCK_MAX:
            raise ValueError("Data length cannot exceed %d bytes" % I2C_SMBUS_BLOCK_MAX)

        self._write_register(i2c_addr, register, bytes([length]) + data)


    def block_process_call(self, i2c_addr, register, data, force=None):
        """
        Executes a SMBus Block Process Call, sending a variable-size data
        block and receiving another variable-size response

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param register: Register to read/write to
        :type register: int
        :param data: List of bytes
        :type data: list
        :param force:
        :type force: Boolean
        :return: List of bytes
        :rtype: list
        """
        length = len(data)
        self.mcp.I2C_write(i2c_addr, register + bytes([length]) + data, kind = 'nonstop')

        out = self.mcp.I2C_read(i2c_addr, length = I2C_SMBUS_BLOCK_MAX, kind = 'restart')
        length = out[0]
        return out[1:1+length]


    def read_i2c_block_data(self, i2c_addr, register, length, force=None):
        """
        Read a block of byte data from a given register.

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param register: Start register
        :type register: int
        :param length: Desired block length
        :type length: int
        :param force:
        :type force: Boolean
        :return: List of bytes
        :rtype: list
        """
        if length > I2C_SMBUS_BLOCK_MAX:
            raise ValueError("Desired block length over %d bytes" % I2C_SMBUS_BLOCK_MAX)

        out = self._read_register(i2c_addr, register, length = length)
        return out


    def write_i2c_block_data(self, i2c_addr, register, data, force=None):
        """
        Write a block of byte data to a given register.

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param register: Start register
        :type register: int
        :param data: List of bytes
        :type data: list
        :param force:
        :type force: Boolean
        :rtype: None
        """
        length = len(data)
        if length > I2C_SMBUS_BLOCK_MAX:
            raise ValueError("Data length cannot exceed %d bytes" % I2C_SMBUS_BLOCK_MAX)

        self._write_register(i2c_addr, register, data)
