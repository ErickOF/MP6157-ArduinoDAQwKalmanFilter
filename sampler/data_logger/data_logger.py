"""data_logger
"""
import logging
import os

from logging import Logger
from serial import Serial


class DataLogger:
    """DataLogger class to get data from Arduino
    """
    def __init__(
            self,
            port: str = 'COM3',
            baudrate: int = 115200,
            timeout: float = 0.1,
            log_filename: str = 'arduino_data.log',
            data_filename: str = 'arduino_data.csv'
        ):
        """Constructor

        Args:
            port (str, optional): serial port to connect. Defaults to 'COM3'.
            baudrate (int, optional): serial port baudrate. Defaults to 115200.
            timeout (float, optional): serial port timeout. Defaults to 0.1.
            log_filename (str, optional): filename of the log file. Default to 'arduino_data.log'.
            data_filename (str, optional): filename of the CSV. Default to 'arduino_data.csv'.
        """
        self.__log_dir: str = 'logs'
        self.__data_dir: str = 'data'
        self.__data_filename: str = os.path.join(self.__data_dir, data_filename)
        self.__log_filename: str = os.path.join(self.__log_dir, log_filename)

        # Create log and data dir
        if not os.path.exists(self.__log_dir):
            os.mkdir(self.__log_dir)

        if not os.path.exists(self.__data_dir):
            os.mkdir(self.__data_dir)

        # Create data file
        if not os.path.exists(self.__data_filename):
            with open(self.__data_filename, 'w', encoding='utf-8') as file:
                file.write('Time (t),Data')

        # Serial connection
        self.__connection: Serial = Serial(port=port, baudrate=baudrate, timeout=timeout)

        # Log file
        self.__logger: Logger = logging.getLogger(__name__)
        self.__logger.setLevel(logging.NOTSET)
        logging.basicConfig(filename=self.__log_filename)

    def read(self, size: int|None = -1) -> bytes:
        """Read data from serial port
        Args:
            size (int | None, optional): number of bytes to read. Defaults to -1.

        Returns:
            bytes: read data
        """
        data: bytes = self.__connection.readline(size)
        self.__logger.info('Sampling data: %s', data)

        return data

    def write(self, data: str) -> (int | None):
        """Output the given str over the serial port

        Args:
            data (str): data to write

        Returns:
            (int | None): written bytes
        """
        self.__connection.write(bytes(data, 'utf-8'))


if __name__ == '__main__':
    dataLogger = DataLogger()
    dataLogger.start()
