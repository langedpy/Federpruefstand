import minimalmodbus

class Eurotherm2408(minimalmodbus.instrument):
    """Instrument class for Eurotherm 3500 process controller.
    Args:
            * portname (str): port name
            * slaveaddress (int): slave address in the range 1 to 247

    """

    def __init__(self, port, slaveaddress):
        minimalmodbus.CLOSE_PORT_AFTER_EACH_CALL = True
        minimalmodbus.Instrument.__init__(self, port, slaveaddress)

    def get_pv(self):
        """
        Read out temperature (process value (PV)) from Eurotherm.
        """
        return(instrument.read_register(1, 1)*10)  # Registernumber, number of decimals

    def set_sp(self, setpoint):
        """
        Define the Eurotherm's temperature setpoint.

        Args:
            setpoint (float): Setpoint (most often in degrees)
        """
        instrument.write_register(2, setpoint / 10, 1)  # setpoint (2, X, 1) festlegen
