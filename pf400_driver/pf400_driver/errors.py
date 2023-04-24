
class ConnectionException(Exception):
    def __init__(self, message="Could not establish connection"):
        # Call the base class constructor with the parameters it needs
        super(ConnectionException, self).__init__(message)