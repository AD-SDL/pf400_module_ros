
class ConnectionException(Exception):
    def __init__(self, err_message = "error"):
        # Call the base class constructor with the parameters it needs
        super(ConnectionException, self).__init__("Could not establish connection! Error type: " + err_message)

class CommandException(Exception):
    def __init__(self, err_message = "error"):
        super(CommandException, self).__init__(
            "Invalid command! Check if communication is open. Error type: " + err_message
        )

