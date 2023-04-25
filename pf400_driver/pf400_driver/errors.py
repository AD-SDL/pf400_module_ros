
class ConnectionException(Exception):
    def __init__(self, err_message = "error"):
        # Call the base class constructor with the parameters it needs
        super(ConnectionException, self).__init__("Could not establish connection! Error type: " + err_message)

class CommandException(Exception):
    def __init__(self, err_message = "error"):
        super(CommandException, self).__init__(
            "Invalid command! Check if communication is open. Error type: " + err_message
        )

class ErrorResponse(Exception):
    """Error during command execution.."""

    @staticmethod
    def from_error_code(error: str):
        error_codes = {
                "-1009":"*No robot attached*",
                "-1012":"*Joint out-of-range* Set robot joints within their range",
                "-1039":"*Position too close* Robot 1",
                "-1040":"*Position too far* Robot 1",
                "-1042":"*Can't change robot config* Robot 1",
                "-1046":"*Power not enabled*",
                "-1600":"*Power off requested*",
                "-2800":"*Warning Parameter Mismatch*",
                "-2801":"*Warning No Parameters",
                "-2802":"*Warning Illegal move command*",
                "-2803":"*Warning Invalid joint angles*",
                "-2804":"*Warning: Invalid Cartesian coordinate values*",
                "-2805":"*Unknown command*  ",
                "-2806":"*Command Exception*",
                "-2807":"*Warning cannot set Input states*",
                "-2808":"*Not allowed by this thread*",
                "-2809":"*Invalid robot type*",
                "-2810":"*Invalid serial command*",
                "-2811":"*Invalid robot number*",
                "-2812":"*Robot already selected*",
                "-2813":"*Module not initialized*",
                "-2814":"*Invalid location index*",
                "-2816":"*Undefined location*",
                "-2817":"*Undefined profile*",
                "-2818":"*Undefined pallet*",
                "-2819":"*Pallet not supported*",
                "-2820":"*Invalid station index*",
                "-2821":"*Undefined station*",
                "-2822":"*Not a pallet*",
                "-2823":"*Not at pallet origin*",    
                "-3122":"*Soft envelope error* Robot 1: 1"   
        }
        if error not in error_codes:
            return ErrorResponse(f"Unknown error code: {error}")
        return ErrorResponse(error_codes[error])
    
