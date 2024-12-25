


class PID:
    """
    A class to implement a simple PID (Proportional, Integral, Derivative) controller.
    
    Attributes:
        target (float): The target value that the PID controller is trying to reach.
        kp (float): The proportional gain constant.
        ki (float): The integral gain constant.
        kd (float): The derivative gain constant.
        p_error_history (list): A history of the proportional errors for integral and derivative calculations.
    """

    def __init__(self, target, kp: float, ki: float, kd: float):
        """
        Initializes the PID controller with the target value and the PID constants.

        Args:
            target (float): The target value the controller aims to reach.
            kp (float): The proportional gain constant.
            ki (float): The integral gain constant.
            kd (float): The derivative gain constant.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.p_error_history = []

    def error(self, target, current):
        """
        Calculates the error between the target and the current value.

        Args:
            target (float): The target value.
            current (float): The current value.

        Returns:
            float: The difference between the target and current values.
        """
        return target - current

    def get_p_error(self, target, current):
        """
        Gets the proportional error (current error between target and current value).

        Args:
            target (float): The target value.
            current (float): The current value.

        Returns:
            float: The proportional error.
        """
        return self.error(target, current)

    def get_i_error(self, error_history):
        """
        Calculates the integral error by summing all past proportional errors.

        Args:
            error_history (list): The history of the proportional errors.

        Returns:
            float: The sum of all previous errors.
        """
        return sum(error_history)

    def get_d_error(self, error_last, error_before_last):
        """
        Calculates the derivative error, which is the difference between the last and the second-last error.

        Args:
            error_last (float): The most recent error.
            error_before_last (float): The error before the last one.

        Returns:
            float: The rate of change of the error.
        """
        return self.error(error_last, error_before_last)

    def __call__(self, current):
        """
        Calculates the PID output given the current value.

        This method updates the errors (P, I, D) and computes the PID output
        using the formula:
            PID_output = (Kp * P_error) + (Ki * I_error) + (Kd * D_error)

        Args:
            current (float): The current value being measured.

        Returns:
            float: The calculated PID output.
        """
        # Calculate proportional error
        self.current_p_error = self.get_p_error(self.target, current)
        self.p_error_history.append(self.current_p_error)

        # Calculate integral error
        self.current_i_error = self.get_i_error(self.p_error_history)

        # Calculate derivative error (requires at least two past errors)
        if len(self.p_error_history) >= 2:
            self.current_d_error = self.get_d_error(
                self.p_error_history[-1], self.p_error_history[-2])
        else:
            self.current_d_error = 0  # No derivative error for the first step

        # Calculate the PID output using the formula: P + I + D
        pid_output = (self.kp * self.current_p_error +
                      self.ki * self.current_i_error +
                      self.kd * self.current_d_error)
        return pid_output
