import time

class PID():
    def __init__(self, K_p=0.0, K_i=0.0, K_d=0.0):
        """Constructor
        Args:
            K_p (float): The proportional gain
            K_i (float): The integral gain
            K_d (float): The derivative gain
        """
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        self.error_accumulator = 0

        self.reset()

    def reset(self):
        """Reset the PID controller"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.error_accumulator = 0.0

    def update(self, error, error_derivative=None):
        """Update the PID controller
        Args:
            error (float): The current error
        """
        current_time = time.time()
        dt = current_time - self.last_time

        if dt == 0:
            return 0.0

        self.last_time = current_time

        self.integral = self._get_integral(error, dt)
        if error_derivative is None:
            derivative = self._get_derivative(error, dt)
        else:
            derivative = error_derivative

        # K_i is multiplied in the _get_integral function. K_i is being taken into account.
        output = self.K_p * error + self.integral + self.K_d * derivative

        self.last_error = error

        return output

    def _get_integral(self, error, dt):
        """Calculate the integral term
        Args:
            error (float): The current error
            dt (float): The time delta
        Returns:
            float: The integral term
        """

        self.error_accumulator += error * dt # dt is the time since the last update
        integral = self.K_i * self.error_accumulator


        integral = min(self.K_i * self.error_accumulator, 1.0)

        return integral

    def _get_derivative(self, error, dt):
        """Calculate the derivative term
        Args:
            error (float): The current error
            dt (float): The time delta
        Returns:
            float: The derivative term
        """

        derivative = (error - self.last_error) / dt
        return derivative
