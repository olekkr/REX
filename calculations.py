from constants import Constants


def calc_travel_time(dist_cm: float, power: int):
    return dist_cm / calc_speed_cm_s(power)


def calc_speed_cm_s(power: int):
    # Power is 0-based
    return Constants.Robot.MAX_RPM * ((power + 1) / 128) * (Constants.Robot.WHEEL_CIRCUMFERENCE / 60)


if __name__ == "__main__":
    print(calc_travel_time(constants.ROBOT_CIRCUMFERENCE * 0.95, 64))
    print(calc_travel_time(constants.ROBOT_CIRCUMFERENCE * 0.05, 5))
