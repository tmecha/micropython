

def integrate_second_discrete(start_vel=0, start_dist=0, time_units='sec', *, end_vel, time_period, func, func_args):
    """
    Integrates an acceleration profile to return distance travelled, number of time intervals, and time to end velocity.
    Performs discrete double integration via riemann sum right endpoint method
    :return: tuple with distance for speed change, final velocity, and intervals
    """

    dt = time_period

    # todo: Unit Conversions
    # if time_units == 'sec':  # Seconds
    #     dt = time_period
    # if time_units == 'min':  # Minutes
    #     dt = time_period / 60
    # if time_units == 'ms':  # Milliseconds
    #     dt = time_period / 1000

    time = 0
    vel = start_vel
    dist = start_dist
    intervals = 0
    while vel <= end_vel:
        vel += func(time=time, args=func_args) * dt  # First Integral
        dist += vel * dt  # second integral
        time += dt
        intervals += 1

    # todo: Adjust +- 1 intervals if speed is not perfect divisor of acceleration
    #  if (end_vel-start_vel) % accel > 0:
    #       #adjust dist / intervals / vel?

    return dist, vel, intervals


def linear_accel(*, time, args):
    """
    Linear acceleration trajectory function.
    Both parameters must have common time. i.e. minutes for both or seconds for both
    :param time: time in min
    :param kwargs: Dict that must contain 'accel' key for acceleration in mm/min^2
    :return: mm/min
    """
    if 'accel' in args:
        accel = args['accel']
        return accel
    else:
        raise ValueError("'accel' arg dict entry required")


if __name__ == '__main__':
    print("First Integral")
    speed, intervals = integrate_discrete(end_val=2000, time_period_ms=10, func=linear_accel, func_args={'accel': 1000})
    print("Speed: ", speed, " Intervals: ", intervals, "Time (mins):", intervals*10/60000)

    print("Second Integral Mins")
    dist, speed, intervals = integrate_second_discrete(end_vel=2000, time_period=.010/60, time_units='min', func=linear_accel, func_args={'accel': 1000})
    print("Dist: ", dist, " Speed: ", speed, " Intervals: ", intervals, "Time (mins):", intervals*.01/60/60)

    print("Second Integral Secs")
    dist, speed, intervals = integrate_second_discrete(end_vel=2, time_period=.01, time_units='sec', func=linear_accel, func_args={'accel': 1})
    print("Dist: ", dist, " Speed: ", speed, " Intervals: ", intervals, "Time (secs):", intervals/100, "Time (mins):", intervals/6000)
