import argparse
import sys
import time

from xpc import XPlaneConnect

import scenic.syntax.translator as translator

condition_drefs = ["sim/weather/runway_friction",
                   "sim/weather/wind_direction_degt[0]",
                   "sim/weather/wind_speed_kt[0]"]


def run_sampler(sampler, xp_client, samples=10):

    for _ in range(samples):
        scene, _ = sampler.generate()
        params = scene.params

        wind_speed = scene.params['wind_speed']
        wind_direction = scene.params['wind_direction']
        friction = scene.params['friction']
        print(wind_speed, wind_direction, friction)
        xp_client.sendDREFs(condition_drefs, [friction, wind_direction, wind_speed])
        time.sleep(5)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--scenario', help='Scenic scenario to run', default='conditions.sc')
    args = parser.parse_args()
    sampler = translator.scenarioFromFile(args.scenario)
    client = XPlaneConnect()
    run_sampler(sampler, client, samples=10)
