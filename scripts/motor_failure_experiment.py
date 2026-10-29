#!/usr/bin/env python3
"""Run a motor-efficiency experiment on an already running simulation."""

import argparse
import csv
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from laser_gazebo_resources.msg import MotorFailureCommand, MotorState


def sim_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True, help='Model prefix printed by the motor plugin')
    parser.add_argument('--case', type=int, choices=range(1, 6), required=True)
    parser.add_argument('--motor-id', type=int, choices=range(4), default=2)
    parser.add_argument('--start-after', type=float, default=5.0, help='Simulation seconds')
    parser.add_argument('--transition-duration', type=float, default=5.0)
    parser.add_argument('--record-after', type=float, default=5.0)
    parser.add_argument('--ground-truth-topic', help='Defaults to /<model>/ground_truth')
    parser.add_argument('--output', type=Path, required=True, help='New result directory')
    args = parser.parse_args()
    for name in ('start_after', 'transition_duration', 'record_after'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be finite and positive')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('motor_failure_experiment')
    qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
    prefix = f'/{args.model.strip("/")}'
    states = {}
    odometry_count = 0
    errors = []
    efficiencies = []
    nominal_thrusts = []
    request_counter = time.time_ns()
    active_request = None
    command_fields = [
        'motor_id', 'request_id', 'last_request_id', 'request_accepted', 'request_status',
        'command_mode', 'command_received', 'command_limited', 'efficiency', 'omega',
        'nominal_thrust', 'actual_thrust', 'nominal_reaction_torque',
        'actual_reaction_torque', 'loads_valid', 'bem_solver_fallback',
        'legacy_failure_active', 'scheduled', 'transitioning', 'failure_start_time',
    ]
    result = {'case': args.case, 'model': args.model, 'motor_id': args.motor_id}
    motor_file = (args.output / 'motors.csv').open('w', newline='')
    vehicle_file = (args.output / 'vehicle.csv').open('w', newline='')
    motor_csv = csv.writer(motor_file)
    vehicle_csv = csv.writer(vehicle_file)
    motor_csv.writerow(['sim_time', 'frame_id', *command_fields])
    vehicle_csv.writerow([
        'sim_time', 'frame_id', 'child_frame_id', 'x', 'y', 'z',
        'qx', 'qy', 'qz', 'qw', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz'])

    def on_state(message):
        states[message.motor_id] = message
        motor_csv.writerow([sim_seconds(message), message.header.frame_id,
                            *(getattr(message, name) for name in command_fields)])
        if active_request is None:
            return
        if not message.loads_valid or message.legacy_failure_active:
            errors.append('Invalid loads or legacy motor stop active')
        for actual, nominal in (
            (message.actual_thrust, message.nominal_thrust),
            (message.actual_reaction_torque, message.nominal_reaction_torque),
        ):
            if not math.isclose(actual, message.efficiency * nominal,
                                rel_tol=1e-9, abs_tol=1e-10):
                errors.append('Force/torque efficiency relationship failed')
        if message.motor_id != args.motor_id and message.efficiency != 1.0:
            errors.append('A non-target rotor changed efficiency')
        if message.motor_id == args.motor_id:
            efficiencies.append(message.efficiency)
            nominal_thrusts.append(message.nominal_thrust)

    def on_odometry(message):
        nonlocal odometry_count
        odometry_count += 1
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v, w = message.twist.twist.linear, message.twist.twist.angular
        vehicle_csv.writerow([
            sim_seconds(message), message.header.frame_id, message.child_frame_id,
            p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z])

    publisher = node.create_publisher(
        MotorFailureCommand, prefix + '/simulation/motor_failure', 32)
    node.create_subscription(MotorState, prefix + '/simulation/motor_state', on_state, qos)
    node.create_subscription(
        Odometry, args.ground_truth_topic or prefix + '/ground_truth', on_odometry, qos)

    def wait_until(predicate, timeout=20.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not predicate():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    'Timed out waiting for ROS discovery, acknowledgment or simulation progress')

    def send(motor_id, target, start=0.0, duration=0.0):
        nonlocal request_counter
        request_counter += 1
        message = MotorFailureCommand(
            motor_id=motor_id, target_efficiency=float(target),
            failure_start_time=float(start), transition_duration=float(duration),
            failure_type=MotorFailureCommand.LINEAR if duration else MotorFailureCommand.ABRUPT,
            request_id=request_counter)
        publisher.publish(message)
        wait_until(lambda: states[motor_id].last_request_id == message.request_id)
        if not states[motor_id].request_accepted:
            raise RuntimeError(states[motor_id].request_status)
        return message.request_id

    try:
        wait_until(lambda: publisher.get_subscription_count() >= 4 and
                   all(i in states for i in range(4)))
        if args.case == 4:
            wait_until(lambda: odometry_count > 0)
        # Explicitly establish the requested [1,1,1,1] initial condition.
        for motor_id in range(4):
            send(motor_id, 1.0)
        start = max(sim_seconds(state) for state in states.values()) + args.start_after
        target = 1.0 if args.case == 1 else (0.5 if args.case == 2 else 0.0)
        duration = args.transition_duration if args.case == 5 else 0.0
        active_request = send(args.motor_id, target, start, duration)
        result.update(failure_start_time=start, target_efficiency=target,
                      transition_duration=duration, request_id=active_request)
        end = start + duration + args.record_after
        previous_time = sim_seconds(states[args.motor_id])
        while previous_time < end:
            wait_until(lambda: sim_seconds(states[args.motor_id]) > previous_time)
            previous_time = sim_seconds(states[args.motor_id])
        final = states[args.motor_id]
        if (final.request_id != active_request or
                not math.isclose(final.efficiency, target, abs_tol=1e-9)):
            errors.append('Requested final efficiency was not reached')
        if not nominal_thrusts or max(nominal_thrusts) < 1e-6:
            errors.append('Rotor produced no nominal thrust; a stopped motor cannot validate loss')
        if args.case == 5 and not any(0.0 < value < 1.0 for value in efficiencies):
            errors.append('No intermediate ramp sample was recorded')
        if args.case == 4 and odometry_count < 2:
            errors.append('No vehicle response was recorded')
        result.update(passed=not errors, errors=sorted(set(errors)),
                      vehicle_samples=odometry_count, final_efficiency=final.efficiency)
        if errors:
            raise RuntimeError('; '.join(sorted(set(errors))))
        print(f'Experiment passed; records: {args.output}. Final efficiency remains {target}.')
    except BaseException as error:
        result.update(passed=False, error=str(error))
        raise
    finally:
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        motor_file.close()
        vehicle_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
