import argparse

def telemetry_arguments(parser: argparse.ArgumentParser):
    parser.add_argument("-o", "--ok", action="store_true")
    parser.add_argument("-s", "--sampling-state", action="store_true")
    parser.add_argument("-c", "--temperature-control", action="store_true")
    parser.add_argument("-T", "--target-temp", action="store_true")
    parser.add_argument("-t", "--current-temp", action="store_true")
    parser.add_argument("-p", "--current-pressure", action="store_true")
    parser.add_argument("-H", "--current-humidity", action="store_true")
    parser.add_argument("-v", "--battery-voltage", action="store_true")
    return parser.parse_args()
