import argparse

def accelerometer_arguments(parser: argparse.ArgumentParser):
    parser.add_argument("-x", action="store_true")
    parser.add_argument("-y", action="store_true")
    parser.add_argument("-z", action="store_true")
    return parser.parse_args()