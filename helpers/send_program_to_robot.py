import argparse
import os
import socket


def main(args):
    if not os.path.exists(args.program_path) or not os.path.isfile(args.program_path):
        raise ValueError("Could not find program file {}".format(args.program_path))
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((args.robot_ip, 30002))
    with open(args.program_path) as program_file:
        s.sendall(program_file.read())
    s.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("program_path", type=str)
    parser.add_argument("robot_ip", type=str)
    main(parser.parse_args())