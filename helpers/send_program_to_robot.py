import argparse
import os
import socket

urscript_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "urscript")


def preprocess(code):
    preprocessed_code = ""
    for line in code.splitlines():
        if line.startswith("{{") and line.endswith("}}"):
            script_id = line.strip().strip("{{").strip("}}")
            script_filename = script_id + ".script"
            if not script_filename in os.listdir(urscript_dir):
                raise RuntimeError("Invalid script substitution: {} not found".format(script_filename))
            with open(os.path.join(urscript_dir, script_filename)) as script_file:
                script_code = script_file.read()
                processed_script = preprocess(script_code)
                preprocessed_code += processed_script + "\n"
        else:
            preprocessed_code += line + "\n"
    return preprocessed_code


def main(args):
    if not os.path.exists(args.program_path) or not os.path.isfile(args.program_path):
        raise ValueError("Could not find program file {}".format(args.program_path))
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((args.robot_ip, 30002))
    with open(args.program_path) as program_file:
        program_code = preprocess(program_file.read())
        s.sendall(program_code)
    s.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("program_path", type=str)
    parser.add_argument("robot_ip", type=str)
    main(parser.parse_args())
