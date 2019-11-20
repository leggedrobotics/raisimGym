#! /usr/bin/python3
import subprocess
import argparse
import os
from termcolor import colored

parser = argparse.ArgumentParser()
parser.add_argument('env', type=str,
                    help='Enviroment folder path.')

args = parser.parse_args()
current_path = os.getcwd()
absolute_path = os.path.realpath(os.path.join(current_path, args.env))

if not os.path.exists(absolute_path + "/Environment.hpp"):
    print(colored("[ERROR]: The env folder should contain Environment.hpp", 'red'))
    exit(0)

dirname = os.path.dirname(os.path.abspath(__file__))
root = os.path.realpath(dirname + "/../")
setup_path = root + "/setup.py"
os.chdir(root)

command = ["python3", setup_path, "install", "--env", absolute_path, "--user"]
res = subprocess.run(command)
