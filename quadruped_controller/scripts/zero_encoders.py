import subprocess
import argparse


def run_command(leg_id):
    numbers = [leg_id*10, leg_id*10 + 1, leg_id*10 + 2]
    numbers = [leg_id*10]
    for number in numbers:
        command = f"mdtool config zero {number} && mdtool config save {number}"
        subprocess.run(command, shell=True)


def main(args=None):
    parser = argparse.ArgumentParser(description="Zero encoders for a specific leg.")
    parser.add_argument("leg_id", type=int, help="ID of the leg to zero encoders for")
    args = parser.parse_args(args)

    leg_id = args.leg_id
    run_command(leg_id)

if __name__ == "__main__":
    main()
