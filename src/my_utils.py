import argparse
import os

def getargs():
    # Create ArgumentParser object
    parser = argparse.ArgumentParser(description='Example argparse usage')

    # Add positional arguments
    parser.add_argument('--input_dir', type=str, help='Path to the input file')
    # parser.add_argument('output_dir', type=str, help='Path to the output bag file')

    # Add optional arguments
    parser.add_argument('-g', '--gt_pose', action='store_true', help='Enable verbose mode')
    parser.add_argument('-j', '--joint', action='store_true', help='Enable verbose mode')
    parser.add_argument('-i', '--imu', action='store_true', help='Enable verbose mode')
    parser.add_argument('-e', '--event', action='store_true', help='Enable verbose mode')
    parser.add_argument('-r', '--rgb', action='store_true', help='Enable verbose mode')
    parser.add_argument('-d', '--depth', action='store_true', help='Enable verbose mode')
    # parser.add_argument('--threshold', type=float, default=0.5, help='Threshold value (default: 0.5)')

    # Parse the command-line argumen
    args = parser.parse_args()
    return args

def get_png_files(folder_path):
    pnj_files = [file for file in os.listdir(folder_path) if file.lower().endswith('.png')]
    return pnj_files 