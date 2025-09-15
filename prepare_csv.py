#!/usr/bin/python3

import os
import pandas as pd
import argparse

def modify_stops(csv_directory):
    input_file = os.path.join(csv_directory, "stops.csv")
    output_file = os.path.join(csv_directory, "modified_stops.csv")

    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(input_file)

    # Concatenate stop_lat and stop_lon into a new column 'latlon'
    df['latlon'] = '(' + df['Latitude'].astype(str) + '|' + df['Longitude'].astype(str) + ')'

    # Drop the specified columns
    df = df.drop(['Latitude', 'Longitude', 'MinChangeTime', 'Name'], axis=1)

    # Save the modified DataFrame to a new CSV file
    df.to_csv(output_file, index=False)


def merge_files(karri_requests_path, ultra_requests_path, transformed_requests_path, output_path):
    """
    Merges two CSV files line-by-line using pandas.
    Takes origin, destination from ultra_requests and req_time from karri_requests.
    """
    # Read the two CSV files into pandas DataFrames
    df1 = pd.read_csv(karri_requests_path)
    df2 = pd.read_csv(ultra_requests_path)
    df3 = pd.read_csv(transformed_requests_path)

    # Take 'origin' and 'destination' from df2, and 'req_time' from df1
    merged_df = pd.DataFrame({
        'origin': df3['origin'],
        'destination': df3['destination'],
        'req_time': df1['req_time'],
        'sourceId': df2['sourceId'],
        'targetId': df2['targetId']
    })

    # Save the new DataFrame to a CSV file
    merged_df.to_csv(output_path, index=False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process or merge CSV files.")
    parser.add_argument('--mode', choices=['modify_stops', 'merge_files'], required=True,
                        help="Choose a mode: 'modify_stops' or 'merge_files'.")

    # Arguments for modify_stops mode
    parser.add_argument('--csv_directory', help="Path to the directory containing stops.csv.")

    # Arguments for merge_files mode
    parser.add_argument('--karri_requests', help="Path to the KaRRi requests CSV file (with req_time).")
    parser.add_argument('--ultra_requests', help="Path to the ULTRA requests CSV file (with sourceId, targetId).")
    parser.add_argument('--transformed_requests', help="Path to the transformed ULTRA requests CSV file (with origin, destination).")
    parser.add_argument('--output', help="Path for the merged output file.")

    args = parser.parse_args()

    if args.mode == 'modify_stops':
        if not args.csv_directory:
            parser.error("--csv_directory is required for 'modify_stops' mode.")
        modify_stops(args.csv_directory)
    elif args.mode == 'merge_files':
        if not all([args.karri_requests, args.ultra_requests, args.transformed_requests, args.output]):
            parser.error("--karri_requests, --ultra_requests, --transformed_requests, and --output are required for 'merge_files' mode.")
        merge_files(args.karri_requests, args.ultra_requests, args.transformed_requests, args.output)