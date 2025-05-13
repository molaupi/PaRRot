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

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Modify stops.csv in RAPTOR/csv directory.")
    parser.add_argument('--csv_directory', help="Path to the stops.csv", required=True)

    args = parser.parse_args()

    modify_stops(args.csv_directory)

