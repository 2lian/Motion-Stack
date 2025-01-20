"""simple csv utilities"""

import csv
from os import path
from typing import Dict, Optional, Tuple

def update_csv(file_path, new_str: str, new_float: float) -> Tuple[str, Optional[str]]:
    rows = []
    str_found = False
    file_path = path.expanduser(file_path)
    row_save = None

    if not path.exists(file_path):
        # Create the file and write the header
        with open(file_path, mode="w", newline="") as file:
            writer = csv.writer(file)

    with open(file_path, mode="r") as file:
        reader = csv.reader(file)
        for row in reader:
            if row and row[0] == new_str:
                # If the string is found, update the float value
                row_save = row[1]
                row[1] = str(new_float)  # useless ?
                str_found = True
            rows.append(row)

    # If the string is not found, append a new row
    if not str_found:
        rows.append([new_str, str(new_float)])

    # Write the updated data back to the CSV file
    with open(file_path, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerows(rows)

    return new_str, row_save


def csv_to_dict(file_path) -> Optional[Dict[str, float]]:
    if not path.exists(file_path):
        return None
    data_dict = {}

    # Open the CSV file in read mode
    with open(file_path, mode="r") as file:
        reader = csv.reader(file)

        # Iterate through each row in the CSV
        for row in reader:
            # Assuming the first column is string and the second column is float
            data_dict[row[0]] = float(row[1])

    return data_dict

