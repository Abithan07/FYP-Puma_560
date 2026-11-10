import csv
import os
import pickle
import numpy as np

def csv_to_custom_dict(filename):
    """Convert one CSV file into the custom dictionary structure."""
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        rows = list(reader)

    keys = [row[0] for row in rows]
    values = [list(map(float, row[1:])) for row in rows]

    result = {}
    # First key: store as a single array (no nested list)
    result[keys[0][-1]] = np.array(values[0])

    # Combine every 3 keys after the first
    group_index = 1
    i=0
    key_names = ["qp","qv","tau","qa","m","c","g"]
    while group_index < len(keys):
        group = keys[group_index:group_index + 3]
        group_values = values[group_index:group_index + 3]
        if len(group) < 3:
            break
        combined = np.array(list(map(list, zip(*group_values))))  # transpose
        result[key_names[i]] = combined
        group_index += 3
        i +=1

    return result


def combine_csv_dicts(file_list):
    """Combine multiple CSVs into a single dictionary with grouped arrays."""
    combined_dict = {}
    combined_dict["labels"] = []
    i=1

    for file in file_list:
        single_dict = csv_to_custom_dict(file)
        combined_dict["labels"].append(str(i))
        i = i+1

        for key, val in single_dict.items():
            if key not in combined_dict:
                combined_dict[key] = []
            combined_dict[key].append(val)
    print("Number of trajectories: " + str(len(combined_dict["labels"])))
    return combined_dict


def save_as_pickle(data, output_path="combined_data.pickle"):
    """Save the combined dictionary as a pickle file."""
    with open(output_path, "wb") as f:
        pickle.dump(data, f)
    print(f"Pickle file saved as: {output_path}")


if __name__ == "__main__":
    # Example usage: combine all CSV files in a folder
    folder = "./data/100_trajectory_torques_rows"
    files = [os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(".csv")]

    final_dict = combine_csv_dicts(files)
    save_as_pickle(final_dict, f"{folder}/combined_data.pickle")