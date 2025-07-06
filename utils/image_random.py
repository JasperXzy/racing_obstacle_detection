import os
import random

def randomize_filenames(folder_path):
    """
    Randomize the filenames of all files in the specified folder.

    :param folder_path: Path to the folder containing files to be renamed.
    """
    files = os.listdir(folder_path)
    
    random.shuffle(files)
    
    for index, filename in enumerate(files):
        old_file_path = os.path.join(folder_path, filename)
        new_file_name = f"{index:04d}.jpg"
        new_file_path = os.path.join(folder_path, new_file_name)
        
        os.rename(old_file_path, new_file_path)
        print(f"Renamed {old_file_path} to {new_file_path}")

if __name__ == "__main__":
    folder_path = "../dataset/obstacle/images"
    randomize_filenames(folder_path)
    print("Filenames have been randomized.")
