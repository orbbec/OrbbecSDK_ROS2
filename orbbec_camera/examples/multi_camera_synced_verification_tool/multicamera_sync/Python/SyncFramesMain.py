import os
import subprocess
import json
import platform

def list_subdirectories(path):
    for f in os.listdir(path):
        if os.path.isdir(os.path.join(path, f)):
            yield f

def read_pid_from_device_json(frames_dir):
    file_path = frames_dir + "/DevicesInfo.txt"
    if not os.path.exists(file_path):
        print(file_path + " not exists. Please check it")
        return ""

    with open(file_path,  'r') as f:
        data = json.load(f)

    if not 'devicePid' in data:
        print("Not found 'devicePid' in " + file_path)
        return ""

    return data['devicePid']

def execute_sync_width_pid(frames_dir, pid):
    sync_script_file=""
    if pid.lower() == "0x0675":
        sync_script_file="script/config_frameMatch_Gemini2VL.py"
    elif pid.lower() == "0x0670" or pid.lower() == "0x0701":
        sync_script_file="script/config_frameMatch-systemTimestamp.py"
    elif pid.lower() == "0x0660":
        sync_script_file="script/config_frameMatch-Astra2_new.py"
    elif pid.lower() == "0x0800" or pid.lower() == "0x0804" or pid.lower() == "0x0803" or pid.lower() == "0x0807" or pid.lower() == "0x0801" or pid.lower() == "0x0805" or pid.lower() == "0x080b" or pid.lower() == "0x080e" or pid.lower() == "0x080f" or pid.lower() == "0x0A13":
        sync_script_file="script/config_frameMatch_g330.py"
    else:
        sync_script_file="script/config_frameMatch.py"
    
    if len(sync_script_file) > 0:
        print(f"execute synchronize frames. script_file={sync_script_file}, frames_dir={frames_dir}")
        if platform.system() == "Windows": 
            output = subprocess.check_output(["python", sync_script_file, os.path.abspath(f"{frames_dir}"), os.getcwd()])
            print(output.decode("gbk"))
        else:
            output = subprocess.check_output(["python3", sync_script_file, os.path.abspath(f"{frames_dir}"), os.getcwd()])
            print(output.decode("utf-8"))
    else:
        print("Invalid pid=" + pid + ", not match sync frame script")

def sync_frames(frames_dir):
    pid = read_pid_from_device_json(frames_dir)
    if len(pid) <= 0:
        print("Get pid failed.")
        return
    
    execute_sync_width_pid(frames_dir, pid)

def main():
    frames_output_path = os.path.abspath("../output")

    subdirectories = list(list_subdirectories(frames_output_path))

    if not subdirectories:
        print("No subdirectories found in the specified directory.")

    for i, subdir in enumerate(subdirectories):
        print(f"{i + 1}. {subdir}")

    choice = input("Please select a subdirectory by number (or 'q' to quit): ")

    if choice.lower() == 'q':
        return

    is_index_valid = False
    try:
        index = int(choice) - 1
        if 0 <= index < len(subdirectories):
            is_index_valid = True
        else:
            print(f"Invalid choice. index={index}. Please enter a number between 1 and the total number of subdirectories.")
    except ValueError:
        print("Invalid choice(ValueException). Please enter a number between 1 and the total number of subdirectories")
    if is_index_valid:
        selected_subdir = subdirectories[index]
        print(f"You selected the subdirectory: {selected_subdir}")
        sync_frames(f"{frames_output_path }/{selected_subdir}")


if __name__ == "__main__":
    main()
