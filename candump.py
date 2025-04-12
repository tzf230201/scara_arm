import subprocess
import time

def read_and_write_candump_streaming(candump_command, output_file):
    # Run the candump command using subprocess to capture the output in real-time
    process = subprocess.Popen(candump_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Open the output file to write the candump data
    with open(output_file, 'a') as f:  # Open file in append mode to avoid overwriting
        try:
            # Read the candump data line by line in streaming mode
            for line in iter(process.stdout.readline, b''):  # Read line by line as it comes
                # Decode the byte string to a regular string and strip extra spaces
                line_str = line.decode('utf-8').strip()
                
                # Get the current timestamp
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
                
                # Write the timestamp and the candump data to the output file
                f.write(f"{timestamp} - {line_str}\n")
                f.flush()  # Ensure that the data is written to the file immediately
        except KeyboardInterrupt:
            # Handle manual interruption
            print("Process interrupted.")
        finally:
            # Ensure the process ends
            process.terminate()

if __name__ == "__main__":
    # Specify the candump command and the desired output text file name
    candump_command = ['candump', 'can0']  # replace 'can0' with your CAN interface if necessary
    output_file = "output_candump.txt"  # replace with your desired output file path
    
    # Call the function to read and write the streaming data
    read_and_write_candump_streaming(candump_command, output_file)
    print(f"Streaming data from candump has been written to {output_file}")
