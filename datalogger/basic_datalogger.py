import serial
from serial.tools import list_ports
import pandas as pd
from datetime import datetime
import time
from pathlib import Path
from tqdm import trange, tqdm
import glob
import matplotlib.pyplot as plt


lerp_command = 'm 8 512 4'
disable_pid = 'p 1'
enable_pid = 'P 1'
move_command = 'S 1 '
get_enc = 'e 1'
#lerp_command = 'n 8 60 0'

n_trials = 512
n_steps = 128


tag = 'undershoot_ramp_faster_propstep5_halferror_512_first_last_fullrange_motorstep_128'

def main():
    arduino_ports = [p.device for p in list_ports.comports()]
    print(arduino_ports)
    if not arduino_ports:
        raise IOError("No Arduino found")
    if len(arduino_ports) > 1:
        print('Multiple Arduinos found - using the first')
    else:
        print('Using Arduino found at : {}'.format(arduino_ports[0]))

    # establish serial communication
    arduino_serial = serial.Serial(arduino_ports[0],2000000, timeout = 0.5)
    time.sleep(0.2)
    print('Serial Connection Open')

    # open file
    file = open(str(Path.home()) + "/Downloads/" + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + "_" + tag + ".csv", "w+")
    file.write("start,setpoint,encoder,dt\n")

    try:
        for j in trange(n_steps):
            # move to target
            motor_target = 3 - (j/(n_steps-1)) * 6
            print(f"Motor target: {motor_target} mm")
            print(move_command + f"{motor_target:.2f}")
            # enable PID
            #arduino_serial.write(enable_pid.encode())
            # move to target
            arduino_serial.write((move_command + f"{motor_target:.2f}").encode())
            time.sleep(5)
            # disable PID
            arduino_serial.write(disable_pid.encode())
            time.sleep(0.5)
            arduino_serial.flushInput()
            arduino_serial.flushOutput()
            time.sleep(5)

            for i in trange(n_trials):
                arduino_serial.write(lerp_command.encode())

                with tqdm(desc="Receiving", unit=" lines", total=512) as pbar:
                    n_fault = 0
                    while True:
                        line = arduino_serial.readline()
                        if line:
                            line = line.decode("utf-8")
                            if ',' in line: # only save first and last trials to csv
                                if (i == 0 or i == (n_trials - 1)):
                                    file.write(line)
                                pbar.update(1)
                        else:
                            n_fault += 1
                            if n_fault > 15:
                                pbar.close()
                                break
                # arduino_serial.flushInput()
                # arduino_serial.flushOutput()
    except KeyboardInterrupt:
        pass

    arduino_serial.close()
    print("Serial closed")
    file.close()
    print("File closed")

def plot():
    # Get most recent CSV
    path = str(Path.home()) + "/Downloads/**_" + tag + ".csv" 
    files = glob.glob(path)
    files.sort()
    newest = files[-1]
    print(newest)
    
    df = pd.read_csv(newest)
    # df = df[:5*512]

    df['inl'] = df['encoder'] - (df['setpoint'])
    df['inl nm'] = df['inl'] * 0.3 * 10**6 / (200 * 256)
    # df['enc norm'] = sp_max * (-df['encoder']-en_max)/(en_max-en_min)
    df['setpoint'] = df['setpoint'] - df['start']
    df['encoder'] = df['encoder'] - df['start']

    savepath = path.rsplit('**', 1)[0]
    savepath += tag + '_plot.png'

    fig, axes = plt.subplots(nrows=2, ncols=1, figsize=(200, 16))
    df.plot.line(y='inl nm', ax=axes[0])
    #axes[0].hlines(y=15, xmin=0, xmax=len(df), colors='r', linestyles='--', lw=1)
    #axes[0].hlines(y=-15, xmin=0, xmax=len(df), colors='r', linestyles='--', lw=1)
    axes[0].set_title('Integral Nonlinearity (nm)')
    df.plot.line( y=['setpoint', 'encoder'], ax=axes[1])
    axes[1].set_title('Setpoint')
    plt.tight_layout()

    plt.savefig(savepath)


if __name__ == "__main__":
    main()
    plot()