import csv
import os

class Parser:
    def __init__(self):
        pass

    def parse(self, file):
        '''
        Function to parse GUI generated script files
        returns a list of commands where:
            list[0] - command
            list[1] - device address
            list[2] - register address
            list[3] - register value
            list[4] - mask
        '''
        if os.path.exists(file):
            type = self._detect_script_type(file)
        else:
            raise Exception("Script file does not exist")

        infile = open(file, 'r')

        if type == 'csv':
            return self._parse_csv(infile)
        elif type == 'cpp':
            return self._parse_cpp(infile)
        

    def _parse_cpp(self, file):
        cmd_list = []
        for row in file.read().splitlines():
            if len(row) != 0 and row.startswith('0x'):
                cmd = [None] * 5
                row = row.split(",")[:-1]
                if row[0] == '0x00':
                    cmd[0] = "delay"
                    cmd[1] = int(row[1], 16)
                else:
                    byte_len = int(row[0], 16)
                    row_len = len(row)
                    cmd[1] = int(row[1], 16) #addr field
                    if byte_len == row_len - 1:
                        if int(row[0], 16) == 3:
                            cmd[0] = "W1"
                            cmd[2] = int(row[2], 16)
                            cmd[3] = int(row[3], 16)
                        if int(row[0], 16) == 4:
                            cmd[0] = "W2"
                            cmd[2] = int(row[2]+row[3][2:], 16)
                            cmd[3] = int(row[4], 16)
                    else:
                        cmd[0] = "RMW"
                        cmd[2] = int(row[2]+row[3][2:], 16)
                        cmd[3] = int(row[4], 16)
                        cmd[4] = int(row[5], 16)
                cmd_list.append(cmd)
        return cmd_list

    def _parse_csv(self, file):
        cmd_list = []
        csvreader = csv.reader(file)
        for row in csvreader:
           if len(row) != 0 and row[0].startswith('0x'):
                cmd = [None] * 5
                if row[0] == 'delay':
                    cmd[0] = row[0]
                    cmd[1] = int(row[1], 16)
                else:
                    row_len = len(row)
                    cmd[1] = int(row[0], 16) #addr field
                    if row_len == 3:
                        cmd[0] = "W2"
                        cmd[2] = int(row[1], 16)
                        cmd[3] = int(row[2], 16)
                    elif row_len == 4:
                        cmd[0] = "RMW"
                        cmd[2] = int(row[1], 16)
                        cmd[3] = int(row[2], 16)
                        cmd[4] = int(row[3], 16)
                cmd_list.append(cmd)
        return cmd_list

    def _detect_script_type(self, file):
        root,ext = os.path.splitext(file)
        if ext in ['.cpp', '.csv']:
            return ext[-3:]
        
