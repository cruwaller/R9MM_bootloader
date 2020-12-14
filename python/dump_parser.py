#!/usr/bin/env python
import sys, os, re

if len(sys.argv) < 2:
    print("Input file is missing");
    exit -1

output = []


with open(sys.argv[1], 'r') as _input:
    for line in _input:
        res = re.search('\(0x[0-9A-Fa-f]+\)', line)
        if res:
            num = eval(res.group(0))
            output.append(num)

out_file_name = "binary.bin"
if 2 < len(sys.argv):
    out_file_name = sys.argv[2]

print output
print "%s" % bytes(output)
if output:
    with open(out_file_name, 'wb') as _output:
        _output.write(bytearray(output));
