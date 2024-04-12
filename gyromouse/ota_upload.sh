#!/bin/sh

START=$(date +%s);

tftp 88.200.89.248 << EOF
verbose
trace
mode octet
rexmt 1
put firmware/blink.bin firmware.bin
q
EOF

END=$(date +%s);
echo ""
echo $((END-START)) | awk '{print "\nduration: "int($1%60) " sec\n"}'
