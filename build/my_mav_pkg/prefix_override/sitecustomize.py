import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/raft-controller/Rpi_Ard_ser/install/my_mav_pkg'
